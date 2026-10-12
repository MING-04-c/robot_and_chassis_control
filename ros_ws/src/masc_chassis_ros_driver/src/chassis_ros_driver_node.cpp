/*
 * Copyright (C) 2026, MASC.
 * @Description: ROS1/ROS2 node for the MASC chassis CAN SDK.
 */
#include "masc_chassis_ros_driver/chassis_ros_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

#include "masc_chassis_can_sdk/chassis_protocol.hpp"

namespace masc_chassis_ros_driver {
namespace {

std::uint8_t ToRosCommState(ConnectionState state) {
    switch (state) {
        case ConnectionState::Disconnected:
            return rosmsg::CommStatus::COMM_STATE_DISCONNECTED;
        case ConnectionState::Connecting:
            return rosmsg::CommStatus::COMM_STATE_CONNECTING;
        case ConnectionState::ProtocolMismatch:
            return rosmsg::CommStatus::COMM_STATE_PROTOCOL_MISMATCH;
        case ConnectionState::Connected:
            return rosmsg::CommStatus::COMM_STATE_CONNECTED;
        case ConnectionState::Unknown:
        default:
            return rosmsg::CommStatus::COMM_STATE_UNKNOWN;
    }
}

std::uint8_t ToRosFailReason(FailReason reason) {
    switch (reason) {
        case FailReason::None:
            return rosmsg::CommStatus::FAIL_REASON_NONE;
        case FailReason::PortOpenFailed:
            return rosmsg::CommStatus::FAIL_REASON_PORT_OPEN_FAILED;
        case FailReason::VersionTimeout:
            return rosmsg::CommStatus::FAIL_REASON_VERSION_TIMEOUT;
        case FailReason::ProtocolVersionMismatch:
            return rosmsg::CommStatus::FAIL_REASON_PROTOCOL_VERSION_MISMATCH;
        case FailReason::ReadError:
            return rosmsg::CommStatus::FAIL_REASON_READ_ERROR;
        case FailReason::WriteError:
            return rosmsg::CommStatus::FAIL_REASON_WRITE_ERROR;
        case FailReason::TransportError:
            return rosmsg::CommStatus::FAIL_REASON_TRANSPORT_ERROR;
        case FailReason::Unknown:
        default:
            return rosmsg::CommStatus::FAIL_REASON_UNKNOWN;
    }
}

template <typename CacheT>
void UpdateLatestStamp(const CacheT &cache, RosTime *latest_stamp, bool *has_stamp) {
    if (!cache.m_valid) {
        return;
    }
    if (!*has_stamp || SecondsBetween(*latest_stamp, cache.m_stamp) > 0.0) {
        *latest_stamp = cache.m_stamp;
        *has_stamp = true;
    }
}

std::string FormatCanId(std::uint16_t can_id) {
    std::ostringstream stream;
    stream << "0x" << std::uppercase << std::hex << can_id;
    return stream.str();
}

bool IsAutoLockAllowedSystemState(const masc_chassis_can_sdk::SystemFeedback &feedback) {
    using SystemState = masc_chassis_can_sdk::SystemState;
    switch (feedback.m_system_state) {
        case SystemState::Unknown:
        case SystemState::Estop:
        case SystemState::Tow:
        case SystemState::Fault:
        case SystemState::SelfCheck:
        case SystemState::RemoteControl:
            return false;
        case SystemState::Park:
        case SystemState::CommControl:
            return true;
    }
    return false;
}

const char *AutoLockActionName(AutoLockAction action) {
    switch (action) {
        case AutoLockAction::Lock:
            return "auto lock";
        case AutoLockAction::Unlock:
            return "auto unlock";
        case AutoLockAction::None:
        default:
            return "auto lock";
    }
}

template <typename CacheArrayT>
void AppendDriverFaultGroup(const char *label, const CacheArrayT &cache_array,
                            std::ostringstream *stream, bool *has_group) {
    bool has_item = false;
    std::ostringstream group;
    group << label << "[";
    for (const auto &cache : cache_array) {
        if (!cache.m_valid || cache.m_data.m_driver_fault == 0u) {
            continue;
        }

        if (has_item) {
            group << ",";
        }
        group << cache.m_data.m_index << ":f=0x" << std::uppercase << std::hex << std::setw(8)
              << std::setfill('0') << static_cast<unsigned int>(cache.m_data.m_driver_fault)
              << std::dec << std::setfill(' ') << std::fixed << std::setprecision(1)
              << " mt=" << cache.m_data.m_motor_temperature_deg_c << "C"
              << " dt=" << cache.m_data.m_driver_temperature_deg_c << "C";
        has_item = true;
    }
    if (!has_item) {
        return;
    }

    *stream << (*has_group ? " " : " | Motor ") << group.str() << "]";
    *has_group = true;
}

std::string BuildMotorDriverFaultLog(const PowerWheelDriverCache &power_wheel_drivers,
                                     const ServoDriverCache &servo_drivers) {
    std::ostringstream stream;
    bool has_group = false;
    AppendDriverFaultGroup("drv", power_wheel_drivers, &stream, &has_group);
    AppendDriverFaultGroup("srv", servo_drivers, &stream, &has_group);
    return stream.str();
}

template <typename CacheArrayT, typename ArrayMsgT, typename AddMessageT>
void PublishIndexedFeedbackArray(const CacheArrayT &cache_array, std::uint64_t sequence,
                                 std::uint64_t *last_published_sequence,
                                 const RosPublisher<ArrayMsgT> &publisher, RosNodeBase *node,
                                 AddMessageT add_message) {
    if (last_published_sequence == nullptr || sequence == *last_published_sequence) {
        return;
    }

    ArrayMsgT array_msg;
    bool has_stamp = false;
    RosTime latest_stamp = Now(node);
    for (const auto &cache : cache_array) {
        if (!cache.m_valid) continue;
        add_message(&array_msg, cache);
        UpdateLatestStamp(cache, &latest_stamp, &has_stamp);
    }
    if (!has_stamp) {
        return;
    }

    array_msg.header = MakeHeader(latest_stamp);
    Publish(publisher, array_msg);
    *last_published_sequence = sequence;
}

}  // namespace

#if defined(MASC_CHASSIS_USE_ROS2)
ChassisRosDriverNode::ChassisRosDriverNode(const rclcpp::NodeOptions &options)
    : RosNodeBase("masc_chassis", options) {
    Initialize();
}
#else
ChassisRosDriverNode::ChassisRosDriverNode() : RosNodeBase("masc_chassis") {
    Initialize();
}
#endif

ChassisRosDriverNode::~ChassisRosDriverNode() {
    m_stop_requested = true;
    m_sdk.Disconnect();
    if (m_connect_thread.joinable()) {
        m_connect_thread.join();
    }
}

void ChassisRosDriverNode::Initialize() {
    // 初始化顺序：参数 -> 算法组件 -> ROS 接口 -> SDK 回调 -> 后台连接线程。
    LoadParameters();
    m_command_filter.Configure(m_command_filter_config);
    m_odom_integrator.Configure(m_odom_config);
    m_tracked_can_ids = BuildTrackedCanIds();
    SetupRosInterfaces();
    SetupSdkCallbacks();
    StartConnectThread();
    MCHASSIS_INFO(this, "MASC chassis ROS driver started");
}

void ChassisRosDriverNode::LoadParameters() {
    using masc_chassis_ros_driver::DeclareParameter;

    m_can_config.m_interface_name =
        DeclareParameter(this, "can.interface_name", std::string("can0"));
    m_can_config.m_bitrate = DeclareParameter(this, "can.bitrate", 500000);
    m_can_config.m_expected_protocol_version = static_cast<std::uint8_t>(
        DeclareParameter(this, "can.expected_protocol_version", 10));
    m_can_config.m_connect_timeout_ms = DeclareParameter(this, "can.connect_timeout_ms", 1000);
    m_auto_reconnect = DeclareParameter(this, "can.auto_reconnect", true);
    m_reconnect_period_ms = DeclareParameter(this, "can.reconnect_period_ms", 1000);

    m_can_config.m_motion_command_period_ms = DeclareParameter(this, "command.period_ms", 20);
    m_can_config.m_motion_command_timeout_ms = DeclareParameter(this, "command.timeout_ms", 100);
    m_stamped_control = DeclareParameter(this, "command.stamped_control", false);
    m_command_filter_config.m_timeout_ms = m_can_config.m_motion_command_timeout_ms;
    m_command_filter_config.m_max_linear_x =
        DeclareParameter(this, "command.max_linear_x", 1.5);
    m_command_filter_config.m_max_linear_y =
        DeclareParameter(this, "command.max_linear_y", 1.0);
    m_command_filter_config.m_max_angular_z =
        DeclareParameter(this, "command.max_angular_z", 2.0);
    m_command_filter_config.m_max_steering_angle =
        DeclareParameter(this, "command.max_steering_angle", 2.0);

    m_odom_config.m_odom_frame_id =
        DeclareParameter(this, "odom.frame_id", std::string("odom"));
    m_odom_config.m_base_frame_id =
        DeclareParameter(this, "odom.child_frame_id", std::string("base_link"));
    m_publish_tf = DeclareParameter(this, "odom.publish_tf", false);
    m_odom_config.m_linear_scale = DeclareParameter(this, "odom.linear_scale", 1.0);
    m_odom_config.m_angular_scale = DeclareParameter(this, "odom.angular_scale", 1.0);
    m_imu_frame_id = DeclareParameter(this, "imu.frame_id", std::string("imu_link"));

    m_odom_hz = DeclareParameter(this, "publish_rate.odom_hz", 100.0);
    m_chassis_status_hz = DeclareParameter(this, "publish_rate.chassis_status_hz", 10.0);
    m_motor_hz = DeclareParameter(this, "publish_rate.motor_hz", 50.0);
    m_motor_status_hz = DeclareParameter(this, "publish_rate.motor_status_hz", 10.0);
    m_imu_hz = DeclareParameter(this, "publish_rate.imu_hz", 50.0);
    m_rc_hz = DeclareParameter(this, "publish_rate.rc_hz", 20.0);
    m_bms_hz = DeclareParameter(this, "publish_rate.bms_hz", 1.0);
    m_comm_hz = DeclareParameter(this, "publish_rate.comm_hz", 1.0);
    m_frame_stats_hz = DeclareParameter(this, "publish_rate.frame_stats_hz", 1.0);

    m_motion_feedback_timeout_ms =
        DeclareParameter(this, "freshness.motion_feedback_timeout_ms", 200);
    m_system_feedback_timeout_ms =
        DeclareParameter(this, "freshness.system_feedback_timeout_ms", 500);
    m_bms_feedback_timeout_ms = DeclareParameter(this, "freshness.bms_feedback_timeout_ms", 2500);
    m_imu_feedback_timeout_ms = DeclareParameter(this, "freshness.imu_feedback_timeout_ms", 100);
    m_rc_feedback_timeout_ms = DeclareParameter(this, "freshness.rc_feedback_timeout_ms", 250);

    AutoLockConfig auto_lock_config;
    auto_lock_config.m_enabled = DeclareParameter(this, "auto_lock.enabled", true);
    auto_lock_config.m_delay_ms = DeclareParameter(this, "auto_lock.delay_ms", 3000);
    auto_lock_config.m_cmd_filter_alpha =
        DeclareParameter(this, "auto_lock.cmd_filter_alpha", 0.2);
    auto_lock_config.m_cmd_zero_linear_threshold =
        DeclareParameter(this, "auto_lock.cmd_zero_linear_threshold", 0.01);
    auto_lock_config.m_cmd_zero_angular_threshold =
        DeclareParameter(this, "auto_lock.cmd_zero_angular_threshold", 0.01);
    auto_lock_config.m_feedback_linear_stop_threshold =
        DeclareParameter(this, "auto_lock.feedback_linear_stop_threshold", 0.02);
    auto_lock_config.m_feedback_angular_stop_threshold =
        DeclareParameter(this, "auto_lock.feedback_angular_stop_threshold", 0.02);
    auto_lock_config.m_auto_unlock_on_cmd =
        DeclareParameter(this, "auto_lock.auto_unlock_on_cmd", true);
    m_auto_lock_controller.Configure(auto_lock_config);
}

void ChassisRosDriverNode::SetupRosInterfaces() {
    // 所有 ROS topic/service 都使用私有名称 ~/xxx，最终会挂在节点命名空间下。
    using masc_chassis_ros_driver::CreatePublisher;
    using masc_chassis_ros_driver::CreateService;
    using masc_chassis_ros_driver::CreateSubscription;
    using masc_chassis_ros_driver::CreateTimer;

    m_odom_publisher = CreatePublisher<rosmsg::Odometry>(this, "~/odom", 10);
    m_chassis_status_publisher =
        CreatePublisher<rosmsg::ChassisStatus>(this, "~/chassis_status", 10);
    m_bms_status_publisher = CreatePublisher<rosmsg::BmsStatus>(this, "~/bms_status", 10);
    m_comm_status_publisher = CreatePublisher<rosmsg::CommStatus>(this, "~/comm_status", 10);
    m_drive_motors_publisher =
        CreatePublisher<rosmsg::DriveMotorArray>(this, "~/drive_motors", 10);
    m_drive_motor_status_publisher =
        CreatePublisher<rosmsg::DriveMotorStatusArray>(this, "~/drive_motor_status", 10);
    m_servo_motors_publisher =
        CreatePublisher<rosmsg::ServoMotorArray>(this, "~/servo_motors", 10);
    m_servo_motor_status_publisher =
        CreatePublisher<rosmsg::ServoMotorStatusArray>(this, "~/servo_motor_status", 10);
    m_imu_publisher = CreatePublisher<rosmsg::Imu>(this, "~/imu", 10);
    m_rc_status_publisher = CreatePublisher<rosmsg::RcStatus>(this, "~/rc_status", 10);
    m_can_frame_stats_publisher =
        CreatePublisher<rosmsg::CanFrameStatsArray>(this, "~/can_frame_stats", 5);

    if (m_stamped_control) {
        m_cmd_vel_stamped_subscription = CreateSubscription<rosmsg::TwistStamped>(
            this, "~/cmd_vel", 10,
            [this](const rosmsg::TwistStamped &msg) { CmdVelStampedCallback(msg); });
    } else {
        m_cmd_vel_subscription = CreateSubscription<rosmsg::Twist>(
            this, "~/cmd_vel", 10, [this](const rosmsg::Twist &msg) { CmdVelCallback(msg); });
    }

    m_set_motion_mode_service = CreateService<rosmsg::SetMotionMode>(
        this, "~/set_motion_mode",
        [this](const rosmsg::SetMotionMode::Request &request,
               rosmsg::SetMotionMode::Response &response) {
            HandleSetMotionMode(request, response);
        });
    m_set_tow_mode_service = CreateService<rosmsg::SetTowMode>(
        this, "~/set_tow_mode",
        [this](const rosmsg::SetTowMode::Request &request,
               rosmsg::SetTowMode::Response &response) {
            HandleSetTowMode(request, response);
        });
    m_clear_fault_service = CreateService<rosmsg::ClearFault>(
        this, "~/clear_fault",
        [this](const rosmsg::ClearFault::Request &request,
               rosmsg::ClearFault::Response &response) { HandleClearFault(request, response); });
    m_calibrate_servo_service = CreateService<rosmsg::CalibrateServo>(
        this, "~/calibrate_servo",
        [this](const rosmsg::CalibrateServo::Request &request,
               rosmsg::CalibrateServo::Response &response) {
            HandleCalibrateServo(request, response);
        });

    m_command_timer = CreateTimer(this, std::chrono::milliseconds(m_can_config.m_motion_command_period_ms),
                                  [this]() { CommandTimerCallback(); });
    m_odom_timer = CreateTimer(this, PeriodFromHz(m_odom_hz), [this]() { PublishOdom(); });
    m_chassis_status_timer = CreateTimer(this, PeriodFromHz(m_chassis_status_hz),
                                         [this]() { PublishChassisStatus(); });
    m_bms_timer = CreateTimer(this, PeriodFromHz(m_bms_hz), [this]() { PublishBmsStatus(); });
    m_comm_timer = CreateTimer(this, PeriodFromHz(m_comm_hz), [this]() { PublishCommStatus(); });
    m_motor_timer = CreateTimer(this, PeriodFromHz(m_motor_hz), [this]() {
        PublishDriveMotors();
        PublishServoMotors();
    });
    m_motor_status_timer = CreateTimer(this, PeriodFromHz(m_motor_status_hz), [this]() {
        PublishDriveMotorStatus();
        PublishServoMotorStatus();
    });
    m_imu_timer = CreateTimer(this, PeriodFromHz(m_imu_hz), [this]() { PublishImu(); });
    m_rc_timer = CreateTimer(this, PeriodFromHz(m_rc_hz), [this]() { PublishRcStatus(); });
    m_frame_stats_timer =
        CreateTimer(this, PeriodFromHz(m_frame_stats_hz), [this]() { PublishCanFrameStats(); });

    if (m_publish_tf) {
#if defined(MASC_CHASSIS_USE_ROS2)
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
#else
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>();
#endif
    }
}

void ChassisRosDriverNode::SetupSdkCallbacks() {
    // SDK 回调运行在 CAN 接收线程中；这里只做轻量的“打时间戳并写缓存”。
    m_sdk.SetMotionFeedbackCallback([this](const masc_chassis_can_sdk::MotionFeedback &feedback) {
        m_feedback_cache.SetMotionFeedback(feedback, Now(this));
    });
    m_sdk.SetSystemFeedbackCallback([this](const masc_chassis_can_sdk::SystemFeedback &feedback) {
        m_feedback_cache.SetSystemFeedback(feedback, Now(this));
    });
    m_sdk.SetMotionModeFeedbackCallback(
        [this](const masc_chassis_can_sdk::MotionModeFeedback &feedback) {
            m_feedback_cache.SetMotionModeFeedback(feedback, Now(this));
        });
    m_sdk.SetBmsBasicFeedbackCallback(
        [this](const masc_chassis_can_sdk::BmsBasicFeedback &feedback) {
            m_feedback_cache.SetBmsBasicFeedback(feedback, Now(this));
        });
    m_sdk.SetBmsFaultFeedbackCallback(
        [this](const masc_chassis_can_sdk::BmsFaultFeedback &feedback) {
            m_feedback_cache.SetBmsFaultFeedback(feedback, Now(this));
        });
    m_sdk.SetPowerWheelFeedbackCallback(
        [this](const masc_chassis_can_sdk::PowerWheelFeedback &feedback) {
            m_feedback_cache.SetPowerWheelFeedback(feedback, Now(this));
        });
    m_sdk.SetPowerWheelDriverFeedbackCallback(
        [this](const masc_chassis_can_sdk::PowerWheelDriverFeedback &feedback) {
            m_feedback_cache.SetPowerWheelDriverFeedback(feedback, Now(this));
        });
    m_sdk.SetServoFeedbackCallback([this](const masc_chassis_can_sdk::ServoFeedback &feedback) {
        m_feedback_cache.SetServoFeedback(feedback, Now(this));
    });
    m_sdk.SetServoDriverFeedbackCallback(
        [this](const masc_chassis_can_sdk::ServoDriverFeedback &feedback) {
            m_feedback_cache.SetServoDriverFeedback(feedback, Now(this));
        });
    m_sdk.SetImuFeedbackCallback([this](const masc_chassis_can_sdk::ImuFeedback &feedback) {
        m_feedback_cache.SetImuFeedback(feedback, Now(this));
    });
    m_sdk.SetRcFeedbackCallback([this](const masc_chassis_can_sdk::RcFeedback &feedback) {
        m_feedback_cache.SetRcFeedback(feedback, Now(this));
    });
}

void ChassisRosDriverNode::StartConnectThread() {
    m_stop_requested = false;
    m_connect_thread = std::thread(&ChassisRosDriverNode::ConnectLoop, this);
}

void ChassisRosDriverNode::ConnectLoop() {
    // 连接线程独立于 ROS spin：失败后按配置重试，成功后监视 SDK 是否掉线。
    while (!m_stop_requested.load()) {
        SetConnectionState(ConnectionState::Connecting, FailReason::None);
        if (!m_sdk.Configure(m_can_config)) {
            m_sdk.Disconnect();
            m_sdk.Configure(m_can_config);
        }

        const bool connected = m_sdk.Connect();
        if (m_stop_requested.load()) {
            break;
        }

        if (connected) {
            SetConnectionState(ConnectionState::Connected, FailReason::None);
            MCHASSIS_INFO(this, "Connected to chassis CAN interface %s",
                          m_can_config.m_interface_name.c_str());
            while (!m_stop_requested.load() && m_sdk.IsConnected()) {
                SleepInterruptible(std::chrono::milliseconds(200));
            }
            SetConnectionState(ConnectionState::Disconnected, FailReason::TransportError);
        } else {
            const FailReason reason = DetermineConnectFailReason();
            const ConnectionState state = reason == FailReason::ProtocolVersionMismatch
                                              ? ConnectionState::ProtocolMismatch
                                              : ConnectionState::Disconnected;
            SetConnectionState(state, reason);
            MCHASSIS_WARN(this, "Failed to connect chassis CAN interface %s",
                          m_can_config.m_interface_name.c_str());
        }

        if (!m_auto_reconnect) {
            break;
        }
        SleepInterruptible(std::chrono::milliseconds(m_reconnect_period_ms));
    }
}

void ChassisRosDriverNode::SleepInterruptible(std::chrono::milliseconds duration) const {
    constexpr auto kStep = std::chrono::milliseconds(50);
    auto slept = std::chrono::milliseconds(0);
    while (!m_stop_requested.load() && slept < duration) {
        const auto remain = duration - slept;
        const auto step = remain < kStep ? remain : kStep;
        std::this_thread::sleep_for(step);
        slept += step;
    }
}

void ChassisRosDriverNode::CmdVelCallback(const rosmsg::Twist &msg) {
    std::lock_guard<std::mutex> lock(m_command_mutex);
    m_latest_cmd_vel = msg;
    m_last_cmd_vel_time = Now(this);
    m_has_cmd_vel = true;
}

void ChassisRosDriverNode::CmdVelStampedCallback(const rosmsg::TwistStamped &msg) {
    CmdVelCallback(msg.twist);
}

void ChassisRosDriverNode::CommandTimerCallback() {
    // 控制主循环：读取最新 cmd_vel -> 限幅/超时 -> 自动驻车决策 -> 交给 SDK 周期发送。
    if (!m_sdk.IsConnected()) {
        m_auto_lock_controller.Reset();
        return;
    }

    rosmsg::Twist twist;
    RosTime stamp;
    bool has_command = false;
    {
        std::lock_guard<std::mutex> lock(m_command_mutex);
        twist = m_latest_cmd_vel;
        stamp = m_last_cmd_vel_time;
        has_command = m_has_cmd_vel;
    }

    const auto now = Now(this);
    auto command = m_command_filter.BuildCommand(twist, has_command, stamp, now);
    const auto auto_lock_result = m_auto_lock_controller.Update(BuildAutoLockInput(command, now));
    SendAutoLockMotionModeRequest(auto_lock_result);
    m_sdk.SetMotionCommand(auto_lock_result.m_command);
}

AutoLockUpdateInput ChassisRosDriverNode::BuildAutoLockInput(
    const masc_chassis_can_sdk::MotionCommand &command, const RosTime &now) const {
    const auto motion = m_feedback_cache.GetMotionFeedback();
    const auto mode = m_feedback_cache.GetMotionModeFeedback();
    const auto system = m_feedback_cache.GetSystemFeedback();

    AutoLockUpdateInput input;
    input.m_command = command;
    input.m_motion_mode_switch_supported = m_sdk.IsMotionModeSwitchSupported();
    input.m_now = now;
    if (motion.m_valid && !IsCacheStale(motion, m_motion_feedback_timeout_ms)) {
        input.m_motion_feedback = motion.m_data;
    }
    if (mode.m_valid && !IsCacheStale(mode, m_system_feedback_timeout_ms)) {
        input.m_motion_mode_feedback = mode.m_data;
    }
    if (system.m_valid && !IsCacheStale(system, m_system_feedback_timeout_ms)) {
        input.m_auto_lock_allowed = IsAutoLockAllowedSystemState(system.m_data);
    }
    return input;
}

void ChassisRosDriverNode::SendAutoLockMotionModeRequest(
    const AutoLockUpdateResult &result) {
    if (!result.m_motion_mode_request.has_value()) {
        return;
    }

    const char *action = AutoLockActionName(result.m_action);
    const bool sent = m_sdk.SendMotionMode(*result.m_motion_mode_request);
    if (sent) {
        MCHASSIS_INFO(this, "%s motion mode command sent", action);
    } else {
        MCHASSIS_WARN(this, "%s motion mode command failed", action);
    }
    m_auto_lock_controller.HandleMotionModeRequestResult(result.m_action, sent);
}

void ChassisRosDriverNode::PublishOdom() {
    // ROS 定时器只检查缓存；真正的新数据由 SDK callback 产生 sequence 后才发布。
    const auto motion = m_feedback_cache.GetMotionFeedback();

    if (!motion.m_valid || IsCacheStale(motion, m_motion_feedback_timeout_ms)) {
        return;
    }
    if (motion.m_sequence == m_last_odom_sequence) {
        return;
    }

    m_last_odom = m_odom_integrator.Integrate(motion.m_data, motion.m_stamp);
    m_last_odom_sequence = motion.m_sequence;
    Publish(m_odom_publisher, m_last_odom);
    if (m_tf_broadcaster) {
        m_tf_broadcaster->sendTransform(m_odom_integrator.BuildTransform(m_last_odom));
    }
}

void ChassisRosDriverNode::PublishChassisStatus() {
    const auto system = m_feedback_cache.GetSystemFeedback();
    const auto mode = m_feedback_cache.GetMotionModeFeedback();
    if (system.m_sequence == m_last_chassis_status_system_sequence &&
        mode.m_sequence == m_last_chassis_status_mode_sequence) {
        return;
    }

    std::optional<masc_chassis_can_sdk::SystemFeedback> system_value;
    std::optional<masc_chassis_can_sdk::MotionModeFeedback> mode_value;
    RosTime stamp = Now(this);
    if (system.m_valid && !IsCacheStale(system, m_system_feedback_timeout_ms)) {
        system_value = system.m_data;
        stamp = system.m_stamp;
    }
    if (mode.m_valid && !IsCacheStale(mode, m_system_feedback_timeout_ms)) {
        mode_value = mode.m_data;
    }
    const auto msg = BuildChassisStatus(stamp, system_value, mode_value);
    LogChassisFaultIfNeeded(msg);
    Publish(m_chassis_status_publisher, msg);
    m_last_chassis_status_system_sequence = system.m_sequence;
    m_last_chassis_status_mode_sequence = mode.m_sequence;
}

void ChassisRosDriverNode::LogChassisFaultIfNeeded(const rosmsg::ChassisStatus &msg) {
    static constexpr double kChangedFaultLogIntervalSec = 1.0;
    static constexpr double kSameFaultLogIntervalSec = 10.0;

    if (msg.chassis_state != rosmsg::ChassisStatus::CHASSIS_STATE_FAULT) {
        m_chassis_fault_log_active = false;
        return;
    }

    const auto now = Now(this);
    const bool raw_value_changed =
        !m_chassis_fault_log_active || msg.chassis_fault != m_last_logged_chassis_fault ||
        msg.system_status_flags != m_last_logged_system_status_flags;
    if (m_chassis_fault_log_active) {
        const double elapsed_sec = SecondsBetween(m_last_chassis_fault_log_time, now);
        const double log_interval_sec =
            raw_value_changed ? kChangedFaultLogIntervalSec : kSameFaultLogIntervalSec;
        if (elapsed_sec < log_interval_sec) {
            return;
        }
    }

    const auto basic = m_feedback_cache.GetBmsBasicFeedback();
    const auto fault = m_feedback_cache.GetBmsFaultFeedback();
    const bool basic_timeout = !basic.m_valid || IsCacheStale(basic, m_bms_feedback_timeout_ms);
    const bool fault_timeout = !fault.m_valid || IsCacheStale(fault, m_bms_feedback_timeout_ms);
    std::optional<masc_chassis_can_sdk::BmsBasicFeedback> basic_value;
    std::optional<masc_chassis_can_sdk::BmsFaultFeedback> fault_value;
    if (!basic_timeout) {
        basic_value = basic.m_data;
    }
    if (!fault_timeout) {
        fault_value = fault.m_data;
    }
    const auto bms_msg = BuildBmsStatus(now, basic_timeout, basic_value, fault_value);
    const auto motor_fault_log =
        BuildMotorDriverFaultLog(m_feedback_cache.GetPowerWheelDriverFeedback().m_cache,
                                 m_feedback_cache.GetServoDriverFeedback().m_cache);

    MCHASSIS_WARN(this,
                  "Chassis FAULT fault=0x%08X sys=0x%08X | BMS health=%u soc=%u%% "
                  "v=%.1fV i=%.1fA tmax=%.1fC%s",
                  static_cast<unsigned int>(msg.chassis_fault),
                  static_cast<unsigned int>(msg.system_status_flags),
                  static_cast<unsigned int>(bms_msg.bms_health),
                  static_cast<unsigned int>(bms_msg.bat_soc),
                  bms_msg.bat_voltage,
                  bms_msg.bat_current,
                  bms_msg.bat_temp_max,
                  motor_fault_log.c_str());

    m_chassis_fault_log_active = true;
    m_last_logged_chassis_fault = msg.chassis_fault;
    m_last_logged_system_status_flags = msg.system_status_flags;
    m_last_chassis_fault_log_time = now;
}

void ChassisRosDriverNode::PublishBmsStatus() {
    const auto basic = m_feedback_cache.GetBmsBasicFeedback();
    const auto fault = m_feedback_cache.GetBmsFaultFeedback();
    if (basic.m_sequence == m_last_bms_basic_sequence &&
        fault.m_sequence == m_last_bms_fault_sequence) {
        return;
    }

    const bool basic_timeout = !basic.m_valid || IsCacheStale(basic, m_bms_feedback_timeout_ms);
    const bool fault_timeout = !fault.m_valid || IsCacheStale(fault, m_bms_feedback_timeout_ms);
    std::optional<masc_chassis_can_sdk::BmsBasicFeedback> basic_value;
    std::optional<masc_chassis_can_sdk::BmsFaultFeedback> fault_value;
    if (!basic_timeout) {
        basic_value = basic.m_data;
    }
    if (!fault_timeout) {
        fault_value = fault.m_data;
    }
    const RosTime stamp = !basic_timeout ? basic.m_stamp : Now(this);
    Publish(m_bms_status_publisher, BuildBmsStatus(stamp, basic_timeout, basic_value, fault_value));
    m_last_bms_basic_sequence = basic.m_sequence;
    m_last_bms_fault_sequence = fault.m_sequence;
}

void ChassisRosDriverNode::PublishCommStatus() {
    // 通信状态按固定频率发布，即使没有新 CAN 反馈，也能反映连接和错误计数变化。
    rosmsg::CommStatus msg;
    msg.header = MakeHeader(Now(this));

    ConnectionState state;
    FailReason reason;
    {
        std::lock_guard<std::mutex> lock(m_connection_mutex);
        state = m_connection_state;
        reason = m_fail_reason;
    }
    const auto health = m_sdk.GetHealthSnapshot();
    msg.comm_state = ToRosCommState(state);
    msg.fail_reason = ToRosFailReason(reason);
    msg.motion_command_timeout_active = health.m_motion_command_timeout_active;
    msg.connection_attempt_count = health.m_connection_attempt_count;
    msg.transport_error_count = health.m_transport_error_count;
    msg.tx_error_count = health.m_tx_error_count;
    msg.rx_error_count = health.m_rx_error_count;
    if (health.m_version_info.has_value()) {
        for (std::size_t i = 0; i < health.m_version_info->m_uuid.size(); ++i) {
            msg.chassis_uuid[i] = health.m_version_info->m_uuid[i];
        }
        msg.system_software_hardware_version =
            health.m_version_info->m_system_software_hardware_version;
        msg.chassis_type_version = health.m_version_info->m_chassis_type_version;
        msg.chassis_protocol_version = health.m_version_info->m_protocol_version;
    }
    Publish(m_comm_status_publisher, msg);
}

void ChassisRosDriverNode::PublishDriveMotors() {
    const auto snapshot = m_feedback_cache.GetPowerWheelFeedback();
    PublishIndexedFeedbackArray<PowerWheelCache, rosmsg::DriveMotorArray>(
        snapshot.m_cache, snapshot.m_sequence, &m_last_drive_motors_published_sequence,
        m_drive_motors_publisher, this, [](rosmsg::DriveMotorArray *msg,
           const StampedCache<masc_chassis_can_sdk::PowerWheelFeedback> &cache) {
            msg->motors.push_back(BuildDriveMotor(cache.m_stamp, cache.m_data));
        });
}

void ChassisRosDriverNode::PublishDriveMotorStatus() {
    const auto snapshot = m_feedback_cache.GetPowerWheelDriverFeedback();
    PublishIndexedFeedbackArray<PowerWheelDriverCache, rosmsg::DriveMotorStatusArray>(
        snapshot.m_cache, snapshot.m_sequence, &m_last_drive_motor_status_published_sequence,
        m_drive_motor_status_publisher, this,
        [](rosmsg::DriveMotorStatusArray *msg,
           const StampedCache<masc_chassis_can_sdk::PowerWheelDriverFeedback> &cache) {
            msg->status.push_back(BuildDriveMotorStatus(cache.m_stamp, cache.m_data));
        });
}

void ChassisRosDriverNode::PublishServoMotors() {
    const auto snapshot = m_feedback_cache.GetServoFeedback();
    PublishIndexedFeedbackArray<ServoCache, rosmsg::ServoMotorArray>(
        snapshot.m_cache, snapshot.m_sequence, &m_last_servo_motors_published_sequence,
        m_servo_motors_publisher, this, [](rosmsg::ServoMotorArray *msg,
           const StampedCache<masc_chassis_can_sdk::ServoFeedback> &cache) {
            msg->motors.push_back(BuildServoMotor(cache.m_stamp, cache.m_data));
        });
}

void ChassisRosDriverNode::PublishServoMotorStatus() {
    const auto snapshot = m_feedback_cache.GetServoDriverFeedback();
    PublishIndexedFeedbackArray<ServoDriverCache, rosmsg::ServoMotorStatusArray>(
        snapshot.m_cache, snapshot.m_sequence, &m_last_servo_motor_status_published_sequence,
        m_servo_motor_status_publisher, this,
        [](rosmsg::ServoMotorStatusArray *msg,
           const StampedCache<masc_chassis_can_sdk::ServoDriverFeedback> &cache) {
            msg->status.push_back(BuildServoMotorStatus(cache.m_stamp, cache.m_data));
        });
}

void ChassisRosDriverNode::PublishImu() {
    const auto imu = m_feedback_cache.GetImuFeedback();
    if (!imu.m_valid || IsCacheStale(imu, m_imu_feedback_timeout_ms)) {
        return;
    }
    if (imu.m_sequence == m_last_imu_sequence) {
        return;
    }
    Publish(m_imu_publisher, BuildImu(imu.m_stamp, m_imu_frame_id, imu.m_data));
    m_last_imu_sequence = imu.m_sequence;
}

void ChassisRosDriverNode::PublishRcStatus() {
    const auto rc = m_feedback_cache.GetRcFeedback();
    if (!rc.m_valid || IsCacheStale(rc, m_rc_feedback_timeout_ms)) {
        return;
    }
    if (rc.m_sequence == m_last_rc_status_sequence) {
        return;
    }
    Publish(m_rc_status_publisher, BuildRcStatus(rc.m_stamp, rc.m_data));
    m_last_rc_status_sequence = rc.m_sequence;
}

void ChassisRosDriverNode::PublishCanFrameStats() {
    rosmsg::CanFrameStatsArray array_msg;
    array_msg.header = MakeHeader(Now(this));
    const auto now = std::chrono::steady_clock::now();
    for (const auto can_id : m_tracked_can_ids) {
        const auto stats = m_sdk.GetFrameStats(can_id);
        if (!stats.has_value()) {
            continue;
        }
        rosmsg::CanFrameStats msg;
        msg.can_id = FormatCanId(stats->m_can_id);
        msg.rx_count = stats->m_rx_count;
        msg.parse_error_count = stats->m_parse_error_count;
        msg.drop_count = stats->m_drop_count;
        msg.frequency = stats->m_frequency_hz;
        if (stats->m_last_rx_time.has_value()) {
            msg.last_rx_age_ms = static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(now - *stats->m_last_rx_time)
                    .count());
        }
        array_msg.stats.push_back(msg);
    }
    Publish(m_can_frame_stats_publisher, array_msg);
}

void ChassisRosDriverNode::HandleSetMotionMode(
    const rosmsg::SetMotionMode::Request &request, rosmsg::SetMotionMode::Response &response) {
    if (!IsConnected()) {
        response.success = false;
        response.message = "chassis CAN SDK is not connected";
        return;
    }

    const auto mode = ToSdkMotionMode(request.motion_mode);
    if (!mode.has_value()) {
        response.success = false;
        response.message = "invalid motion mode";
        return;
    }
    if (!m_sdk.IsMotionModeSwitchSupported()) {
        response.success = false;
        response.message = "motion mode switch is unsupported by this chassis type";
        return;
    }
    response.success = m_sdk.SendMotionMode(*mode);
    response.message = response.success ? "motion mode command sent" : "failed to send motion mode";
}

void ChassisRosDriverNode::HandleSetTowMode(const rosmsg::SetTowMode::Request &request,
                                            rosmsg::SetTowMode::Response &response) {
    if (!IsConnected()) {
        response.success = false;
        response.message = "chassis CAN SDK is not connected";
        return;
    }
    response.success = m_sdk.SendTowMode(request.enable);
    response.message = response.success ? "tow mode command sent" : "failed to send tow mode";
}

void ChassisRosDriverNode::HandleClearFault(const rosmsg::ClearFault::Request &request,
                                            rosmsg::ClearFault::Response &response) {
    if (!IsConnected()) {
        response.success = false;
        response.message = "chassis CAN SDK is not connected";
        return;
    }
    response.success = m_sdk.SendFaultClear(request.fault_id);
    response.message = response.success ? "fault clear command sent" : "failed to send fault clear";
}

void ChassisRosDriverNode::HandleCalibrateServo(
    const rosmsg::CalibrateServo::Request &request, rosmsg::CalibrateServo::Response &response) {
    if (!IsConnected()) {
        response.success = false;
        response.message = "chassis CAN SDK is not connected";
        return;
    }
    response.success = m_sdk.SendCalibrateServo(request.servo_id);
    response.message =
        response.success ? "servo calibration command sent" : "failed to send servo calibration";
}

void ChassisRosDriverNode::SetConnectionState(ConnectionState state, FailReason reason) {
    std::lock_guard<std::mutex> lock(m_connection_mutex);
    m_connection_state = state;
    m_fail_reason = reason;
}

FailReason ChassisRosDriverNode::DetermineConnectFailReason() const {
    const auto health = m_sdk.GetHealthSnapshot();
    if (health.m_version_info.has_value() && !health.m_protocol_version_matched) {
        return FailReason::ProtocolVersionMismatch;
    }
    if (health.m_transport_error_count > 0) {
        return FailReason::TransportError;
    }
    return FailReason::VersionTimeout;
}

bool ChassisRosDriverNode::IsConnected() const {
    std::lock_guard<std::mutex> lock(m_connection_mutex);
    return m_connection_state == ConnectionState::Connected && m_sdk.IsConnected();
}

std::chrono::milliseconds ChassisRosDriverNode::PeriodFromHz(double hz) const {
    if (hz <= 0.0) {
        hz = 1.0;
    }
    return std::chrono::milliseconds(
        static_cast<int>(std::max(1.0, std::round(1000.0 / hz))));
}

std::vector<std::uint16_t> ChassisRosDriverNode::BuildTrackedCanIds() const {
    namespace protocol = masc_chassis_can_sdk::chassis_protocol;
    std::vector<std::uint16_t> ids = {
        protocol::kMotionFeedbackId,     protocol::kSystemFeedbackId,
        protocol::kVersionFeedbackId,    protocol::kBmsBasicFeedbackId,
        protocol::kBmsFaultFeedbackId,   protocol::kMotionModeFeedbackId,
        protocol::kImuFeedbackId,        protocol::kRcFeedbackId,
    };
    for (std::uint16_t id = protocol::kPowerWheelFeedbackBeginId;
         id <= protocol::kPowerWheelFeedbackEndId; ++id) {
        ids.push_back(id);
    }
    for (std::uint16_t id = protocol::kPowerWheelDriverFeedbackBeginId;
         id <= protocol::kPowerWheelDriverFeedbackEndId; ++id) {
        ids.push_back(id);
    }
    for (std::uint16_t id = protocol::kServoFeedbackBeginId; id <= protocol::kServoFeedbackEndId;
         ++id) {
        ids.push_back(id);
    }
    for (std::uint16_t id = protocol::kServoDriverFeedbackBeginId;
         id <= protocol::kServoDriverFeedbackEndId; ++id) {
        ids.push_back(id);
    }
    return ids;
}

}  // namespace masc_chassis_ros_driver
