/*
 * Copyright (C) 2026, MASC.
 * @Description: ROS1/ROS2 node for the MASC chassis CAN SDK.
 */
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <tf2_ros/transform_broadcaster.h>

#include "masc_chassis_can_sdk/chassis_sdk.hpp"
#include "masc_chassis_ros_driver/auto_lock_controller.hpp"
#include "masc_chassis_ros_driver/command_filter.hpp"
#include "masc_chassis_ros_driver/feedback_cache.hpp"
#include "masc_chassis_ros_driver/odom_integrator.hpp"
#include "masc_chassis_ros_driver/ros_api.hpp"
#include "masc_chassis_ros_driver/ros_conversions.hpp"
#include "masc_chassis_ros_driver/ros_msgs.hpp"

namespace masc_chassis_ros_driver {

// ROS 驱动的主节点：负责把 ROS2 的命令/服务转换为 SDK 控制命令，
// 再把 SDK 收到的 CAN 反馈转换成 ROS2 消息。

enum class ConnectionState : std::uint8_t {
    Unknown,
    Disconnected,
    Connecting,
    ProtocolMismatch,
    Connected,
};

enum class FailReason : std::uint8_t {
    None,
    PortOpenFailed,
    VersionTimeout,
    ProtocolVersionMismatch,
    ReadError,
    WriteError,
    TransportError,
    Unknown,
};

class ChassisRosDriverNode : public RosNodeBase {
public:
#if defined(MASC_CHASSIS_USE_ROS2)
    explicit ChassisRosDriverNode(const rclcpp::NodeOptions &options);
#else
    ChassisRosDriverNode();
#endif
    ~ChassisRosDriverNode() override;

private:
    void Initialize();
    void LoadParameters();
    void SetupRosInterfaces();
    void SetupSdkCallbacks();
    void StartConnectThread();
    void ConnectLoop();
    void SleepInterruptible(std::chrono::milliseconds duration) const;

    void CmdVelCallback(const rosmsg::Twist &msg);
    void CmdVelStampedCallback(const rosmsg::TwistStamped &msg);
    void CommandTimerCallback();
    AutoLockUpdateInput BuildAutoLockInput(const masc_chassis_can_sdk::MotionCommand &command,
                                           const RosTime &now) const;
    void SendAutoLockMotionModeRequest(const AutoLockUpdateResult &result);
    void PublishOdom();
    void PublishChassisStatus();
    void LogChassisFaultIfNeeded(const rosmsg::ChassisStatus &msg);
    void PublishBmsStatus();
    void PublishCommStatus();
    void PublishDriveMotors();
    void PublishDriveMotorStatus();
    void PublishServoMotors();
    void PublishServoMotorStatus();
    void PublishImu();
    void PublishRcStatus();
    void PublishCanFrameStats();

    void HandleSetMotionMode(const rosmsg::SetMotionMode::Request &request,
                             rosmsg::SetMotionMode::Response &response);
    void HandleSetTowMode(const rosmsg::SetTowMode::Request &request,
                          rosmsg::SetTowMode::Response &response);
    void HandleClearFault(const rosmsg::ClearFault::Request &request,
                          rosmsg::ClearFault::Response &response);
    void HandleCalibrateServo(const rosmsg::CalibrateServo::Request &request,
                              rosmsg::CalibrateServo::Response &response);

    void SetConnectionState(ConnectionState state, FailReason reason);
    FailReason DetermineConnectFailReason() const;
    bool IsConnected() const;
    std::chrono::milliseconds PeriodFromHz(double hz) const;
    std::vector<std::uint16_t> BuildTrackedCanIds() const;

    template <typename T>
    bool IsCacheStale(const StampedCache<T> &cache, int timeout_ms) const {
        if (!cache.m_valid) {
            return true;
        }
        return SecondsBetween(cache.m_stamp, Now(const_cast<ChassisRosDriverNode *>(this))) * 1000.0 >
               static_cast<double>(timeout_ms);
    }

    // 底层 CAN SDK：真正执行 SocketCAN 打开、收帧、解析和发帧。
    masc_chassis_can_sdk::ChassisSdk m_sdk;
    masc_chassis_can_sdk::CanConfig m_can_config;
    CommandFilter m_command_filter;
    CommandFilterConfig m_command_filter_config;
    AutoLockController m_auto_lock_controller;
    OdomIntegrator m_odom_integrator;
    OdomIntegratorConfig m_odom_config;
    FeedbackCache m_feedback_cache;

    bool m_auto_reconnect = true;
    int m_reconnect_period_ms = 1000;
    int m_motion_feedback_timeout_ms = 200;
    int m_system_feedback_timeout_ms = 500;
    int m_bms_feedback_timeout_ms = 2500;
    int m_imu_feedback_timeout_ms = 100;
    int m_rc_feedback_timeout_ms = 250;
    bool m_stamped_control = false;
    bool m_publish_tf = false;
    std::string m_imu_frame_id = "imu_link";
    double m_odom_hz = 100.0;
    double m_chassis_status_hz = 10.0;
    double m_motor_hz = 50.0;
    double m_motor_status_hz = 10.0;
    double m_imu_hz = 50.0;
    double m_rc_hz = 20.0;
    double m_bms_hz = 1.0;
    double m_comm_hz = 1.0;
    double m_frame_stats_hz = 1.0;

    // ROS2 发布器：向上层导航、监控和诊断节点发布底盘数据。
    RosPublisher<rosmsg::Odometry> m_odom_publisher;
    RosPublisher<rosmsg::ChassisStatus> m_chassis_status_publisher;
    RosPublisher<rosmsg::BmsStatus> m_bms_status_publisher;
    RosPublisher<rosmsg::CommStatus> m_comm_status_publisher;
    RosPublisher<rosmsg::DriveMotorArray> m_drive_motors_publisher;
    RosPublisher<rosmsg::DriveMotorStatusArray> m_drive_motor_status_publisher;
    RosPublisher<rosmsg::ServoMotorArray> m_servo_motors_publisher;
    RosPublisher<rosmsg::ServoMotorStatusArray> m_servo_motor_status_publisher;
    RosPublisher<rosmsg::Imu> m_imu_publisher;
    RosPublisher<rosmsg::RcStatus> m_rc_status_publisher;
    RosPublisher<rosmsg::CanFrameStatsArray> m_can_frame_stats_publisher;

    // ROS2 订阅器/服务：接收速度控制和一次性控制请求。
    RosSubscription<rosmsg::Twist> m_cmd_vel_subscription;
    RosSubscription<rosmsg::TwistStamped> m_cmd_vel_stamped_subscription;
    RosService<rosmsg::SetMotionMode> m_set_motion_mode_service;
    RosService<rosmsg::SetTowMode> m_set_tow_mode_service;
    RosService<rosmsg::ClearFault> m_clear_fault_service;
    RosService<rosmsg::CalibrateServo> m_calibrate_servo_service;

    // 定时器：命令发送、反馈发布和诊断发布分别按不同频率运行。
    RosTimer m_command_timer;
    RosTimer m_odom_timer;
    RosTimer m_chassis_status_timer;
    RosTimer m_bms_timer;
    RosTimer m_comm_timer;
    RosTimer m_motor_timer;
    RosTimer m_motor_status_timer;
    RosTimer m_imu_timer;
    RosTimer m_rc_timer;
    RosTimer m_frame_stats_timer;

    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

    mutable std::mutex m_command_mutex;
    rosmsg::Twist m_latest_cmd_vel;
    RosTime m_last_cmd_vel_time;
    bool m_has_cmd_vel = false;

    mutable std::mutex m_connection_mutex;
    ConnectionState m_connection_state = ConnectionState::Unknown;
    FailReason m_fail_reason = FailReason::None;

    std::atomic_bool m_stop_requested{false};
    std::thread m_connect_thread;

    std::uint64_t m_last_odom_sequence = 0;
    rosmsg::Odometry m_last_odom;
    std::vector<std::uint16_t> m_tracked_can_ids;
    std::uint64_t m_last_chassis_status_system_sequence = 0;
    std::uint64_t m_last_chassis_status_mode_sequence = 0;
    bool m_chassis_fault_log_active = false;
    std::uint32_t m_last_logged_chassis_fault = 0;
    std::uint32_t m_last_logged_system_status_flags = 0;
    RosTime m_last_chassis_fault_log_time;
    std::uint64_t m_last_bms_basic_sequence = 0;
    std::uint64_t m_last_bms_fault_sequence = 0;
    std::uint64_t m_last_drive_motors_published_sequence = 0;
    std::uint64_t m_last_drive_motor_status_published_sequence = 0;
    std::uint64_t m_last_servo_motors_published_sequence = 0;
    std::uint64_t m_last_servo_motor_status_published_sequence = 0;
    std::uint64_t m_last_imu_sequence = 0;
    std::uint64_t m_last_rc_status_sequence = 0;
};

}  // namespace masc_chassis_ros_driver
