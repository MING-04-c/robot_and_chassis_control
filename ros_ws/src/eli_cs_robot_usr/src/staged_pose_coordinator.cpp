// 分段位姿控制协调器。
//
// 控制流程：
//   1. 接收最终目标位姿；
//   2. 沿指定方向生成一个距离目标更远的接近点；
//   3. 把接近点交给 moveit_pose_executor 做普通规划；
//   4. 规划执行成功后，把原始最终点交给 admittance_pose_controller；
//   5. 最后一小段由导纳控制完成。
//
// 这个节点不做碰撞检测，也不修改 MoveIt 或底层控制器。

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class StagedPoseCoordinator : public rclcpp::Node
{
public:
  StagedPoseCoordinator()
  : Node("staged_pose_coordinator")
  , tf_buffer_(get_clock())
  , tf_listener_(tf_buffer_)
  {
    approach_axis_ = declare_parameter<std::string>("approach_axis", "z");
    approach_offset_ = declare_parameter<double>("approach_offset", 0.05);
    transition_delay_ = declare_parameter<double>("transition_delay", 0.2);
    handoff_hold_delay_ = declare_parameter<double>("handoff_hold_delay", 0.15);
    skip_target_position_tolerance_ = declare_parameter<double>(
      "skip_target_position_tolerance", 0.01);
    skip_target_orientation_tolerance_ = declare_parameter<double>(
      "skip_target_orientation_tolerance", 0.05);
    switch_controllers_ = declare_parameter<bool>("switch_controllers", true);
    moveit_controller_ = declare_parameter<std::string>(
      "moveit_controller", "joint_trajectory_controller");
    compliance_controller_ = declare_parameter<std::string>(
      "compliance_controller", "forward_position_controller");
    admittance_node_name_ = declare_parameter<std::string>(
      "admittance_node_name", "admittance_controller");
    moveit_target_topic_ = declare_parameter<std::string>(
      "moveit_target_topic", "/moveit_pose_executor/target_poses");
    moveit_execute_service_ = declare_parameter<std::string>(
      "moveit_execute_service", "/moveit_pose_executor/execute_latest");

    if (approach_axis_ != "x" && approach_axis_ != "y" && approach_axis_ != "z") {
      throw std::runtime_error("approach_axis must be x, y, or z");
    }
    if (approach_offset_ <= 0.0 || transition_delay_ < 0.0) {
      throw std::runtime_error("approach_offset must be positive and transition_delay non-negative");
    }
    if (handoff_hold_delay_ < 0.0) {
      throw std::runtime_error("handoff_hold_delay must be non-negative");
    }
    if (skip_target_position_tolerance_ <= 0.0 ||
      skip_target_orientation_tolerance_ <= 0.0)
    {
      throw std::runtime_error("target skip tolerances must be positive");
    }

    approach_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
      moveit_target_topic_, rclcpp::QoS(10).reliable());
    final_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/admittance_controller/target_poses", rclcpp::QoS(10).reliable());

    target_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "~/target_poses", rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::PoseArray::ConstSharedPtr message) {
        receive_target(*message);
      });

    // 仿真中最终一段会把 position command interface 从轨迹控制器交给
    // forward_position_controller。缓存真实关节状态用于交接时的“保持命令”，
    // 避免新控制器的参考值短暂回到默认值而使机械臂受重力下坠。
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr message) {
        cache_joint_state(*message);
      });
    // 真机使用 JointTrajectory，不能在同一话题上创建 Float64MultiArray 发布器。
    // 因此这个仿真交接发布器只在确实需要切换到 position controller 时创建。
    if (switch_controllers_) {
      hold_position_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/" + compliance_controller_ + "/commands", rclcpp::QoS(10).reliable());
    }

    execute_client_ = create_client<std_srvs::srv::Trigger>(moveit_execute_service_);
    admittance_enable_client_ = create_client<std_srvs::srv::SetBool>(
      "/" + admittance_node_name_ + "/enable");
    switch_controller_client_ =
      create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

    RCLCPP_INFO(
      get_logger(),
      "Staged control ready: approach axis=%s, offset=%.3f m, final controller=admittance",
      approach_axis_.c_str(), approach_offset_);
  }

private:
  void receive_target(const geometry_msgs::msg::PoseArray & message)
  {
    if (message.poses.empty()) {
      RCLCPP_ERROR(get_logger(), "Received an empty target PoseArray");
      return;
    }
    if (message.poses.size() != 1) {
      RCLCPP_ERROR(
        get_logger(),
        "This staged test currently accepts exactly one final pose; received %zu",
        message.poses.size());
      return;
    }

    // 目标已经在当前 TCP 附近时，不再生成“目标外侧 5 cm”的接近点。
    // 否则重复输入同一个目标会让机械臂先离开当前位置，再回到原位置。
    if (target_is_already_reached(message.poses.front())) {
      RCLCPP_INFO(
        get_logger(),
        "Target is already reached; ignoring duplicate target instead of replanning");
      return;
    }

    if (executing_) {
      RCLCPP_WARN(get_logger(), "A staged motion is already running; target ignored");
      return;
    }

    start_target(message);
  }

  void start_target(const geometry_msgs::msg::PoseArray & message)
  {
    executing_ = true;
    final_target_ = message;
    approach_target_ = message;
    offset_pose(approach_target_.poses.front());

    // 新目标到来时，先停止上一目标的导纳输出，避免 MoveIt 规划期间
    // admittance_pose_controller 继续发布旧目标的关节命令。
    disable_admittance_then_plan();
  }

  bool target_is_already_reached(const geometry_msgs::msg::Pose & target) const
  {
    geometry_msgs::msg::TransformStamped current;
    try {
      current = tf_buffer_.lookupTransform(
        "base_link", "tool0", tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      // TF 尚未准备好时不能判定重复目标，继续走正常规划流程。
      return false;
    }

    const double dx = target.position.x - current.transform.translation.x;
    const double dy = target.position.y - current.transform.translation.y;
    const double dz = target.position.z - current.transform.translation.z;
    const double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (position_error > skip_target_position_tolerance_) {
      return false;
    }

    tf2::Quaternion current_q(
      current.transform.rotation.x, current.transform.rotation.y,
      current.transform.rotation.z, current.transform.rotation.w);
    tf2::Quaternion target_q(
      target.orientation.x, target.orientation.y,
      target.orientation.z, target.orientation.w);
    current_q.normalize();
    target_q.normalize();
    const double absolute_dot = std::clamp(
      std::abs(current_q.dot(target_q)), 0.0, 1.0);
    const double orientation_error = 2.0 * std::acos(absolute_dot);
    return orientation_error <= skip_target_orientation_tolerance_;
  }

  void disable_admittance_then_plan()
  {
    if (!admittance_enable_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Service /admittance_controller/enable is unavailable");
      executing_ = false;
      return;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    admittance_enable_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        const auto response = future.get();
        if (!response->success) {
          RCLCPP_ERROR(get_logger(), "Could not disable admittance: %s", response->message.c_str());
          executing_ = false;
          return;
        }

        auto publish_approach = [this]() {
          approach_publisher_->publish(approach_target_);
          RCLCPP_INFO(get_logger(), "Approach pose published to MoveIt");
          transition_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(transition_delay_)),
            [this]() { request_moveit_execution(); });
        };

        if (switch_controllers_) {
          // 仿真：MoveIt 用轨迹控制器，导纳用直接位置控制器。
          switch_controllers(
            moveit_controller_, compliance_controller_, std::move(publish_approach));
        } else {
          // 真机：MoveIt 和导纳都通过 scaled_joint_trajectory_controller，
          // 不调用控制器切换服务，避免打断真实机器人控制器。
          publish_approach();
        }
      });
  }

  void switch_controllers(
    const std::string & activate,
    const std::string & deactivate,
    std::function<void()> on_success)
  {
    if (!switch_controller_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Service /controller_manager/switch_controller is unavailable");
      executing_ = false;
      return;
    }

    auto request =
      std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = {activate};
    request->deactivate_controllers = {deactivate};
    // BEST_EFFORT 允许“要激活的控制器已经 active”这种无害情况，服务返回后
    // 仍检查 ok，真正的资源冲突或控制器缺失不会被当作成功继续执行。
    request->strictness = request->BEST_EFFORT;
    request->activate_asap = true;
    request->timeout.sec = 5;

    switch_controller_client_->async_send_request(
      request,
      [this, activate, deactivate, on_success = std::move(on_success)](
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
        const auto response = future.get();
        if (!response->ok) {
          RCLCPP_ERROR(
            get_logger(), "Controller switch failed: deactivate '%s', activate '%s'",
            deactivate.c_str(), activate.c_str());
          executing_ = false;
          return;
        }
        RCLCPP_INFO(
          get_logger(), "Controller switched: '%s' -> '%s'",
          deactivate.c_str(), activate.c_str());
        on_success();
      });
  }

  void cache_joint_state(const sensor_msgs::msg::JointState & message)
  {
    if (message.name.size() != message.position.size()) {
      return;
    }
    std::array<double, 6> positions{};
    for (std::size_t index = 0; index < joint_names_.size(); ++index) {
      const auto iterator = std::find(
        message.name.begin(), message.name.end(), joint_names_[index]);
      if (iterator == message.name.end()) {
        return;
      }
      positions[index] = message.position[static_cast<std::size_t>(
        std::distance(message.name.begin(), iterator))];
    }
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    latest_joint_positions_ = positions;
    have_joint_state_ = true;
  }

  bool publish_hold_position()
  {
    if (!hold_position_publisher_) {
      RCLCPP_ERROR(get_logger(), "Position hold publisher is unavailable");
      return false;
    }
    std::array<double, 6> positions{};
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      if (!have_joint_state_) {
        RCLCPP_ERROR(get_logger(), "Cannot hand off controllers: no complete /joint_states received");
        return false;
      }
      positions = latest_joint_positions_;
    }

    std_msgs::msg::Float64MultiArray command;
    command.data.assign(positions.begin(), positions.end());
    hold_position_publisher_->publish(command);
    return true;
  }

  void offset_pose(geometry_msgs::msg::Pose & pose) const
  {
    // 接近点位于最终目标的“外侧”。例如 z 轴时，最终点 z=0.40 m，
    // approach_offset=0.05 m，则先规划到 z=0.45 m，再向 z=0.40 m 运动。
    if (approach_axis_ == "x") {
      pose.position.x += approach_offset_;
    } else if (approach_axis_ == "y") {
      pose.position.y += approach_offset_;
    } else {
      pose.position.z += approach_offset_;
    }
  }

  void request_moveit_execution()
  {
    if (transition_timer_) {
      transition_timer_->cancel();
    }
    if (!execute_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for /moveit_pose_executor/execute_latest");
      transition_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() { request_moveit_execution(); });
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    execute_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        if (!response->success) {
          RCLCPP_ERROR(get_logger(), "Approach motion failed: %s", response->message.c_str());
          executing_ = false;
          return;
        }

        if (switch_controllers_) {
          // 先在 inactive 状态向 forward_position_controller 写入当前关节角。
          // 该控制器激活的第一个控制周期就会保持当前位置，而不是使用默认值。
          if (!publish_hold_position()) {
            executing_ = false;
            return;
          }
          switch_controllers(
            compliance_controller_, moveit_controller_,
            [this]() { handoff_hold_then_enable_admittance(); });
        } else {
          // 真机不切换控制器，直接开启导纳节点；导纳节点会继续向
          // scaled_joint_trajectory_controller 发布带时间的轨迹点。
          enable_admittance_then_publish_final();
        }
      });
  }

  void handoff_hold_then_enable_admittance()
  {
    // 控制器切换成功后再发一次保持命令，覆盖订阅器在 inactive 阶段可能丢失
    // 的消息。等待一个很短的稳定时间后才下发最终末端点。
    if (!publish_hold_position()) {
      executing_ = false;
      return;
    }
    handoff_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(handoff_hold_delay_)),
      [this]() {
        if (handoff_timer_) {
          handoff_timer_->cancel();
        }
        enable_admittance_then_publish_final();
      });
  }

  void enable_admittance_then_publish_final()
  {
    if (!admittance_enable_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Service /admittance_controller/enable is unavailable after MoveIt motion");
      executing_ = false;
      return;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    admittance_enable_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        const auto response = future.get();
        if (!response->success) {
          RCLCPP_ERROR(get_logger(), "Could not enable admittance: %s", response->message.c_str());
          executing_ = false;
          return;
        }

        final_publisher_->publish(final_target_);
        RCLCPP_INFO(get_logger(), "Admittance enabled; final pose handed to admittance controller");
        executing_ = false;
      });
  }

  std::string approach_axis_;
  double approach_offset_;
  double transition_delay_;
  double handoff_hold_delay_;
  double skip_target_position_tolerance_;
  double skip_target_orientation_tolerance_;
  bool switch_controllers_{true};
  std::string moveit_controller_;
  std::string compliance_controller_;
  std::string admittance_node_name_;
  std::string moveit_target_topic_;
  std::string moveit_execute_service_;
  bool executing_{false};
  geometry_msgs::msg::PoseArray approach_target_;
  geometry_msgs::msg::PoseArray final_target_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr approach_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr final_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr target_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr hold_position_publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr execute_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr admittance_enable_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_client_;
  rclcpp::TimerBase::SharedPtr transition_timer_;
  rclcpp::TimerBase::SharedPtr handoff_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  const std::array<std::string, 6> joint_names_{
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  std::mutex joint_state_mutex_;
  std::array<double, 6> latest_joint_positions_{};
  bool have_joint_state_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<StagedPoseCoordinator>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("staged_pose_coordinator"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
