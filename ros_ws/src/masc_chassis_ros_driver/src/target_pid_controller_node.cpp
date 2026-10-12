#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <masc_chassis_msgs/msg/task_status.hpp>
#include <masc_chassis_msgs/srv/task_control.hpp>

namespace {
double clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

double wrap_angle(double value) {
  while (value > M_PI) value -= 2.0 * M_PI;
  while (value < -M_PI) value += 2.0 * M_PI;
  return value;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q) {
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

class TargetPidController final : public rclcpp::Node {
public:
  TargetPidController() : Node("target_pid_controller") {
    kp_xy_ = declare_parameter("kp_xy", 0.8);
    ki_xy_ = declare_parameter("ki_xy", 0.0);
    kd_xy_ = declare_parameter("kd_xy", 0.05);
    kp_yaw_ = declare_parameter("kp_yaw", 1.2);
    ki_yaw_ = declare_parameter("ki_yaw", 0.0);
    kd_yaw_ = declare_parameter("kd_yaw", 0.05);
    position_deadband_ = declare_parameter("position_deadband", 0.01);
    yaw_deadband_ = declare_parameter("yaw_deadband", 0.02);
    max_speed_ = declare_parameter("max_linear_speed", 0.05);
    max_yaw_speed_ = declare_parameter("max_angular_speed", 0.8);
    integral_limit_ = declare_parameter("integral_limit", 1.0);
    control_period_ = declare_parameter("control_period", 0.02);
    task_names_ = declare_parameter<std::vector<std::string>>(
      "task_names", {"forward_10cm", "backward_10cm"});
    task_delta_x_m_ = declare_parameter<std::vector<double>>(
      "task_delta_x_m", {0.10, -0.10});
    task_delta_y_m_ = declare_parameter<std::vector<double>>(
      "task_delta_y_m", {0.0, 0.0});
    task_delta_yaw_rad_ = declare_parameter<std::vector<double>>(
      "task_delta_yaw_rad", {0.0, 0.0});
    task_timeout_sec_ = declare_parameter("task_timeout_sec", 10.0);
    odom_timeout_sec_ = declare_parameter("odom_timeout_sec", 0.5);
    status_period_sec_ = declare_parameter("status_period_sec", 0.5);
    status_topic_ = declare_parameter("status_topic", std::string("/chassis/task_status"));
    target_reached_topic_ =
      declare_parameter("target_reached_topic", std::string("/target_reached"));

    if (control_period_ <= 0.0 || max_speed_ <= 0.0 ||
        task_timeout_sec_ <= 0.0 || odom_timeout_sec_ <= 0.0 ||
        status_period_sec_ <= 0.0 || task_names_.empty() ||
        task_names_.size() != task_delta_x_m_.size() ||
        task_names_.size() != task_delta_y_m_.size() ||
        task_names_.size() != task_delta_yaw_rad_.size()) {
      throw std::invalid_argument("invalid target PID or test task parameter");
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "wheel_odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        odom_ = *msg;
        have_odom_ = true;
        last_odom_time_ = now();
      });
    target_sub_ = create_subscription<geometry_msgs::msg::Pose2D>(
      "target_pose", 10,
      [this](geometry_msgs::msg::Pose2D::ConstSharedPtr msg) {
        if (test_task_active_) {
          RCLCPP_WARN(get_logger(), "ignored target_pose while test task is active");
          return;
        }
        target_ = *msg;
        have_target_ = true;
        resetPid();
      });
    task_service_ = create_service<masc_chassis_msgs::srv::TaskControl>(
      "/chassis/task_control",
      [this](const std::shared_ptr<masc_chassis_msgs::srv::TaskControl::Request> request,
             std::shared_ptr<masc_chassis_msgs::srv::TaskControl::Response> response) {
        using Request = masc_chassis_msgs::srv::TaskControl::Request;
        if (request->command == Request::STOP) {
          stopMotion("stopped_by_task_controller");
          response->success = true;
          response->message = "stopped";
        } else if (request->command == Request::EXECUTE) {
          response->message = startTask(request->task, request->execution_id);
          response->success = response->message.empty();
        } else {
          response->success = false;
          response->message = "unsupported command";
        }
      });
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    reached_pub_ =
      create_publisher<std_msgs::msg::Bool>(target_reached_topic_, 10);
    status_pub_ = create_publisher<masc_chassis_msgs::msg::TaskStatus>(status_topic_, 10);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(control_period_)),
      [this] { update(); });

    RCLCPP_INFO(
      get_logger(),
      "target PID ready; loaded %zu task plans, max speed %.3f m/s",
      task_names_.size(), max_speed_);
  }

private:
  std::string startTask(const std::string & plan_id, const std::string & execution_id) {
    if (execution_id.empty()) return "execution_id is required";

    const auto plan = std::find(task_names_.begin(), task_names_.end(), plan_id);
    if (plan == task_names_.end()) {
      return "unsupported task";
    }
    const auto current_time = now();
    if (!have_odom_ ||
        (current_time - last_odom_time_).seconds() > odom_timeout_sec_) {
      return "odometry_unavailable";
    }
    if (have_target_ || test_task_active_) {
      return "controller_busy";
    }

    const double current_x = odom_.pose.pose.position.x;
    const double current_y = odom_.pose.pose.position.y;
    const double current_yaw = yaw_from_quaternion(odom_.pose.pose.orientation);
    start_x_ = current_x;
    start_y_ = current_y;
    active_task_id_ = plan_id;
    active_execution_id_ = execution_id;
    const auto plan_index = static_cast<std::size_t>(
      std::distance(task_names_.begin(), plan));
    const double dx = task_delta_x_m_[plan_index];
    const double dy = task_delta_y_m_[plan_index];
    const double dyaw = task_delta_yaw_rad_[plan_index];
    const double translation = std::hypot(dx, dy);
    active_extent_ = translation > 1e-9 ? translation : std::abs(dyaw);
    active_is_rotation_ = translation <= 1e-9;
    start_yaw_ = current_yaw;
    target_.x = current_x + std::cos(current_yaw) * dx - std::sin(current_yaw) * dy;
    target_.y = current_y + std::sin(current_yaw) * dx + std::cos(current_yaw) * dy;
    target_.theta = wrap_angle(current_yaw + dyaw);
    task_start_time_ = current_time;
    last_status_time_ = current_time;
    test_task_active_ = true;
    have_target_ = true;
    resetPid();
    publishTaskRunning(0.0, "executing_task");
    RCLCPP_INFO(
      get_logger(), "test started from (%.3f, %.3f), target=(%.3f, %.3f)",
      current_x, current_y, target_.x, target_.y);
    return "";
  }

  void resetPid() {
    integral_x_ = 0.0;
    integral_y_ = 0.0;
    integral_yaw_ = 0.0;
    previous_x_ = 0.0;
    previous_y_ = 0.0;
    previous_yaw_ = 0.0;
    have_previous_error_ = false;
  }

  void update() {
    geometry_msgs::msg::Twist cmd;
    if (!have_target_) {
      cmd_pub_->publish(cmd);
      return;
    }

    const auto current_time = now();
    if (!have_odom_ ||
        (current_time - last_odom_time_).seconds() > odom_timeout_sec_) {
      if (test_task_active_) publishTaskError(active_task_id_, "odometry_timeout");
      clearTarget();
      cmd_pub_->publish(cmd);
      RCLCPP_ERROR(get_logger(), "target control stopped: odometry timeout");
      return;
    }
    if (test_task_active_ &&
        (current_time - task_start_time_).seconds() > task_timeout_sec_) {
      publishTaskError(active_task_id_, "motion_timeout");
      clearTarget();
      cmd_pub_->publish(cmd);
      RCLCPP_ERROR(get_logger(), "test stopped: motion timeout");
      return;
    }

    const double x = odom_.pose.pose.position.x;
    const double y = odom_.pose.pose.position.y;
    const double yaw = yaw_from_quaternion(odom_.pose.pose.orientation);
    const double world_error_x = target_.x - x;
    const double world_error_y = target_.y - y;
    const double distance = std::hypot(world_error_x, world_error_y);
    const double yaw_error = wrap_angle(target_.theta - yaw);

    if (distance <= position_deadband_ && std::fabs(yaw_error) <= yaw_deadband_) {
      const bool completed_test = test_task_active_;
      clearTarget();
      cmd_pub_->publish(cmd);
      if (completed_test) {
        std_msgs::msg::Bool reached;
        reached.data = true;
        reached_pub_->publish(reached);
        publishTaskStatus(2, 1.0, "target_reached");
        RCLCPP_INFO(get_logger(), "task completed: %s", active_task_id_.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "target pose reached");
      }
      return;
    }

    // 将 odom 世界坐标误差旋转到底盘自身坐标系，再生成 Twist。
    const double body_error_x =
      std::cos(yaw) * world_error_x + std::sin(yaw) * world_error_y;
    const double body_error_y =
      -std::sin(yaw) * world_error_x + std::cos(yaw) * world_error_y;
    const double dt = control_period_;
    integral_x_ = clamp(integral_x_ + body_error_x * dt, -integral_limit_, integral_limit_);
    integral_y_ = clamp(integral_y_ + body_error_y * dt, -integral_limit_, integral_limit_);
    integral_yaw_ = clamp(integral_yaw_ + yaw_error * dt, -integral_limit_, integral_limit_);

    double derivative_x = 0.0;
    double derivative_y = 0.0;
    double derivative_yaw = 0.0;
    if (have_previous_error_) {
      derivative_x = (body_error_x - previous_x_) / dt;
      derivative_y = (body_error_y - previous_y_) / dt;
      derivative_yaw = (yaw_error - previous_yaw_) / dt;
    }
    previous_x_ = body_error_x;
    previous_y_ = body_error_y;
    previous_yaw_ = yaw_error;
    have_previous_error_ = true;

    cmd.linear.x = clamp(
      kp_xy_ * body_error_x + ki_xy_ * integral_x_ + kd_xy_ * derivative_x,
      -max_speed_, max_speed_);
    cmd.linear.y = clamp(
      kp_xy_ * body_error_y + ki_xy_ * integral_y_ + kd_xy_ * derivative_y,
      -max_speed_, max_speed_);
    cmd.angular.z = clamp(
      kp_yaw_ * yaw_error + ki_yaw_ * integral_yaw_ + kd_yaw_ * derivative_yaw,
      -max_yaw_speed_, max_yaw_speed_);
    cmd_pub_->publish(cmd);

    if (test_task_active_ &&
        (current_time - last_status_time_).seconds() >= status_period_sec_) {
      const double travelled = std::hypot(x - start_x_, y - start_y_);
      const double rotated = std::abs(wrap_angle(yaw - start_yaw_));
      const double progress = clamp(
        (active_is_rotation_ ? rotated : travelled) / active_extent_, 0.0, 1.0);
      publishTaskRunning(progress, "executing_task");
      last_status_time_ = current_time;
    }
  }

  void stopMotion(const std::string & reason) {
    const bool was_active = have_target_ || test_task_active_;
    if (test_task_active_) publishTaskStatus(3, 0.0, reason);
    clearTarget();
    publishZero();
    if (was_active) {
      RCLCPP_WARN(get_logger(), "motion stopped: %s", reason.c_str());
    }
  }

  void clearTarget() {
    have_target_ = false;
    test_task_active_ = false;
    resetPid();
  }

  void publishZero() {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  void publishTaskStatus(uint8_t state, double progress, const std::string & message) {
    masc_chassis_msgs::msg::TaskStatus status;
    status.execution_id = active_execution_id_;
    status.task = active_task_id_;
    status.state = state;
    status.progress = progress;
    status.message = message;
    status_pub_->publish(status);
  }

  void publishTaskRunning(double progress, const std::string & message) {
    publishTaskStatus(1, progress, message);
  }

  void publishTaskError(const std::string &, const std::string & message) {
    publishTaskStatus(3, 0.0, message);
  }

  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::Pose2D target_;
  bool have_odom_{false};
  bool have_target_{false};
  bool test_task_active_{false};
  bool have_previous_error_{false};
  double kp_xy_;
  double ki_xy_;
  double kd_xy_;
  double kp_yaw_;
  double ki_yaw_;
  double kd_yaw_;
  double position_deadband_;
  double yaw_deadband_;
  double max_speed_;
  double max_yaw_speed_;
  double integral_limit_;
  double control_period_;
  double active_extent_{0.10};
  bool active_is_rotation_{false};
  double task_timeout_sec_;
  double odom_timeout_sec_;
  double status_period_sec_;
  double start_x_{0.0};
  double start_y_{0.0};
  double start_yaw_{0.0};
  double integral_x_{0.0};
  double integral_y_{0.0};
  double integral_yaw_{0.0};
  double previous_x_{0.0};
  double previous_y_{0.0};
  double previous_yaw_{0.0};
  std::string status_topic_;
  std::string target_reached_topic_;
  std::string active_task_id_;
  std::string active_execution_id_;
  std::vector<std::string> task_names_;
  std::vector<double> task_delta_x_m_;
  std::vector<double> task_delta_y_m_;
  std::vector<double> task_delta_yaw_rad_;
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time task_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr target_sub_;
  rclcpp::Service<masc_chassis_msgs::srv::TaskControl>::SharedPtr task_service_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;
  rclcpp::Publisher<masc_chassis_msgs::msg::TaskStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetPidController>());
  rclcpp::shutdown();
  return 0;
}
