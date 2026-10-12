// 末端位姿发布节点。
//
// 这个节点不做运动学计算，只负责把配置文件中的数字组装成
// geometry_msgs/msg/PoseArray，然后发布给 moveit_pose_executor。
// 输入时每 6 个数字表示一个位姿：
//   x, y, z, roll, pitch, yaw
// 发布时，roll、pitch、yaw 会被转换成 Pose 消息需要的四元数：
//   x, y, z, qx, qy, qz, qw
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>

class PoseTargetPublisher : public rclcpp::Node
{
public:
  // 构造发布节点，读取 YAML 或命令行传入的 ROS 2 参数。
  PoseTargetPublisher()
  : Node("pose_target_publisher")
  {
    // 目标 topic 默认连接到 MoveIt 执行节点的订阅 topic。
    const std::string topic = declare_parameter<std::string>(
      "target_topic", "/moveit_pose_executor/target_poses");
    // 所有 Pose 都使用同一个坐标系，例如 base_link。
    target_.header.frame_id = declare_parameter<std::string>("frame_id", "base_link");

    // poses 是扁平的一维数组。
    // 例如：
    //   [x1,y1,z1,roll1,pitch1,yaw1, x2,y2,z2,roll2,pitch2,yaw2]
    // 就表示两个末端位姿。
    const auto values = declare_parameter<std::vector<double>>(
      "poses", std::vector<double>{0.4, 0.0, 0.4, 0.0, 0.0, 0.0});

    // ROS 内部通常使用弧度，所以默认值是 false。
    // 如果设置为 true，就可以在配置文件里写 90、180 这样的角度值，
    // 程序会先把它们转换成弧度，再计算四元数。
    angles_in_degrees_ = declare_parameter<bool>("angles_in_degrees", false);
    const bool auto_publish = declare_parameter<bool>("auto_publish", true);
    const double auto_publish_delay = declare_parameter<double>("auto_publish_delay_sec", 1.0);
    interactive_mode_ = declare_parameter<bool>("interactive_mode", false);
    execute_after_input_ = declare_parameter<bool>("execute_after_input", true);
    execute_after_yaml_ = declare_parameter<bool>("execute_after_yaml", false);
    // 真机启动时，执行器会先同步运动到初始关节位，执行服务要等这段运动
    // 完成后才会出现。发布器因此不能只等 1 秒；默认最多等待 30 秒。
    service_wait_timeout_sec_ = declare_parameter<double>("service_wait_timeout_sec", 30.0);
    // 服务被发现后再发布目标，并留一点时间让执行器的订阅回调保存 PoseArray。
    // 这是 DDS 消息传递等待时间，不是机械臂运动延迟。
    target_delivery_delay_sec_ = declare_parameter<double>("target_delivery_delay_sec", 0.1);

    // 将一维数字数组转换成 PoseArray 中的多个 Pose。
    parse_poses(values);

    // QoS(10) 表示最多缓存 10 条消息，reliable 表示尽量保证消息送达。
    publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(topic, rclcpp::QoS(10).reliable());

    // 交互模式需要调用 MoveIt 的执行服务，因此创建一个服务客户端。
    // 该服务由 moveit_pose_executor 提供。
    execute_client_ = create_client<std_srvs::srv::Trigger>(
      "/moveit_pose_executor/execute_latest");
    // 提供手动发布服务：
    // ros2 service call /pose_target_publisher/publish std_srvs/srv/Trigger "{}"
    publish_service_ = create_service<std_srvs::srv::Trigger>(
      "~/publish",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        publish_targets();
        response->success = true;
        response->message = "Published " + std::to_string(target_.poses.size()) + " pose(s)";
      });

    // 自动发布不是立即执行，而是延迟一小段时间发布。
    // 这样可以给 MoveIt 执行节点留出启动和创建订阅者的时间。
    // 交互模式下不自动发布 YAML 中的默认目标，避免启动后立即发送一条
    // 用户没有输入的轨迹。普通 YAML 模式仍然保留自动发布功能。
    if (auto_publish && !interactive_mode_) {
      const auto delay = std::chrono::duration<double>(std::max(0.01, auto_publish_delay));
      auto_publish_timer_ = create_wall_timer(
        delay, [this]() {
          if (execute_after_yaml_) {
            execute_latest_target();
          } else {
            publish_targets();
          }
          auto_publish_timer_->cancel();
        });
    }

    RCLCPP_INFO(
      get_logger(), "Loaded %zu pose(s) in frame '%s'; publishing to '%s'",
      target_.poses.size(), target_.header.frame_id.c_str(), topic.c_str());

    if (interactive_mode_) {
      RCLCPP_INFO(
        get_logger(),
        "Interactive mode: enter one or more groups of "
        "'x y z roll pitch yaw' on one line, or enter 'q' to quit");
    }
  }

  // 从 main() 中单独启动一个输入线程，避免 std::getline 阻塞 ROS 2 的 spin。
  void start_interactive_input()
  {
    if (!interactive_mode_) {
      return;
    }

    auto self = std::static_pointer_cast<PoseTargetPublisher>(shared_from_this());
    std::weak_ptr<PoseTargetPublisher> weak_node = self;
    std::thread(
      [weak_node]() {
        if (auto node = weak_node.lock()) {
          node->interactive_loop();
        }
      }).detach();
  }

private:
  geometry_msgs::msg::Pose make_pose(
    double x, double y, double z, double roll, double pitch, double yaw) const
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      !std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw))
    {
      throw std::invalid_argument("Pose contains NaN or infinity");
    }

    if (angles_in_degrees_) {
      constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
      roll *= degrees_to_radians;
      pitch *= degrees_to_radians;
      yaw *= degrees_to_radians;
    }

    // ROS 消息保存的是四元数，不保存 RPY，因此在发布前完成转换。
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    quaternion.normalize();

    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    return pose;
  }

  void parse_poses(const std::vector<double> & values)
  {
    // 输入的一个目标由位置 3 个数和 RPY 欧拉角 3 个数构成，总共 6 个数。
    constexpr std::size_t values_per_pose = 6;
    if (values.empty() || values.size() % values_per_pose != 0) {
      throw std::invalid_argument(
              "Parameter 'poses' must contain 6*N values: x y z roll pitch yaw");
    }

    // 提前分配数组空间，避免后续 push_back 时反复扩容。
    target_.poses.reserve(values.size() / values_per_pose);
    for (std::size_t offset = 0; offset < values.size(); offset += values_per_pose) {
      // 从第 offset 个数字开始读取当前位姿。
      target_.poses.push_back(
        make_pose(
          values[offset], values[offset + 1], values[offset + 2],
          values[offset + 3], values[offset + 4], values[offset + 5]));
    }
  }

  void interactive_loop()
  {
    std::string line;
    while (rclcpp::ok()) {
      std::cout << "pose [x y z roll pitch yaw] > " << std::flush;
      if (!std::getline(std::cin, line)) {
        break;
      }

      if (line == "q" || line == "Q" || line == "quit" || line == "exit") {
        RCLCPP_INFO(get_logger(), "Interactive input stopped");
        break;
      }

      std::istringstream input(line);
      std::vector<double> values;
      double value = 0.0;
      while (input >> value) {
        values.push_back(value);
      }

      // 每个位姿正好需要 6 个数，但一行可以包含多组位姿。例如 12 个数
      // 表示两个连续目标点，18 个数表示三个连续目标点。
      // 如果遇到字母等非数字内容，input 会在到达行尾前解析失败，此时
      // !input.eof() 为 true，也应当拒绝这次输入。
      constexpr std::size_t values_per_pose = 6;
      if (values.empty() || values.size() % values_per_pose != 0 || !input.eof()) {
        RCLCPP_ERROR(
          get_logger(),
          "Expected 6*N numbers: x y z roll pitch yaw "
          "[x y z roll pitch yaw ...]");
        continue;
      }

      try {
        // 先在局部数组中完成全部 RPY 到四元数的转换。只有所有目标都合法时，
        // 才替换 target_，避免后面某一组数据错误却留下不完整的轨迹。
        std::vector<geometry_msgs::msg::Pose> poses;
        poses.reserve(values.size() / values_per_pose);
        for (std::size_t offset = 0; offset < values.size(); offset += values_per_pose) {
          poses.push_back(
            make_pose(
              values[offset], values[offset + 1], values[offset + 2],
              values[offset + 3], values[offset + 4], values[offset + 5]));
        }

        {
          std::lock_guard<std::mutex> lock(target_mutex_);
          target_.poses = std::move(poses);
        }
        if (execute_after_input_) {
          execute_latest_target();
        } else {
          publish_targets();
        }
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Invalid pose: %s", error.what());
      }
    }
  }

  void execute_latest_target()
  {
    // execute_latest 服务只有在执行器完成启动初始化后才创建。先等待服务，
    // 可以保证 YAML 目标不会在机械臂回初始位的过程中被过早发布并丢失。
    const auto wait_timeout = std::chrono::duration<double>(
      std::max(0.0, service_wait_timeout_sec_));
    if (!execute_client_->wait_for_service(wait_timeout)) {
      RCLCPP_ERROR(
        get_logger(),
        "Service /moveit_pose_executor/execute_latest was not available within %.1f seconds",
        service_wait_timeout_sec_);
      return;
    }

    // 服务就绪后再发送最新目标。这样无论初始运动实际用了 2 秒还是 20 秒，
    // 发布时 moveit_pose_executor 都已经完成初始化并保持目标订阅有效。
    publish_targets();

    // publish() 是异步发送。短暂等待可确保订阅回调先把 PoseArray 写入
    // latest_target_，然后 Trigger 服务再读取它。默认 0.1 秒对本机 DDS 足够。
    const auto delivery_delay = std::chrono::duration<double>(
      std::max(0.0, target_delivery_delay_sec_));
    std::this_thread::sleep_for(delivery_delay);

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    execute_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(get_logger(), "Pose execution succeeded: %s", response->message.c_str());
        } else {
          RCLCPP_ERROR(get_logger(), "Pose execution failed: %s", response->message.c_str());
        }
      });
  }

  void publish_targets()
  {
    geometry_msgs::msg::PoseArray message;
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      message = target_;
    }

    // 每次发送前刷新时间戳。
    message.header.stamp = now();

    // 发布整个 PoseArray。
    // 多个 Pose 会作为一条消息同时发送，MoveIt 节点会按照数组顺序执行。
    publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "Published %zu target pose(s)", message.poses.size());
  }

  // 要发布的全部目标位姿。
  geometry_msgs::msg::PoseArray target_;
  bool angles_in_degrees_;
  bool interactive_mode_;
  bool execute_after_input_;
  bool execute_after_yaml_;
  double service_wait_timeout_sec_;
  double target_delivery_delay_sec_;
  std::mutex target_mutex_;

  // ROS 2 发布者、服务端和定时器对象。
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr publish_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr execute_client_;
  rclcpp::TimerBase::SharedPtr auto_publish_timer_;
};

int main(int argc, char ** argv)
{
  // 初始化 ROS 2，然后创建并运行发布节点。
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<PoseTargetPublisher>();
    node->start_interactive_input();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("pose_target_publisher"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
