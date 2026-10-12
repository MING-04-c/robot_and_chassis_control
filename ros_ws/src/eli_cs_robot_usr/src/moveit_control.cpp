// MoveIt 末端位姿执行节点。
//
// 本文件完成的事情可以概括为：
//   1. 订阅一个或多个末端位姿；
//   2. 让 MoveIt 根据末端位姿求解关节角度并生成轨迹；
//   3. 把生成的轨迹交给 MoveIt 的控制器管理器执行。
//
// 本节点不直接操作 eli_cs_controllers，而是调用 MoveGroupInterface。
// MoveIt 会根据 controllers.yaml 中的配置，把轨迹发送到
// scaled_joint_trajectory_controller 的 FollowJointTrajectory action。
#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class MoveItPoseExecutor : public rclcpp::Node
{
public:
  // 构造 ROS 2 节点，并读取规划相关参数。
  //
  // Node 的名字最终会显示为：/moveit_pose_executor。
  // automatically_declare_parameters_from_overrides(true) 允许 launch 文件
  // 中传入的参数自动声明，这样可以在 launch 文件里覆盖下面的默认值。
  MoveItPoseExecutor()
  : Node(
      "moveit_pose_executor",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // cs_manipulator 来自 SRDF 中的 group name。
    // tool0 是机械臂末端执行器的 link。
    // base_link 是默认的位姿参考坐标系。
    planning_group_ = parameter_or<std::string>("planning_group", "cs_manipulator");
    end_effector_link_ = parameter_or<std::string>("end_effector_link", "tool0");
    reference_frame_ = parameter_or<std::string>("reference_frame", "base_link");
    // 速度和加速度缩放范围通常是 0.0 到 1.0。
    // 0.2 表示使用 MoveIt 允许最大速度/加速度的 20%。
    velocity_scaling_ = parameter_or<double>("velocity_scaling", 0.2);
    acceleration_scaling_ = parameter_or<double>("acceleration_scaling", 0.2);

    // 普通规划最多允许搜索 5 秒。
    planning_time_ = parameter_or<double>("planning_time", 5.0);

    // MoveIt 等待完整关节状态的最长时间。真机启动时 DDS 发现和
    // /joint_states 订阅建立可能需要一小段时间，因此不使用过短的等待时间。
    current_state_wait_sec_ = parameter_or<double>("current_state_wait_sec", 5.0);

    // 多个位姿时使用笛卡尔路径：
    // eef_step 是末端插值步长，单位是米；
    // jump_threshold 用于限制关节角突然跳变，0.0 表示关闭这个检查；
    // min_cartesian_fraction 是允许执行的最小路径完成比例。
    eef_step_ = parameter_or<double>("cartesian_eef_step", 0.01);
    jump_threshold_ = parameter_or<double>("cartesian_jump_threshold", 0.0);
    min_cartesian_fraction_ = parameter_or<double>("cartesian_min_fraction", 0.95);
    use_cartesian_path_ = parameter_or<bool>("use_cartesian_path", true);

    // 在执行末端目标之前，是否先让机械臂运动到指定的关节初始位姿。
    // 这个初始位姿使用弧度，六个数按照 MoveIt 规划组中的关节顺序排列。
    initialize_before_execution_ = parameter_or<bool>("initialize_before_execution", true);
    initialize_on_startup_ = parameter_or<bool>("initialize_on_startup", true);
    initial_joint_tolerance_ = parameter_or<double>("initial_joint_tolerance", 0.01);
    initial_joint_positions_ = parameter_or<std::vector<double>>(
      "initial_joint_positions", {0.0, -1.57, 0.0, -1.57, 1.57, 0.0});
  }

  // 初始化 MoveGroupInterface、订阅者和服务。
  //
  // 这个函数没有放到构造函数里，是因为 shared_from_this() 只有在对象已经
  // 被 std::make_shared 创建以后才可以安全使用。
  void initialize()
  {
    // MoveGroupInterface 是 MoveIt 提供的 C++ 接口。
    // 它负责和已经启动的 move_group 节点通信，完成规划、求解和执行请求。
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), planning_group_);

    // 告诉 MoveIt 使用哪个末端 link、哪个参考坐标系以及规划速度。
    move_group_->setEndEffectorLink(end_effector_link_);
    move_group_->setPoseReferenceFrame(reference_frame_);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_->setPlanningTime(planning_time_);

    // 提前创建并启动 CurrentStateMonitor，使它在用户输入目标之前就开始接收
    // /joint_states。若等到笛卡尔轨迹已经计算完成才第一次调用
    // getCurrentState()，第一次读取可能因 DDS 订阅尚未准备好而超时。
    if (!move_group_->startStateMonitor(current_state_wait_sec_)) {
      RCLCPP_WARN(
        get_logger(),
        "MoveIt did not receive a complete current state within %.1f seconds; "
        "Cartesian execution will retry when a target is received",
        current_state_wait_sec_);
    }

    // 订阅末端位姿数组。
    // "~" 表示使用当前节点名称作为前缀，因此最终 topic 是：
    // /moveit_pose_executor/target_poses
    //
    // PoseArray 中：
    //   header.frame_id：所有位姿所在的坐标系；
    //   poses：一个或多个 geometry_msgs/msg/Pose。
    target_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "~/target_poses", rclcpp::QoS(1).reliable(),
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr message) {
        // 订阅回调和服务回调可能由不同线程执行。
        // 加锁后再写 latest_target_，避免两个线程同时读写同一份数据。
        std::lock_guard<std::mutex> lock(target_mutex_);
        latest_target_ = *message;
        RCLCPP_INFO(
          get_logger(), "Received %zu target pose(s) in frame '%s'",
          latest_target_.poses.size(), latest_target_.header.frame_id.c_str());
      });

    // 必须先创建目标订阅，再同步执行启动初始轨迹。真机回初始位可能需要数秒，
    // 如果订阅在这段运动结束后才创建，期间发布的 PoseArray 不会被 ROS 2 保存，
    // 随后的 execute_latest 服务就只能得到 "No target poses received"。
    //
    // 此处只提前创建订阅，不提前开放执行服务：目标可以先进入 DDS 接收队列，
    // 但新的轨迹必须等初始轨迹结束后才能执行，避免两条轨迹同时控制机械臂。
    if (initialize_before_execution_ && initialize_on_startup_) {
      auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
      RCLCPP_INFO(get_logger(), "Startup initialization is enabled");
      if (!move_to_initial_joint_pose(response)) {
        RCLCPP_ERROR(
          get_logger(), "Startup initialization failed: %s", response->message.c_str());
        RCLCPP_ERROR(
          get_logger(), "The executor will retry the initial pose before the first target");
      }
    }

    // 创建执行服务。
    // 调用这个服务时，节点会取出最近收到的 PoseArray 并开始规划执行。
    // 最终服务名是：/moveit_pose_executor/execute_latest
    execute_service_ = create_service<std_srvs::srv::Trigger>(
      "~/execute_latest",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        execute_latest(response);
      });

    RCLCPP_INFO(
      get_logger(),
      "Ready: group='%s', end_effector='%s', default_frame='%s'",
      planning_group_.c_str(), end_effector_link_.c_str(), reference_frame_.c_str());

    if (initialize_before_execution_ && !initial_pose_executed_) {
      RCLCPP_INFO(
        get_logger(),
        "The first target will start from the configured %zu-joint initial pose",
        initial_joint_positions_.size());
    }
  }

private:
  template<typename T>
  T parameter_or(const std::string & name, const T & default_value)
  {
    // ROS 2 参数需要先声明才能读取。
    // 如果 launch 没有传入该参数，就使用这里的默认值。
    if (!has_parameter(name)) {
      declare_parameter<T>(name, default_value);
    }
    return get_parameter(name).get_value<T>();
  }

  static bool valid_pose(const geometry_msgs::msg::Pose & pose)
  {
    // position 必须是有限数字，四元数也必须是有限数字。
    // 四元数不能是全 0，因为全 0 不是合法的旋转表示。
    const auto & p = pose.position;
    const auto & q = pose.orientation;
    const double norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
           std::isfinite(q.w) && norm_squared > 1e-12;
  }

  bool move_to_initial_joint_pose(std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    // 初始位姿在一次节点生命周期内只执行一次。也就是说，每次重新 launch
    // 都会先回初始位，但同一次 launch 中后续收到的新目标不会反复回初始位。
    if (!initialize_before_execution_ || initial_pose_executed_) {
      return true;
    }

    // getJointNames() 返回 planning_group_ 中参与规划的活动关节。
    // 初始角数量必须和它完全一致，避免把某个角度错误地发给另一个关节。
    const auto joint_names = move_group_->getJointNames();
    if (initial_joint_positions_.size() != joint_names.size()) {
      response->success = false;
      response->message =
        "Parameter initial_joint_positions contains " +
        std::to_string(initial_joint_positions_.size()) + " values, but planning group '" +
        planning_group_ + "' has " + std::to_string(joint_names.size()) + " joints";
      return false;
    }

    for (const double position : initial_joint_positions_) {
      if (!std::isfinite(position)) {
        response->success = false;
        response->message = "Parameter initial_joint_positions contains NaN or infinity";
        return false;
      }
    }

    // 如果机械臂已经位于初始关节位，就不要向控制器发送一条几乎没有位移的
    // 退化轨迹。部分真机控制器会直接拒绝这种轨迹。getCurrentJointValues()
    // 返回的顺序与 getJointNames() 相同，因此可以逐轴比较。
    const auto current_joint_positions = move_group_->getCurrentJointValues();
    if (current_joint_positions.size() == initial_joint_positions_.size()) {
      bool already_at_initial_pose = true;
      double maximum_error = 0.0;
      for (std::size_t index = 0; index < initial_joint_positions_.size(); ++index) {
        const double error = std::abs(
          current_joint_positions[index] - initial_joint_positions_[index]);
        maximum_error = std::max(maximum_error, error);
        if (error > initial_joint_tolerance_) {
          already_at_initial_pose = false;
        }
      }

      if (already_at_initial_pose) {
        initial_pose_executed_ = true;
        response->success = true;
        response->message = "Robot is already at the configured initial joint pose";
        RCLCPP_INFO(
          get_logger(),
          "Robot is already at the initial joint pose (maximum error %.6f rad)",
          maximum_error);
        return true;
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Could not compare the current and initial joint poses; planning will continue");
    }

    RCLCPP_INFO(get_logger(), "Planning motion to the configured initial joint pose");
    for (std::size_t index = 0; index < joint_names.size(); ++index) {
      RCLCPP_INFO(
        get_logger(), "  %s = %.6f rad",
        joint_names[index].c_str(), initial_joint_positions_[index]);
    }

    // 先把真实机械臂当前状态作为规划起点，再设置六个关节角目标。
    // 这里执行的是关节空间规划，不需要把初始关节角转换为末端 Pose。
    move_group_->setStartStateToCurrentState();
    if (!move_group_->setJointValueTarget(initial_joint_positions_)) {
      response->success = false;
      response->message = "Initial joint pose is outside the robot joint limits";
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan initial_plan;
    const auto planning_result = move_group_->plan(initial_plan);
    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
      response->success = false;
      response->message = "MoveIt failed to plan to the initial joint pose";
      return false;
    }

    const auto execution_result = move_group_->execute(initial_plan);
    if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
      response->success = false;
      response->message = "Controller failed to execute the initial joint trajectory";
      return false;
    }

    initial_pose_executed_ = true;
    RCLCPP_INFO(get_logger(), "Initial joint pose reached; executing end-effector targets");
    return true;
  }

  void execute_latest(std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    // exchange(true) 会先返回旧值，再把 executing_ 设置为 true。
    // 如果旧值已经是 true，说明前一条轨迹仍在执行，拒绝重复执行。
    if (executing_.exchange(true)) {
      response->success = false;
      response->message = "An execution is already in progress";
      return;
    }

    struct ExecutionGuard
    {
      std::atomic_bool & flag;
      // 无论后面规划成功、失败还是提前 return，都自动清除执行标志。
      ~ExecutionGuard() {flag.store(false);}
    } guard{executing_};

    // 从共享变量复制一份目标数据。
    // 复制完成后立刻释放锁，耗时的 MoveIt 规划不占用这把锁。
    geometry_msgs::msg::PoseArray target;
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      target = latest_target_;
    }

    if (target.poses.empty()) {
      response->success = false;
      response->message = "No target poses received";
      return;
    }

    // 检查并归一化四元数。
    // 归一化后四元数的长度为 1，MoveIt 更容易正确解释其旋转含义。
    for (auto & pose : target.poses) {
      if (!valid_pose(pose)) {
        response->success = false;
        response->message = "Target contains a non-finite pose or a zero quaternion";
        return;
      }

      const double norm = std::sqrt(
        pose.orientation.x * pose.orientation.x +
        pose.orientation.y * pose.orientation.y +
        pose.orientation.z * pose.orientation.z +
        pose.orientation.w * pose.orientation.w);
      pose.orientation.x /= norm;
      pose.orientation.y /= norm;
      pose.orientation.z /= norm;
      pose.orientation.w /= norm;
    }

    // 目标数据全部合法以后才回到初始关节位姿。这样即使收到错误的 Pose，
    // 机械臂也不会发生任何不必要的运动。
    if (!move_to_initial_joint_pose(response)) {
      return;
    }

    // 如果发布者填写了 header.frame_id，就使用发布者指定的坐标系；
    // 如果没有填写，则使用 reference_frame_ 参数，默认是 base_link。
    const std::string input_frame =
      target.header.frame_id.empty() ? reference_frame_ : target.header.frame_id;
    move_group_->setPoseReferenceFrame(input_frame);
    move_group_->setStartStateToCurrentState();

    if (!use_cartesian_path_ && target.poses.size() == 1) {
      execute_pose_plan(target.poses.front(), response);
      return;
    }

    // 无论是一个目标点还是多个轨迹点，都使用笛卡尔路径。
    //
    // 单点时，computeCartesianPath() 会从机械臂当前的末端位姿开始，
    // 按 eef_step_ 设置的步长插值到目标位姿。这样末端看起来是直接移向
    // 目标，不再让 OMPL 自由选择可能绕远的关节空间路径。
    //
    // 多点时，机械臂会按 waypoints 的顺序逐段做笛卡尔插值。
    // 规划过程仍然会检查碰撞、关节限制和逆运动学是否有解。
    execute_cartesian_path(target.poses, response);
  }

  void execute_pose_plan(
    const geometry_msgs::msg::Pose & pose,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    move_group_->setPoseTarget(pose, end_effector_link_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planning_result = move_group_->plan(plan);
    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
      response->success = false;
      response->message = "MoveIt failed to plan the real-robot approach pose";
      move_group_->clearPoseTargets();
      return;
    }
    const auto execution_result = move_group_->execute(plan);
    move_group_->clearPoseTargets();
    response->success = execution_result == moveit::core::MoveItErrorCode::SUCCESS;
    response->message = response->success ?
      "MoveIt approach trajectory executed" :
      "Controller failed to execute the approach trajectory";
  }

  void execute_cartesian_path(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    // RobotTrajectory 是 MoveIt 的轨迹消息，内部包含关节名称和每个轨迹点。
    moveit_msgs::msg::RobotTrajectory trajectory_message;

    // MoveIt 从当前机器人状态开始，依次连接 waypoints。
    // 返回值 fraction 表示成功计算出的路径比例：
    //   1.0：所有点都成功；
    //   0.5：大约只完成了一半；
    //   0.0：没有得到有效路径。
    const double fraction = move_group_->computeCartesianPath(
      waypoints, eef_step_, jump_threshold_, trajectory_message, true);

    if (fraction < min_cartesian_fraction_) {
      response->success = false;
      response->message = "Cartesian path fraction " + std::to_string(fraction) +
        " is below required " + std::to_string(min_cartesian_fraction_);
      return;
    }

    // computeCartesianPath 得到的轨迹可能没有完整的时间信息，但轨迹消息中
    // 已经包含用于规划的起始关节位置。真机上 MoveGroupInterface 自己的
    // CurrentStateMonitor 偶尔无法在这里再次取得状态，即使 /joint_states
    // 正在稳定发布；因此直接使用规划结果的第一个轨迹点构造起始状态。
    const auto & joint_trajectory = trajectory_message.joint_trajectory;
    if (joint_trajectory.joint_names.empty() || joint_trajectory.points.empty() ||
      joint_trajectory.points.front().positions.size() != joint_trajectory.joint_names.size())
    {
      response->success = false;
      response->message = "Cartesian trajectory does not contain a valid start state";
      return;
    }

    moveit::core::RobotState trajectory_start_state(move_group_->getRobotModel());
    trajectory_start_state.setToDefaultValues();
    trajectory_start_state.setVariablePositions(
      joint_trajectory.joint_names, joint_trajectory.points.front().positions);
    trajectory_start_state.update();

    // 把 ROS 消息转换成 MoveIt 的 RobotTrajectory 对象，方便进行时间参数化。
    robot_trajectory::RobotTrajectory robot_trajectory(
      move_group_->getRobotModel(), planning_group_);
    robot_trajectory.setRobotTrajectoryMsg(trajectory_start_state, trajectory_message);
    // 根据速度和加速度限制，为每个轨迹点计算 time_from_start、速度和加速度。
    trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        robot_trajectory, velocity_scaling_, acceleration_scaling_))
    {
      response->success = false;
      response->message = "Failed to add timestamps to Cartesian trajectory";
      return;
    }
    robot_trajectory.getRobotTrajectoryMsg(trajectory_message);

    // 时间参数完整后，执行整条笛卡尔轨迹。
    const auto execution_result = move_group_->execute(trajectory_message);
    response->success = execution_result == moveit::core::MoveItErrorCode::SUCCESS;
    response->message = response->success ?
      "Cartesian trajectory executed; fraction=" + std::to_string(fraction) :
      "Controller failed to execute Cartesian trajectory";
  }

  // 下面是节点运行时保存的配置参数。
  std::string planning_group_;
  std::string end_effector_link_;
  std::string reference_frame_;
  double velocity_scaling_;
  double acceleration_scaling_;
  double planning_time_;
  double current_state_wait_sec_;
  double eef_step_;
  double jump_threshold_;
  double min_cartesian_fraction_;
  bool use_cartesian_path_;
  bool initialize_before_execution_;
  bool initialize_on_startup_;
  double initial_joint_tolerance_;
  std::vector<double> initial_joint_positions_;
  bool initial_pose_executed_{false};

  // unique_ptr 表示 MoveGroupInterface 由本节点独占管理，节点析构时自动释放。
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  // ROS 2 通信对象：订阅者和服务端。
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr target_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;
  geometry_msgs::msg::PoseArray latest_target_;
  std::mutex target_mutex_;  // 保护 latest_target_ 的并发访问。
  std::atomic_bool executing_{false};  // 防止同时执行两条轨迹。
};

int main(int argc, char ** argv)
{
  // 初始化 ROS 2。所有 ROS 2 节点都必须先调用 rclcpp::init。
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveItPoseExecutor>();

  // 使用多线程执行器：一个线程可以处理服务回调，另一个线程可以处理
  // MoveIt action 的反馈和状态更新。MoveGroupInterface 的阻塞调用因此
  // 不会完全阻塞节点内部的 ROS 通信。
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  try {
    // 必须在 executor 已经开始 spin 后初始化 MoveIt，便于接收 robot_description
    // 和 MoveIt action/service 的通信。
    node->initialize();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize MoveIt: %s", error.what());
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  // 等待节点退出。通常由 Ctrl+C 触发 rclcpp shutdown，使 spin 返回。
  spinner.join();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
