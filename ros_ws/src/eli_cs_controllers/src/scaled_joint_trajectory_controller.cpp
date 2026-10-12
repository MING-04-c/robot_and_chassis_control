#include <memory>
#include <vector>

#include "eli_cs_controllers/scaled_joint_trajectory_controller.hpp"

#include "lifecycle_msgs/msg/state.hpp"

namespace ELITE_CS_CONTROLLER {

// 控制器初始化阶段：创建参数监听器并读取参数
controller_interface::CallbackReturn ScaledJointTrajectoryController::on_init() {
    // 参数监听器负责从节点获取配置参数
    scaled_param_listener_ = std::make_shared<scaled_joint_trajectory_controller::ParamListener>(get_node());
    scaled_params_ = scaled_param_listener_->get_params();

    // 调用基类初始化逻辑，确保 JointTrajectoryController 的内部也被正确初始化
    return JointTrajectoryController::on_init();
}

// 扩展状态接口配置：在原有 JointTrajectoryController 的状态接口基础上，增加速度缩放接口
controller_interface::InterfaceConfiguration ScaledJointTrajectoryController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration conf;
    conf = JointTrajectoryController::state_interface_configuration();
    conf.names.push_back(scaled_params_.speed_scaling_interface_name);

    return conf;
}

// 激活阶段：初始化时间数据，用于后续控制周期的速度缩放计算
controller_interface::CallbackReturn ScaledJointTrajectoryController::on_activate(const rclcpp_lifecycle::State& state) {
    TimeData time_data;
    time_data.time = get_node()->now();
    time_data.period = rclcpp::Duration::from_nanoseconds(0);
    time_data.uptime = get_node()->now();
    time_data_.initRT(time_data);
    return JointTrajectoryController::on_activate(state);
}

// 每个控制周期都会调用 update()，执行轨迹计算与命令写入
controller_interface::return_type ScaledJointTrajectoryController::update(const rclcpp::Time& time,
                                                                          const rclcpp::Duration& period) {
    // 读取速度缩放状态接口值
    if (state_interfaces_.back().get_name() == scaled_params_.speed_scaling_interface_name) {
        scaling_factor_ = state_interfaces_.back().get_value();
    } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Speed scaling interface (%s) not found in hardware interface.",
                     scaled_params_.speed_scaling_interface_name.c_str());
    }

    // 如果控制器当前处于非激活状态，则跳过轨迹执行
    if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        return controller_interface::return_type::OK;
    }

    // 局部 lambda：计算每个关节当前与期望之间的误差
    auto compute_error_for_joint = [&](JointTrajectoryPoint& error, int index, const JointTrajectoryPoint& current,
                                       const JointTrajectoryPoint& desired) {
        if (joints_angle_wraparound_[index]) {
            // 处理角度环绕，确保误差在 [-pi, pi] 范围内
            error.positions[index] = angles::shortest_angular_distance(current.positions[index], desired.positions[index]);
        } else {
            error.positions[index] = desired.positions[index] - current.positions[index];
        }
        if (has_velocity_state_interface_ && (has_velocity_command_interface_ || has_effort_command_interface_)) {
            error.velocities[index] = desired.velocities[index] - current.velocities[index];
        }
        if (has_acceleration_state_interface_ && has_acceleration_command_interface_) {
            error.accelerations[index] = desired.accelerations[index] - current.accelerations[index];
        }
    };

    // 读取当前活动目标，避免实时与非实时线程间出现竞态条件
    const auto active_goal = *rt_active_goal_.readFromRT();

    // 检查是否有新的外部轨迹消息到来
    auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
    auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();
    if (current_external_msg != *new_external_msg && !(rt_has_pending_goal_ && !active_goal)) {
        fill_partial_goal(*new_external_msg);
        sort_to_local_joint_order(*new_external_msg);
        // TODO: 在这里可以集成额外的位置/速度补偿逻辑
        traj_external_point_ptr_->update(*new_external_msg);
    }

    // 从状态接口读取当前机器人状态
    state_current_.time_from_start.set__sec(0);
    read_state_from_state_interfaces(state_current_);

    // 仅在存在活动轨迹的情况下执行轨迹跟踪逻辑
    if (has_active_trajectory()) {
        // 计算本周期实际时间间隔，并根据速度缩放因子调整
        TimeData time_data;
        time_data.time = time;
        //跟上一次更新时间的差值，单位为纳秒
        rcl_duration_value_t t_period = (time_data.time - time_data_.readFromRT()->time).nanoseconds();
        time_data.period = rclcpp::Duration::from_nanoseconds(scaling_factor_ * t_period);
        time_data.uptime = time_data_.readFromRT()->uptime + time_data.period;

        // traj_time 用于轨迹采样，使用未缩放的实际时间差
        rclcpp::Time traj_time = time_data_.readFromRT()->uptime + rclcpp::Duration::from_nanoseconds(t_period);
        time_data_.reset();
        time_data_.initRT(time_data);

        bool first_sample = false;
        if (!traj_external_point_ptr_->is_sampled_already()) {
            first_sample = true;
            if (params_.open_loop_control) {
                // 开环控制时，起始点使用上一次命令值
                traj_external_point_ptr_->set_point_before_trajectory_msg(traj_time, last_commanded_state_);
            } else {
                // 闭环控制时，起始点使用当前真实状态
                traj_external_point_ptr_->set_point_before_trajectory_msg(traj_time, state_current_);
            }
        }

        // 根据 traj_time 从轨迹中采样目标点
        joint_trajectory_controller::TrajectoryPointConstIter start_segment_itr, end_segment_itr;
        const bool valid_point =
            traj_external_point_ptr_->sample(traj_time, interpolation_method_, state_desired_, start_segment_itr, end_segment_itr);

        if (valid_point) {
            const rclcpp::Time traj_start = traj_external_point_ptr_->time_from_start();
            const rclcpp::Time segment_time_from_start = traj_start + start_segment_itr->time_from_start;
            double time_difference = time.seconds() - segment_time_from_start.seconds();
            bool tolerance_violated_while_moving = false;
            bool outside_goal_tolerance = false;
            bool within_goal_time = true;
            const bool before_last_point = end_segment_itr != traj_external_point_ptr_->end();

            // 如果已经到达末点且命令超时，则中止执行
            if (!before_last_point && !rt_is_holding_ && cmd_timeout_ > 0.0 &&
                time_difference > cmd_timeout_) {
                RCLCPP_WARN(get_node()->get_logger(), "Aborted due to command timeout");
                traj_msg_external_point_ptr_.reset();
                traj_msg_external_point_ptr_.initRT(set_hold_position());
            }

            // 检查每个关节的状态容差与目标容差
            for (size_t index = 0; index < dof_; ++index) {
                compute_error_for_joint(state_error_, index, state_current_, state_desired_);
                if ((before_last_point || first_sample) &&
                    !check_state_tolerance_per_joint(state_error_, index, default_tolerances_.state_tolerance[index], true) &&
                    !rt_is_holding_) {
                    tolerance_violated_while_moving = true;
                }
                if (!before_last_point &&
                    !check_state_tolerance_per_joint(state_error_, index, default_tolerances_.goal_state_tolerance[index], false) &&
                    !rt_is_holding_) {
                    outside_goal_tolerance = true;
                    if (default_tolerances_.goal_time_tolerance != 0.0) {
                        if (time_difference > default_tolerances_.goal_time_tolerance) {
                            within_goal_time = false;
                        }
                    }
                }
            }

            // 如果误差在允许范围内，则写入新命令
            if (!tolerance_violated_while_moving && within_goal_time) {
                if (use_closed_loop_pid_adapter_) {
                    // 计算闭环 PID 的输出值
                    for (auto i = 0ul; i < dof_; ++i) {
                        tmp_command_[i] = (state_desired_.velocities[i] * ff_velocity_scale_[i]) +
                                          pids_[i]->computeCommand(state_error_.positions[i], state_error_.velocities[i],
                                                                   (uint64_t)period.nanoseconds());
                    }
                }
                if (has_position_command_interface_) {
                    assign_interface_from_point(joint_command_interface_[0], state_desired_.positions);
                }
                if (has_velocity_command_interface_) {
                    if (use_closed_loop_pid_adapter_) {
                        assign_interface_from_point(joint_command_interface_[1], tmp_command_);
                    } else {
                        assign_interface_from_point(joint_command_interface_[1], state_desired_.velocities);
                    }
                }
                if (has_acceleration_command_interface_) {
                    assign_interface_from_point(joint_command_interface_[2], state_desired_.accelerations);
                }
                if (has_effort_command_interface_) {
                    assign_interface_from_point(joint_command_interface_[3], tmp_command_);
                }
                last_commanded_state_ = state_desired_;
            }

            if (active_goal) {
                // 向 action 客户端发送反馈
                auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
                feedback->header.stamp = time;
                feedback->joint_names = params_.joints;
                feedback->actual = state_current_;
                feedback->desired = state_desired_;
                feedback->error = state_error_;
                active_goal->setFeedback(feedback);

                // 根据容差判断是否结束、成功或失败
                if (tolerance_violated_while_moving) {
                    auto result = std::make_shared<FollowJTrajAction::Result>();
                    result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
                    active_goal->setAborted(result);
                    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
                    rt_has_pending_goal_ = false;
                    RCLCPP_WARN(get_node()->get_logger(), "Aborted due to state tolerance violation");
                    traj_msg_external_point_ptr_.reset();
                    traj_msg_external_point_ptr_.initRT(set_hold_position());
                } else if (!before_last_point) {
                    if (!outside_goal_tolerance) {
                        auto res = std::make_shared<FollowJTrajAction::Result>();
                        res->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
                        active_goal->setSucceeded(res);
                        rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
                        rt_has_pending_goal_ = false;
                        RCLCPP_INFO(get_node()->get_logger(), "Goal reached, success!");
                        traj_msg_external_point_ptr_.reset();
                        traj_msg_external_point_ptr_.initRT(set_success_trajectory_point());
                    } else if (!within_goal_time) {
                        auto result = std::make_shared<FollowJTrajAction::Result>();
                        result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
                        active_goal->setAborted(result);
                        rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
                        rt_has_pending_goal_ = false;
                        RCLCPP_WARN(get_node()->get_logger(), "Aborted due goal_time_tolerance exceeding by %f seconds",
                                    time_difference);
                        traj_msg_external_point_ptr_.reset();
                        traj_msg_external_point_ptr_.initRT(set_hold_position());
                    }
                }
            } else if (tolerance_violated_while_moving && !rt_has_pending_goal_) {
                // 无活动 goal 且出现容差失败，则保持当前点
                RCLCPP_ERROR(get_node()->get_logger(), "Holding position due to state tolerance violation");
                traj_msg_external_point_ptr_.reset();
                traj_msg_external_point_ptr_.initRT(set_hold_position());
            } else if (!before_last_point && !within_goal_time && !rt_has_pending_goal_) {
                RCLCPP_ERROR(get_node()->get_logger(), "Exceeded goal_time_tolerance: holding position...");
                traj_msg_external_point_ptr_.reset();
                traj_msg_external_point_ptr_.initRT(set_hold_position());
            }
        }
    }

    // 发布当前轨迹状态给 base class 的状态话题
    publish_state(state_desired_, state_current_, state_error_);
    return controller_interface::return_type::OK;
}

}  // namespace ELITE_CS_CONTROLLER

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ELITE_CS_CONTROLLER::ScaledJointTrajectoryController, controller_interface::ControllerInterface)
