#ifndef _ELITE_CS_CONTROLLER__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_
#define _ELITE_CS_CONTROLLER__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_

#include "angles/angles.h"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "eli_cs_controllers/scaled_joint_trajectory_controller_parameters.hpp"

namespace ELITE_CS_CONTROLLER {
class ScaledJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController {
   public:
    ScaledJointTrajectoryController() = default;
    ~ScaledJointTrajectoryController() override = default;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    CallbackReturn on_init() override;

   protected:
    // 轨迹“虚拟时钟”。time 是本次真实控制时刻，period 是速度缩放后的增量，
    // uptime 是从轨迹开始累计的缩放时间。缩放为 0 时 uptime 不前进，轨迹暂停。
    struct TimeData {
        TimeData() : time(0.0), period(rclcpp::Duration::from_nanoseconds(0.0)), uptime(0.0) {}
        rclcpp::Time time;
        rclcpp::Duration period;
        rclcpp::Time uptime;
    };

   private:
    // 来自硬件状态接口的 [0, 1] 比例，例如 0.5 表示轨迹时间以半速累积。
    double scaling_factor_{};
    // 非实时线程和实时 update() 之间传递 TimeData，避免常规互斥锁影响控制周期。
    realtime_tools::RealtimeBuffer<TimeData> time_data_;

    std::shared_ptr<scaled_joint_trajectory_controller::ParamListener> scaled_param_listener_;
    scaled_joint_trajectory_controller::Params scaled_params_;

    /**
     * @brief 将轨迹点的一个数组（位置、速度、加速度或力矩）逐关节写入命令接口。
     *
     * @tparam T The type of the joint interface.
     * @param[out] joint_interface controller_manager 借给控制器的关节命令接口数组。
     * @param[in] trajectory_point_interface 当前采样出的期望值数组。
     */
    template <typename T>
    void assign_interface_from_point(const T& joint_interface, const std::vector<double>& trajectory_point_interface) {
        for (size_t index = 0; index < dof_; ++index) {
            joint_interface[index].get().set_value(trajectory_point_interface[index]);
        }
    }
};
}  // namespace ELITE_CS_CONTROLLER

#endif
