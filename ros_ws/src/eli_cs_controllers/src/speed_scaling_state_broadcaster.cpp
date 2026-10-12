#include "eli_cs_controllers/speed_scaling_state_broadcaster.hpp"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"


namespace ELITE_CS_CONTROLLER {
SpeedScalingStateBroadcaster::SpeedScalingStateBroadcaster() {}

controller_interface::CallbackReturn SpeedScalingStateBroadcaster::on_init() {
    try {
        // 参数类由 speed_scaling_state_broadcaster_parameters.yaml 自动生成。
        param_listener_ = std::make_shared<speed_scaling_state_broadcaster::ParamListener>(get_node());
        params_ = param_listener_->get_params();

        RCLCPP_INFO(get_node()->get_logger(), "Loading Elite SpeedScalingStateBroadcaster with tf_prefix: %s",
                    params_.tf_prefix.c_str());

    } catch (std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SpeedScalingStateBroadcaster::command_interface_configuration() const {
    // 广播器只读取状态，不向机器人写任何命令。
    return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration SpeedScalingStateBroadcaster::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // 名称必须与 cs.ros2_control.xacro 中 gpio "speed_scaling" 的 state_interface 对应。
    const std::string tf_prefix = params_.tf_prefix;
    config.names.push_back(tf_prefix + "speed_scaling/speed_scaling_factor");
    return config;
}

controller_interface::CallbackReturn SpeedScalingStateBroadcaster::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    if (!param_listener_) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
        return controller_interface::CallbackReturn::ERROR;
    }

    // 读取 launch/YAML 注入的参数；动态参数刷新后在此处取得最新副本。
    param_listener_->refresh_dynamic_parameters();

    // get parameters from the listener in case they were updated
    params_ = param_listener_->get_params();

    publish_rate_ = params_.state_publish_rate;

    RCLCPP_INFO(get_node()->get_logger(), "Publisher rate set to : %.1f Hz", publish_rate_);

    try {
        // "~" 是控制器私有命名空间，最终通常为 /speed_scaling_state_broadcaster/speed_scaling。
        speed_scaling_state_publisher_ =
            get_node()->create_publisher<std_msgs::msg::Float64>("~/speed_scaling", rclcpp::SystemDefaultsQoS());
    } catch (const std::exception& e) {
        // get_node() may throw, logging raw here
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpeedScalingStateBroadcaster::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpeedScalingStateBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SpeedScalingStateBroadcaster::update(const rclcpp::Time& /*time*/,
                                                                       const rclcpp::Duration& period) {
    // 只有在配置了正发布频率、且本周期长度超过发布间隔时才发布。
    // 注意：这里使用单个 period 判断，并未累计时间；这是原有实现的行为。
    if (publish_rate_ > 0.0 && period > rclcpp::Duration(1.0 / publish_rate_, 0.0)) {
        // 本控制器只申请了一个状态接口，因此下标 0 就是 speed_scaling_factor。
        // 硬件值通常是 0~1；此处乘 100 后按百分比发布。
        speed_scaling_state_msg_.data = state_interfaces_[0].get_value() * 100.0;

        // 该 publish 发生在控制器 update() 中，消息反映最近一次硬件 read() 的状态。
        speed_scaling_state_publisher_->publish(speed_scaling_state_msg_);
    }
    return controller_interface::return_type::OK;
}

}  // namespace ELITE_CS_CONTROLLER

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ELITE_CS_CONTROLLER::SpeedScalingStateBroadcaster, controller_interface::ControllerInterface)
