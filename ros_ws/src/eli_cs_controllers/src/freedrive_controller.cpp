#include "eli_cs_controllers/freedrive_controller.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/logging.hpp>

namespace ELITE_CS_CONTROLLER {

controller_interface::CallbackReturn FreedriveController::on_init() {
    try {
        // 参数类由 freedrive_controller_parameters.yaml 自动生成。
        freedrive_param_listener_ = std::make_shared<freedrive_controller::ParamListener>(get_node());
        freedrive_params_ = freedrive_param_listener_->get_params();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration FreedriveController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    const std::string tf_prefix = freedrive_params_.tf_prefix;
    timeout_ = std::chrono::seconds(freedrive_params_.inactive_timeout);

    // 申请三个命令接口。它们由 cs.ros2_control.xacro 声明、硬件驱动实际实现。
    // async_success 在当前代码中仅被查找保存，start/end 是真正写入的触发接口。
    config.names.emplace_back(tf_prefix + "freedrive_mode/freedrive_async_success");
    config.names.emplace_back(tf_prefix + "freedrive_mode/freedrive_start_cmd");
    config.names.emplace_back(tf_prefix + "freedrive_mode/freedrive_end_cmd");

    return config;
}

controller_interface::InterfaceConfiguration FreedriveController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::NONE;
    return config;
}

void FreedriveController::startTimer() {
    if (!freedrive_timer_) {
        // 定时器回调不是实时控制循环；它只设置 atomic 标志，真正写硬件留给 update()。
        auto timeout_func = [&](){
            if(is_freedrive_active_) {
                RCLCPP_INFO(get_node()->get_logger(), "Freedrive will be deactivated because receiving command timeout.");
                is_freedrive_active_ = false;
                is_new_request_ = true;
            }
        };
        freedrive_timer_ = get_node()->create_wall_timer(timeout_, timeout_func);
    }
}

controller_interface::CallbackReturn FreedriveController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    // 收到 Bool 消息时只更新状态机。这样 ROS 回调线程不会与 controller_manager
    // 的实时线程同时写同一个 command interface。
    auto enable_freedrive_cb = [&](const std_msgs::msg::Bool::SharedPtr msg) {
        if (get_node()->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            if (msg->data) {
                // is_new_request_ 表示上一次请求尚未被 update() 消费，避免重复触发。
                if ((!is_freedrive_active_) && (!is_new_request_)) {
                    is_freedrive_active_ = true;
                    is_new_request_ = true;
                    startTimer();
                }
            } else {
                if (is_freedrive_active_ && (!is_new_request_)) {
                    is_freedrive_active_ = false;
                    is_new_request_ = true;
                }
            }
        }
        // true/false 的持续消息都会重置看门狗；停止发送则在 timeout 后自动退出。
        if (freedrive_timer_) {
            freedrive_timer_->reset(); 
        }
    };
    freedrive_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>("~/enable_freedrive", 10, enable_freedrive_cb);

    if (!freedrive_param_listener_) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during configuration");
        return controller_interface::CallbackReturn::ERROR;
    }

    // Update the dynamic map parameters
    freedrive_param_listener_->refresh_dynamic_parameters();

    // Get parameters from the listener in case they were updated
    freedrive_params_ = freedrive_param_listener_->get_params();

    return ControllerInterface::on_configure(previous_state);
}

bool FreedriveController::setReferenceWrapper(
    std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>& wapper, const std::string& interface_name) {
    // command_interfaces_ 的顺序不应被假定；按完整接口名查找更稳妥。
    auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                           [&](auto& interface) { return (interface.get_name() == interface_name); });
    if (it != command_interfaces_.end()) {
        wapper = *it;
    } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Did not find '%s' in command interfaces.", interface_name.c_str());
        return false;
    }
    return true;
}

controller_interface::CallbackReturn FreedriveController::on_activate(const rclcpp_lifecycle::State& state) {
    // 每次激活从关闭状态开始，并缓存 controller_manager 借出的命令接口。
    is_new_request_ = false;
    is_freedrive_active_ = false;

    if (!setReferenceWrapper(async_success_interface_, freedrive_params_.tf_prefix + "freedrive_mode/freedrive_async_success")) {
        return controller_interface::CallbackReturn::ERROR;
    }

    if (!setReferenceWrapper(start_interface_, freedrive_params_.tf_prefix + "freedrive_mode/freedrive_start_cmd")) {
        return controller_interface::CallbackReturn::ERROR;
    }

    if (!setReferenceWrapper(end_interface_, freedrive_params_.tf_prefix + "freedrive_mode/freedrive_end_cmd")) {
        return controller_interface::CallbackReturn::ERROR;
    }

    return ControllerInterface::on_activate(state);
}

controller_interface::CallbackReturn FreedriveController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
    // cleanup 是最后的保护动作：若接口有效，明确向硬件发送退出 Freedrive 的命令。
    if (end_interface_.has_value()) {
        end_interface_->get().set_value(1.0);
        return controller_interface::CallbackReturn::SUCCESS;
    } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Not init to end command interface.");
        return controller_interface::CallbackReturn::ERROR;
    }
}

controller_interface::CallbackReturn FreedriveController::on_deactivate(const rclcpp_lifecycle::State&) {
    // 停用后不再接受状态变化，并销毁看门狗。原实现不在此写 end_cmd。
    is_freedrive_active_ = false;
    freedrive_timer_.reset();
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type FreedriveController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    // 只有有新请求时才写一次脉冲命令。硬件驱动会在随后 write() 中发送它。
    if (is_new_request_) {
        if (is_freedrive_active_) {
            if (end_interface_.has_value() && end_interface_->get().get_value() == 1.0) {
                RCLCPP_INFO(get_node()->get_logger(), "Freedrive mode end by hardware, ending request.");
                is_freedrive_active_ = false;
                return controller_interface::return_type::OK;
            } else {
                RCLCPP_INFO(get_node()->get_logger(), "Received command to start Freedrive Mode.");

                // Set command interface to enable
                start_interface_->get().set_value(1.0);
            }

        } else {
            RCLCPP_INFO(get_node()->get_logger(), "Received command to stop Freedrive Mode.");

            end_interface_->get().set_value(1.0);
        }
        is_new_request_ = false;
    }

    return controller_interface::return_type::OK;
}

}  // namespace ELITE_CS_CONTROLLER

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ELITE_CS_CONTROLLER::FreedriveController, controller_interface::ControllerInterface)
