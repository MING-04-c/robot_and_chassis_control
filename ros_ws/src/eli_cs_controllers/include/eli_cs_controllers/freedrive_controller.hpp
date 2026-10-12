#ifndef __ELITE_CS_CONTROLLERS_FREEDRIVE_CONTROLLER_HPP__
#define __ELITE_CS_CONTROLLERS_FREEDRIVE_CONTROLLER_HPP__

#include <eli_cs_controllers/freedrive_controller_parameters.hpp>

#include <atomic>
#include <controller_interface/controller_interface.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <thread>
#include <vector>

namespace ELITE_CS_CONTROLLER {

class FreedriveController : public controller_interface::ControllerInterface {
   public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    // controller_manager 的实时循环调用此函数；只有这里写硬件命令接口。
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_init() override;

   private:
    std::shared_ptr<freedrive_controller::ParamListener> freedrive_param_listener_;
    freedrive_controller::Params freedrive_params_;

    mutable std::chrono::seconds timeout_;
    rclcpp::TimerBase::SharedPtr freedrive_timer_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr freedrive_sub_;

    // 订阅回调与实时 update() 可能在不同线程执行。atomic 只传递两个简单标志，
    // 避免在回调里直接访问硬件命令接口。
    std::atomic<bool> is_new_request_;
    std::atomic<bool> is_freedrive_active_;

    // 控制器激活后从 command_interfaces_ 中找到并保存“借用”的接口引用。
    // optional 表示查找失败时没有有效接口，reference_wrapper 避免复制接口对象。
    std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> async_success_interface_;
    std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> start_interface_;
    std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> end_interface_;

    // 按全名查找接口，集中处理激活阶段的接口完整性校验。
    bool setReferenceWrapper(std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>& wapper,
                             const std::string& interface_name);
   
    // 看门狗：指定时间未收到 enable_freedrive 消息时请求退出 Freedrive。
    void startTimer();
};

}  // namespace ELITE_CS_CONTROLLER

#endif
