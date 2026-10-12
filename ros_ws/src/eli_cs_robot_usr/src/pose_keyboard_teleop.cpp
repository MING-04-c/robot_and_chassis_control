// 使用 X11 键盘窗口通过 MoveIt Servo 实时控制机械臂末端速度。
//
// 为什么不再读取终端 std::cin：
// 普通终端只能收到按键字符，无法可靠获得物理松键事件，因此必须依赖系统
// 自动重复和超时猜测。X11 窗口能分别收到 KeyPress 与 KeyRelease：按下立即
// 运动，保持按下持续运动，松开立即发送零速度。

#include <X11/XKBlib.h>
#include <X11/Xlib.h>
#include <X11/keysym.h>

// Xlib defines None as a macro, while rclcpp uses None as an enum member.
#undef None

#include <atomic>
#include <chrono>
#include <cstring>
#include <iterator>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class PoseKeyboardTeleop : public rclcpp::Node
{
public:
  PoseKeyboardTeleop()
  : Node("pose_keyboard_teleop")
  {
    command_frame_ = declare_parameter<std::string>("command_frame", "base_link");
    twist_topic_ = declare_parameter<std::string>(
      "twist_topic", "/servo_node/delta_twist_cmds");
    servo_start_service_ = declare_parameter<std::string>(
      "servo_start_service", "/servo_node/start_servo");
    servo_status_topic_ = declare_parameter<std::string>(
      "servo_status_topic", "/servo_node/status");
    linear_speed_ = declare_parameter<double>("linear_speed", 0.05);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.20);
    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);

    if (linear_speed_ <= 0.0 || angular_speed_ <= 0.0 || publish_rate_ <= 0.0) {
      throw std::invalid_argument("teleop speed and publish rate must be greater than zero");
    }

    twist_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      twist_topic_, rclcpp::QoS(10).reliable());
    servo_start_client_ = create_client<std_srvs::srv::Trigger>(servo_start_service_);
    servo_status_subscription_ = create_subscription<std_msgs::msg::Int8>(
      servo_status_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::Int8::SharedPtr message) {
        report_servo_status(message->data);
      });

    // 按固定频率持续发布当前速度。按键按住时 current_command_ 保持非零；
    // KeyRelease、空格或窗口失去焦点时，它会立即变为零。
    const auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_);
    publish_timer_ = create_wall_timer(
      publish_period,
      [this]() {
        publish_command();
        if (quit_requested_) {
          rclcpp::shutdown();
        }
      });

    // Servo 节点启动后默认没有开始计算，自动调用 start_servo 服务。
    start_timer_ = create_wall_timer(500ms, [this]() {try_start_servo();});

    // 定期显示键盘命令话题是否已有 Servo 订阅者。如果这里一直是 0，
    // 说明问题不在键盘事件，而是 Servo 节点没有启动或话题名不匹配。
    connection_timer_ = create_wall_timer(2s, [this]() {report_connections();});

    RCLCPP_INFO(
      get_logger(), "Keyboard window ready: linear=%.3f m/s, angular=%.3f rad/s",
      linear_speed_, angular_speed_);
  }

  void start_keyboard_window()
  {
    auto self = std::static_pointer_cast<PoseKeyboardTeleop>(shared_from_this());
    std::thread([self]() {self->window_loop();}).detach();
  }

private:
  void window_loop()
  {
    Display * display = XOpenDisplay(nullptr);
    if (display == nullptr) {
      RCLCPP_FATAL(get_logger(), "Cannot open X11 display; check the DISPLAY environment variable");
      quit_requested_ = true;
      return;
    }

    const int screen = DefaultScreen(display);
    const Window root = RootWindow(display, screen);
    const unsigned long black = BlackPixel(display, screen);
    const unsigned long white = WhitePixel(display, screen);
    const Window window = XCreateSimpleWindow(
      display, root, 120, 120, 640, 250, 1, black, white);

    XStoreName(display, window, "MoveIt Servo Keyboard Control");
    XSelectInput(
      display, window,
      ExposureMask | KeyPressMask | KeyReleaseMask | FocusChangeMask | StructureNotifyMask);

    Atom wm_delete = XInternAtom(display, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(display, window, &wm_delete, 1);

    // 请求 X11 只在真正松键时发送 KeyRelease，过滤系统自动重复产生的
    // Release/Press 对。即使窗口系统不支持，pressed_keys_ 集合也不会
    // 把重复 KeyPress 累积成多个待执行动作。
    Bool detectable_supported = False;
    XkbSetDetectableAutoRepeat(display, True, &detectable_supported);

    XMapRaised(display, window);
    XFlush(display);

    RCLCPP_INFO(
      get_logger(),
      "X11 keyboard window opened; click it once, then hold a motion key");

    while (rclcpp::ok() && !quit_requested_) {
      XEvent event;
      XNextEvent(display, &event);

      if (event.type == Expose) {
        draw_help(display, window);
      } else if (event.type == FocusOut) {
        // 切换到其他窗口时立即停止，避免按键释放事件发送给其他窗口后，
        // 本节点仍认为按键处于按下状态。
        clear_pressed_keys();
      } else if (event.type == KeyPress) {
        handle_key_press(XLookupKeysym(&event.xkey, 0));
      } else if (event.type == KeyRelease) {
        handle_key_release(XLookupKeysym(&event.xkey, 0));
      } else if (event.type == ClientMessage &&
        static_cast<Atom>(event.xclient.data.l[0]) == wm_delete)
      {
        request_quit();
      }
    }

    clear_pressed_keys();
    XDestroyWindow(display, window);
    XCloseDisplay(display);
  }

  void draw_help(Display * display, Window window) const
  {
    const char * lines[] = {
      "MoveIt Servo end-effector control (hold key to move, release to stop)",
      "W/S : X+ / X-      A/D : Y+ / Y-      R/F : Z+ / Z-",
      "I/K : Roll+ / -    J/L : Pitch+ / -  U/O : Yaw+ / -",
      "SPACE: Stop         Q: Quit",
      "The robot stops automatically when this window loses focus.",
    };

    GC gc = DefaultGC(display, DefaultScreen(display));
    for (std::size_t index = 0; index < std::size(lines); ++index) {
      XDrawString(
        display, window, gc, 20, 40 + static_cast<int>(index) * 38,
        lines[index], static_cast<int>(std::strlen(lines[index])));
    }
  }

  static char key_from_symbol(KeySym symbol)
  {
    switch (symbol) {
      case XK_w: case XK_W: return 'w';
      case XK_s: case XK_S: return 's';
      case XK_a: case XK_A: return 'a';
      case XK_d: case XK_D: return 'd';
      case XK_r: case XK_R: return 'r';
      case XK_f: case XK_F: return 'f';
      case XK_i: case XK_I: return 'i';
      case XK_k: case XK_K: return 'k';
      case XK_j: case XK_J: return 'j';
      case XK_l: case XK_L: return 'l';
      case XK_u: case XK_U: return 'u';
      case XK_o: case XK_O: return 'o';
      default: return 0;
    }
  }

  void handle_key_press(KeySym symbol)
  {
    if (symbol == XK_q || symbol == XK_Q || symbol == XK_Escape) {
      request_quit();
      return;
    }
    if (symbol == XK_space) {
      clear_pressed_keys();
      return;
    }

    const char key = key_from_symbol(symbol);
    if (key == 0) {
      return;
    }

    bool changed = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      changed = pressed_keys_.insert(key).second;
      rebuild_command_locked();
    }
    if (changed) {
      RCLCPP_INFO(get_logger(), "Key '%c' pressed", key);
      publish_command();
    }
  }

  void handle_key_release(KeySym symbol)
  {
    const char key = key_from_symbol(symbol);
    if (key == 0) {
      return;
    }

    bool changed = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      changed = pressed_keys_.erase(key) > 0;
      rebuild_command_locked();
    }
    if (changed) {
      RCLCPP_INFO(get_logger(), "Key '%c' released", key);
      publish_command();
    }
  }

  void rebuild_command_locked()
  {
    current_command_ = geometry_msgs::msg::Twist();
    current_command_.linear.x = linear_speed_ *
      (static_cast<int>(pressed_keys_.count('w')) - static_cast<int>(pressed_keys_.count('s')));
    current_command_.linear.y = linear_speed_ *
      (static_cast<int>(pressed_keys_.count('a')) - static_cast<int>(pressed_keys_.count('d')));
    current_command_.linear.z = linear_speed_ *
      (static_cast<int>(pressed_keys_.count('r')) - static_cast<int>(pressed_keys_.count('f')));
    current_command_.angular.x = angular_speed_ *
      (static_cast<int>(pressed_keys_.count('i')) - static_cast<int>(pressed_keys_.count('k')));
    current_command_.angular.y = angular_speed_ *
      (static_cast<int>(pressed_keys_.count('j')) - static_cast<int>(pressed_keys_.count('l')));
    current_command_.angular.z = angular_speed_ *
      (static_cast<int>(pressed_keys_.count('u')) - static_cast<int>(pressed_keys_.count('o')));
  }

  void clear_pressed_keys()
  {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      pressed_keys_.clear();
      rebuild_command_locked();
    }
    publish_command();
  }

  void request_quit()
  {
    clear_pressed_keys();
    quit_requested_ = true;
  }

  void publish_command()
  {
    if (!servo_started_) {
      return;
    }

    geometry_msgs::msg::TwistStamped message;
    message.header.stamp = now();
    message.header.frame_id = command_frame_;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      message.twist = current_command_;
    }
    twist_publisher_->publish(message);
  }

  void try_start_servo()
  {
    if (servo_started_ || start_request_pending_) {
      return;
    }
    if (!servo_start_client_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for Servo service '%s'", servo_start_service_.c_str());
      return;
    }

    start_request_pending_ = true;
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    servo_start_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        start_request_pending_ = false;
        const auto response = future.get();
        if (response->success) {
          servo_started_ = true;
          start_timer_->cancel();
          RCLCPP_INFO(get_logger(), "MoveIt Servo started; keyboard commands are active");
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to start MoveIt Servo: %s", response->message.c_str());
        }
      });
  }

  void report_connections()
  {
    const auto subscriber_count = twist_publisher_->get_subscription_count();
    if (subscriber_count == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No subscriber on '%s'; MoveIt Servo cannot receive keyboard commands",
        twist_topic_.c_str());
      return;
    }

    if (!connection_reported_) {
      connection_reported_ = true;
      RCLCPP_INFO(
        get_logger(), "Servo command connection ready: '%s' has %zu subscriber(s)",
        twist_topic_.c_str(), subscriber_count);
    }
  }

  void report_servo_status(int8_t status)
  {
    if (status == last_servo_status_) {
      return;
    }
    last_servo_status_ = status;

    const char * description = "Unknown status";
    switch (status) {
      case -1: description = "Invalid Servo status"; break;
      case 0: description = "No warnings"; break;
      case 1: description = "Approaching singularity; decelerating"; break;
      case 2: description = "Too close to singularity; motion halted"; break;
      case 3: description = "Approaching collision; decelerating"; break;
      case 4: description = "Collision detected; motion halted"; break;
      case 5: description = "Joint position or velocity bound; motion halted"; break;
      case 6: description = "Leaving singularity; decelerating"; break;
      default: break;
    }

    if (status == 0) {
      RCLCPP_INFO(get_logger(), "Servo status %d: %s", status, description);
    } else {
      RCLCPP_WARN(get_logger(), "Servo status %d: %s", status, description);
    }
  }

  std::string command_frame_;
  std::string twist_topic_;
  std::string servo_start_service_;
  std::string servo_status_topic_;
  double linear_speed_{0.05};
  double angular_speed_{0.20};
  double publish_rate_{50.0};

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr servo_status_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  rclcpp::TimerBase::SharedPtr connection_timer_;

  std::mutex command_mutex_;
  std::set<char> pressed_keys_;
  geometry_msgs::msg::Twist current_command_;
  std::atomic_bool servo_started_{false};
  std::atomic_bool quit_requested_{false};
  bool start_request_pending_{false};
  bool connection_reported_{false};
  int8_t last_servo_status_{-127};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<PoseKeyboardTeleop>();
    node->start_keyboard_window();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("pose_keyboard_teleop"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
