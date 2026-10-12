#include <algorithm>
#include <cmath>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <ctime>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "eli_cs_robot_serial/srv/write_serial.hpp"

class DetactorNode final : public rclcpp::Node
{
public:
  DetactorNode() : Node("detactor_node")
  {
    // 探测器协议：发送 .d0<:>，设备返回 .e + 数值 + <:>。
    request_command_ = declare_parameter<std::string>("request_command", ".d0<:>");
    frame_start_ = declare_parameter<std::string>("frame_start", ".e");
    frame_end_ = declare_parameter<std::string>("frame_end", "<:>");
    // 当前滚动窗口取整帧最后 10 个数，窗口内从第一个变化值开始保存。
    latest_window_size_ = declare_parameter<int>("latest_window_size", 10);
    request_period_ = declare_parameter<double>("request_period", 1.0);
    auto_request_ = declare_parameter<bool>("auto_request", true);
    data_directory_ = expand_home(
      declare_parameter<std::string>("data_directory", "~/.ros/eli_cs_robot_serial/detactor"));

    const auto rx_topic = declare_parameter<std::string>("rx_topic", "/detactor/rx");
    const auto frame_topic = declare_parameter<std::string>("frame_topic", "/detactor/frame");
    const auto latest_topic = declare_parameter<std::string>("latest_topic", "/detactor/latest");
    const auto values_topic = declare_parameter<std::string>("latest_values_topic", "/detactor/latest_values");
    const auto write_service = declare_parameter<std::string>("write_service", "/detactor/write");
    const auto trigger_topic = declare_parameter<std::string>("trigger_topic", "/detactor/acquire_trigger");
    const auto trigger_service = declare_parameter<std::string>("trigger_service", "/detactor/acquire");

    frame_pub_ = create_publisher<std_msgs::msg::String>(frame_topic, 10);
    latest_pub_ = create_publisher<std_msgs::msg::String>(latest_topic, 10);
    values_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(values_topic, 10);
    write_client_ = create_client<eli_cs_robot_serial::srv::WriteSerial>(write_service);

    // 任意节点发布 Empty 即触发一次采集。
    trigger_sub_ = create_subscription<std_msgs::msg::Empty>(
      trigger_topic, 10, [this](const std_msgs::msg::Empty::SharedPtr) { request_measurement(); });
    trigger_srv_ = create_service<std_srvs::srv::Trigger>(
      trigger_service,
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = request_measurement();
        response->message = response->success ? "Measurement request sent" : "Serial write service is unavailable";
      });
    rx_sub_ = create_subscription<std_msgs::msg::String>(
      rx_topic, 100, [this](const std_msgs::msg::String::SharedPtr message) { consume(message->data); });

    if (auto_request_ && request_period_ > 0.0) {
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(request_period_)),
        [this]() { request_measurement(); });
    }
    RCLCPP_INFO(get_logger(),
      "Detactor collector ready: auto_request=%s, every %.3f s, command='%s', write_service='%s', rx_topic='%s', latest_window_size=%d, data='%s'",
      auto_request_ ? "true" : "false", request_period_, request_command_.c_str(),
      write_service.c_str(), rx_topic.c_str(), latest_window_size_, data_directory_.c_str());
  }

private:
  static std::string expand_home(const std::string &path)
  {
    if (path.rfind("~/", 0) != 0) return path;
    const char *home = std::getenv("HOME");
    return home == nullptr ? path : std::string(home) + path.substr(1);
  }

  bool request_measurement()
  {
    if (!write_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for serial write service; detector request not sent");
      return false;
    }
    auto request = std::make_shared<eli_cs_robot_serial::srv::WriteSerial::Request>();
    request->data = request_command_;
    auto future = write_client_->async_send_request(request);
    (void)future;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "Acquisition command sent: '%s'", request_command_.c_str());
    return true;
  }

  void consume(const std::string &chunk)
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    rx_buffer_ += chunk;

    // 一次串口 read 可能只收到半帧，也可能包含多帧，因此按头尾循环取完整帧。
    while (true) {
      const auto start = rx_buffer_.find(frame_start_);
      if (start == std::string::npos) {
        keep_possible_prefix();
        return;
      }
      if (start > 0) rx_buffer_.erase(0, start);
      const auto end = rx_buffer_.find(frame_end_, frame_start_.size());
      if (end == std::string::npos) return;
      const auto frame_length = end + frame_end_.size();
      const auto frame = rx_buffer_.substr(0, frame_length);
      rx_buffer_.erase(0, frame_length);
      process_frame(frame);
    }
  }

  void keep_possible_prefix()
  {
    // 保留可能是半个帧头的尾部，避免分包时丢掉 '.e' 的第一个字符。
    const auto keep = std::min(frame_start_.size() > 0 ? frame_start_.size() - 1 : 0UL, rx_buffer_.size());
    if (keep == 0) rx_buffer_.clear();
    else rx_buffer_ = rx_buffer_.substr(rx_buffer_.size() - keep);
  }

  static std::string trim(std::string value)
  {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
  }

  static std::vector<std::string> parse_tokens(const std::string &frame, size_t start, size_t length)
  {
    std::vector<std::string> tokens;
    std::string payload = frame.substr(start, length);
    std::replace(payload.begin(), payload.end(), ',', ';');
    std::stringstream input(payload);
    std::string token;
    while (std::getline(input, token, ';')) {
      token = trim(token);
      if (!token.empty()) tokens.push_back(token);
    }
    return tokens;
  }

  static bool to_double(const std::string &text, double &value)
  {
    try {
      size_t parsed = 0;
      value = std::stod(text, &parsed);
      return parsed == text.size();
    } catch (const std::exception &) {
      return false;
    }
  }

  static std::string join_tokens(const std::vector<std::string> &tokens)
  {
    std::ostringstream output;
    for (size_t i = 0; i < tokens.size(); ++i) {
      if (i != 0) output << ';';
      output << tokens[i];
    }
    return output.str();
  }

  void process_frame(const std::string &frame)
  {
    if (latest_window_size_ <= 0 || frame.size() < frame_start_.size() + frame_end_.size()) return;
    const auto payload_start = frame_start_.size();
    const auto payload_length = frame.size() - frame_start_.size() - frame_end_.size();
    const auto tokens = parse_tokens(frame, payload_start, payload_length);
    if (tokens.empty()) {
      RCLCPP_WARN(get_logger(), "Received frame without numeric values: %s", frame.c_str());
      return;
    }
    ++acquisition_count_;

    frame_pub_->publish(std_msgs::msg::String().set__data(frame));
    if (tokens.size() < static_cast<size_t>(latest_window_size_)) {
      RCLCPP_WARN(get_logger(), "Received %zu values, fewer than latest_window_size=%d",
        tokens.size(), latest_window_size_);
      return;
    }
    // 设备返回的是滚动历史，当前结果位于整帧最后 10 个数。
    const auto latest_begin = tokens.size() - static_cast<size_t>(latest_window_size_);
    std::vector<std::string> latest_tokens(tokens.begin() + latest_begin,
      tokens.end());
    for (const auto &token : tokens) {
      double value = 0.0;
      if (!to_double(token, value)) {
        RCLCPP_WARN(get_logger(), "Invalid numeric token '%s' in frame", token.c_str());
        return;
      }
    }

    save_latest(latest_tokens, acquisition_count_);
  }

  static std::string timestamp_for_filename()
  {
    const auto system_now = std::chrono::system_clock::now();
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
      system_now.time_since_epoch()) % 1000;
    const auto time = std::chrono::system_clock::to_time_t(system_now);
    std::tm local_time{};
    localtime_r(&time, &local_time);
    std::ostringstream output;
    output << std::put_time(&local_time, "%Y%m%d_%H%M%S")
           << '_' << std::setfill('0') << std::setw(3) << milliseconds.count();
    return output.str();
  }

  bool latest_changed(const std::vector<std::string> &latest_tokens) const
  {
    return !has_saved_latest_ || latest_tokens != last_saved_latest_tokens_;
  }

  static std::string changed_suffix(const std::vector<std::string> &latest_tokens,
    const std::vector<std::string> &previous_tokens, bool has_previous)
  {
    size_t first_changed = 0;
    if (has_previous) {
      while (first_changed < latest_tokens.size() &&
        first_changed < previous_tokens.size() &&
        latest_tokens[first_changed] == previous_tokens[first_changed]) {
        ++first_changed;
      }
    }
    std::ostringstream output;
    output << "start_" << (first_changed + 1) << '=';
    for (size_t i = first_changed; i < latest_tokens.size(); ++i) {
      if (i != first_changed) output << ';';
      output << latest_tokens[i];
    }
    return output.str();
  }

  void save_latest(const std::vector<std::string> &latest_tokens,
    uint64_t acquisition_count)
  {
    try {
      // 只比较最后一组；最后一组内变化一个或多个数都算新测量。
      if (!latest_changed(latest_tokens)) return;
      std::filesystem::create_directories(data_directory_);
      const auto changed = changed_suffix(
        latest_tokens, last_saved_latest_tokens_, has_saved_latest_);

      // latest.txt 是滚动窗口：新变化值放在最前面，旧值向后移动。
      // 首次收到数据时，设备帧的末尾是最新值，因此反转后保存为“新到旧”。
      if (!has_saved_latest_) {
        latest_display_tokens_ = latest_tokens;
        std::reverse(latest_display_tokens_.begin(), latest_display_tokens_.end());
      } else {
        size_t changed_begin = 0;
        while (changed_begin < latest_tokens.size() &&
          changed_begin < last_saved_latest_tokens_.size() &&
          latest_tokens[changed_begin] == last_saved_latest_tokens_[changed_begin]) {
          ++changed_begin;
        }
        for (size_t i = latest_tokens.size(); i > changed_begin; --i) {
          latest_display_tokens_.insert(latest_display_tokens_.begin(), latest_tokens[i - 1]);
        }
        if (latest_display_tokens_.size() > static_cast<size_t>(latest_window_size_)) {
          latest_display_tokens_.resize(static_cast<size_t>(latest_window_size_));
        }
      }
      const auto latest_text = join_tokens(latest_display_tokens_);

      // 最新数据始终只有一行，设备原始数字格式不被改写。
      std::ofstream latest_file(data_directory_ + "/latest.txt", std::ios::trunc);
      latest_file << latest_text << '\n';

      std_msgs::msg::String latest_message;
      latest_message.data = latest_text;
      latest_pub_->publish(latest_message);
      std_msgs::msg::Float64MultiArray latest_values;
      latest_values.data.reserve(latest_display_tokens_.size());
      for (const auto &token : latest_display_tokens_) {
        double value = 0.0;
        if (to_double(token, value)) latest_values.data.push_back(value);
      }
      values_pub_->publish(latest_values);

      // 一个运行周期只建立一个文件；每行只记录变化的下标和值，最新行在最前面。
      if (measurement_file_.empty()) {
        measurement_file_ = data_directory_ + "/measurement_" + timestamp_for_filename() + ".txt";
      }
      std::string old_content;
      if (std::filesystem::exists(measurement_file_)) {
        std::ifstream old_file(measurement_file_, std::ios::binary);
        old_content.assign(std::istreambuf_iterator<char>(old_file), {});
      }
      const auto temporary_file = measurement_file_ + ".tmp";
      std::ofstream measurement(temporary_file, std::ios::trunc);
      measurement << acquisition_count << ';' << changed << '\n';
      measurement << old_content;
      measurement.close();
      std::filesystem::rename(temporary_file, measurement_file_);

      last_saved_latest_tokens_ = latest_tokens;
      has_saved_latest_ = true;
      RCLCPP_INFO(get_logger(), "New detector data (window=%d): %s",
        latest_window_size_, latest_text.c_str());
    } catch (const std::exception &error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Cannot save detector data in '%s': %s", data_directory_.c_str(), error.what());
    }
  }

  std::string request_command_;
  std::string frame_start_;
  std::string frame_end_;
  int latest_window_size_;
  double request_period_;
  bool auto_request_;
  uint64_t acquisition_count_{0};
  std::string data_directory_;
  std::string measurement_file_;
  std::vector<std::string> last_saved_latest_tokens_;
  std::vector<std::string> latest_display_tokens_;
  bool has_saved_latest_{false};
  std::string rx_buffer_;
  std::mutex buffer_mutex_;
  rclcpp::Client<eli_cs_robot_serial::srv::WriteSerial>::SharedPtr write_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rx_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr frame_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr latest_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr values_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetactorNode>());
  rclcpp::shutdown();
  return 0;
}
