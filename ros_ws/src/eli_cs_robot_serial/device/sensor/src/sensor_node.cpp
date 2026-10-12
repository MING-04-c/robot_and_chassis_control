#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <mutex>
#include <netdb.h>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "eli_cs_robot_serial/srv/write_serial.hpp"

class SensorNode final : public rclcpp::Node
{
public:
  SensorNode() : Node("sensor_node")
  {
    transport_ = declare_parameter<std::string>("transport", "usb");
    device_ = declare_parameter<std::string>("device", "/dev/ttyUSB0");
    host_ = declare_parameter<std::string>("host", "192.168.1.100");
    port_ = declare_parameter<int>("port", 4001);
    reconnect_period_ms_ = declare_parameter<int>("reconnect_period_ms", 1000);
    tcp_keepalive_ = declare_parameter<bool>("tcp_keepalive", true);
    tcp_no_delay_ = declare_parameter<bool>("tcp_no_delay", true);
    baudrate_ = declare_parameter<int>("baudrate", 460800);
    data_bits_ = declare_parameter<int>("data_bits", 8);
    stop_bits_ = declare_parameter<int>("stop_bits", 1);
    parity_ = declare_parameter<std::string>("parity", "none");
    frame_id_ = declare_parameter<std::string>("frame_id", "tool0");
    publish_wrench_ = declare_parameter<bool>("publish_wrench", false);
    data_format_ = declare_parameter<std::string>("data_format", "float32_be");
    data_offset_ = declare_parameter<int>("data_offset", 3);
    frame_size_ = declare_parameter<int>("frame_size", 29);
    request_period_ = declare_parameter<double>("request_period", 0.0);
    command_delay_ms_ = declare_parameter<int>("command_delay_ms", 300);
    zero_wait_ms_ = declare_parameter<int>("zero_wait_ms", 1000);
    unit_switch_delay_ms_ = declare_parameter<int>("unit_switch_delay_ms", 300);
    stop_exit_delay_ms_ = declare_parameter<int>("stop_exit_delay_ms", 1000);
    auto_start_ = declare_parameter<bool>("auto_start", true);
    zero_before_request_ = declare_parameter<bool>("zero_before_request", true);
    unit_ = declare_parameter<std::string>("unit", "N");
    baud_code_ = declare_parameter<int>("baud_code", 0);

    if (transport_ != "usb" && transport_ != "tcp") {
      throw std::runtime_error("transport must be 'usb' or 'tcp'");
    }
    if (transport_ == "tcp" && (host_.empty() || port_ <= 0 || port_ > 65535)) {
      throw std::runtime_error("TCP host must be non-empty and port must be in [1, 65535]");
    }

    const auto raw_topic = declare_parameter<std::string>("raw_topic", "/sensor/rx_bytes");
    const auto raw_rx_topic = declare_parameter<std::string>("raw_rx_topic", "/sensor/raw_rx_bytes");
    const auto raw_rx_hex_topic = declare_parameter<std::string>("raw_rx_hex_topic", "/sensor/raw_rx");
    const auto hex_topic = declare_parameter<std::string>("hex_topic", "/sensor/rx");
    const auto wrench_topic = declare_parameter<std::string>("wrench_topic", "/sensor/wrench");
    raw_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(raw_topic, 10);
    raw_rx_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(raw_rx_topic, 10);
    raw_rx_hex_pub_ = create_publisher<std_msgs::msg::String>(raw_rx_hex_topic, 10);
    hex_pub_ = create_publisher<std_msgs::msg::String>(hex_topic, 10);
    if (publish_wrench_) wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(wrench_topic, 10);

    write_srv_ = create_service<eli_cs_robot_serial::srv::WriteSerial>(
      "/sensor/write", std::bind(&SensorNode::write_hex, this,
      std::placeholders::_1, std::placeholders::_2));
    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/sensor/start", std::bind(&SensorNode::start_callback, this,
      std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/sensor/stop", std::bind(&SensorNode::stop_callback, this,
      std::placeholders::_1, std::placeholders::_2));
    zero_srv_ = create_service<std_srvs::srv::Trigger>(
      "/sensor/zero", std::bind(&SensorNode::zero_callback, this,
      std::placeholders::_1, std::placeholders::_2));
    unit_srv_ = create_service<std_srvs::srv::Trigger>(
      "/sensor/toggle_unit", std::bind(&SensorNode::unit_callback, this,
      std::placeholders::_1, std::placeholders::_2));
    baud_srv_ = create_service<std_srvs::srv::Trigger>(
      "/sensor/set_baud", std::bind(&SensorNode::baud_callback, this,
      std::placeholders::_1, std::placeholders::_2));

    reader_ = std::thread([this]() { read_loop(); });
    if (request_period_ > 0.0) {
      timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(request_period_)), [this]() { request_once(); });
    }
    if (transport_ == "tcp") {
      RCLCPP_INFO(
        get_logger(), "Sensor ready: transport=tcp, endpoint=%s:%d, unit=%s, raw=%s",
        host_.c_str(), port_, unit_.c_str(), raw_topic.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(), "Sensor ready: transport=usb, device=%s @ %d, unit=%s, raw=%s",
        device_.c_str(), baudrate_, unit_.c_str(), raw_topic.c_str());
    }
  }

  ~SensorNode() override
  {
    shutdown_sensor();
  }

public:
  void shutdown_sensor()
  {
    if (shutdown_started_.exchange(true)) return;
    // 协议规定：先停止采集，再退出 debug，最后释放串口。
    RCLCPP_INFO(get_logger(), "Stopping sensor: stop acquisition, then exit debug");
    send_command(acquisition_stop_command());
    std::this_thread::sleep_for(std::chrono::milliseconds(stop_exit_delay_ms_));
    send_command(debug_exit_command());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    running_ = false;
    close_connection();
    if (reader_.joinable() && reader_.get_id() != std::this_thread::get_id()) reader_.join();
  }

private:
  static std::vector<uint8_t> parse_hex(const std::string &text)
  {
    std::vector<uint8_t> result; int high = -1;
    auto digit = [](char c) { if (c >= '0' && c <= '9') return c - '0'; if (c >= 'A' && c <= 'F') return c - 'A' + 10; if (c >= 'a' && c <= 'f') return c - 'a' + 10; return -1; };
    for (size_t i = 0; i < text.size(); ++i) {
      if (std::isspace(static_cast<unsigned char>(text[i])) || text[i] == ',' || text[i] == ';') continue;
      if (text[i] == '0' && i + 1 < text.size() && (text[i + 1] == 'x' || text[i + 1] == 'X')) { ++i; continue; }
      const int value = digit(text[i]); if (value < 0) return {};
      if (high < 0) high = value; else { result.push_back(static_cast<uint8_t>((high << 4) | value)); high = -1; }
    }
    return high < 0 ? result : std::vector<uint8_t>{};
  }

  static std::string hex_string(const std::vector<uint8_t> &data)
  {
    static const char digits[] = "0123456789ABCDEF"; std::ostringstream out;
    for (size_t i = 0; i < data.size(); ++i) { if (i) out << ' '; out << digits[data[i] >> 4] << digits[data[i] & 15]; }
    return out.str();
  }

  static speed_t speed(int value)
  {
    switch (value) { case 9600: return B9600; case 19200: return B19200; case 38400: return B38400; case 57600: return B57600; case 115200: return B115200; case 230400: return B230400; case 460800: return B460800; case 921600: return B921600; default: return 0; }
  }

  bool open_connection()
  {
    if (transport_ == "tcp") {
      addrinfo hints{}; hints.ai_socktype = SOCK_STREAM; hints.ai_family = AF_UNSPEC;
      addrinfo *result = nullptr; const auto service = std::to_string(port_);
      if (getaddrinfo(host_.c_str(), service.c_str(), &hints, &result) != 0) return false;
      int fd = -1;
      for (auto *entry = result; entry; entry = entry->ai_next) { fd = socket(entry->ai_family, entry->ai_socktype, entry->ai_protocol); if (fd >= 0 && connect(fd, entry->ai_addr, entry->ai_addrlen) == 0) break; if (fd >= 0) close(fd); fd = -1; }
      freeaddrinfo(result); if (fd < 0) return false;
      if (tcp_keepalive_) {
        int enabled = 1;
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &enabled, sizeof(enabled));
      }
      if (tcp_no_delay_) {
        int enabled = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
      }
      fcntl(fd, F_SETFL, O_NONBLOCK); std::lock_guard<std::mutex> lock(mutex_); fd_ = fd; return true;
    }
    const auto baud = speed(baudrate_); if (!baud) { RCLCPP_ERROR(get_logger(), "Unsupported baudrate: %d", baudrate_); return false; }
    const int fd = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK); if (fd < 0) return false;
    termios tty{}; if (tcgetattr(fd, &tty) != 0) { close(fd); return false; } cfmakeraw(&tty); cfsetispeed(&tty, baud); cfsetospeed(&tty, baud); tty.c_cflag |= CLOCAL | CREAD; tty.c_cflag &= ~(CSTOPB | PARENB | PARODD | CSIZE); tty.c_cflag |= data_bits_ == 7 ? CS7 : CS8; if (stop_bits_ == 2) tty.c_cflag |= CSTOPB; if (parity_ == "odd") tty.c_cflag |= PARENB | PARODD; else if (parity_ == "even") tty.c_cflag |= PARENB; tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 1; if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return false; }
    std::lock_guard<std::mutex> lock(mutex_); fd_ = fd; return true;
  }

  void close_connection() { std::lock_guard<std::mutex> lock(mutex_); if (fd_ >= 0) { close(fd_); fd_ = -1; } }
  bool send_command(const std::vector<uint8_t> &command)
  {
    if (command.empty()) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0) return false;
    size_t total = 0;
    while (total < command.size()) {
      ssize_t written = -1;
      if (transport_ == "tcp") {
        written = ::send(
          fd_, command.data() + total, command.size() - total, MSG_NOSIGNAL);
      } else {
        written = ::write(fd_, command.data() + total, command.size() - total);
      }
      if (written > 0) {
        total += static_cast<size_t>(written);
        continue;
      }
      if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        pollfd descriptor{fd_, POLLOUT, 0};
        if (::poll(&descriptor, 1, 500) > 0) continue;
      }
      RCLCPP_WARN(
        get_logger(), "Sensor TX failed: [%s], wrote %zu/%zu bytes: %s",
        hex_string(command).c_str(), total, command.size(), std::strerror(errno));
      return false;
    }
    if (transport_ == "usb") tcdrain(fd_);
    RCLCPP_INFO(get_logger(), "Sensor TX HEX: [%s]", hex_string(command).c_str());
    return true;
  }
  std::vector<uint8_t> command(uint8_t code) const { return {0xAA, 0x55, code, 0x0D, 0x0A}; }
  std::vector<uint8_t> zero_command() const { return command(0x30); }
  std::vector<uint8_t> debug_exit_command() const { return command(0x31); }
  std::vector<uint8_t> acquisition_stop_command() const { return command(0x01); }
  std::vector<uint8_t> unit_command() const { return command(0x35); }
  bool start_stream()
  {
    // 设备需要在协议命令之间留出处理时间，不能把三条命令连成一次突发发送。
    if (!send_command(command(0x32))) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(command_delay_ms_));
    if (zero_before_request_) {
      if (!send_command(zero_command())) return false;
      // 清零会返回多行 zero[...] 文本，等待设备完成校准后再发送 02。
      std::this_thread::sleep_for(std::chrono::milliseconds(zero_wait_ms_));
    }
    streaming_ = send_command(command(0x02));
    return streaming_;
  }
  bool stop_stream()
  {
    streaming_ = false;
    const bool acquisition_stopped = send_command(acquisition_stop_command());
    std::this_thread::sleep_for(std::chrono::milliseconds(stop_exit_delay_ms_));
    const bool debug_stopped = send_command(debug_exit_command());
    return acquisition_stopped && debug_stopped;
  }
  void request_once() { if (streaming_ && zero_before_request_) send_command(zero_command()); if (streaming_) send_command(command(0x02)); }
  bool set_baud() { if (baud_code_ < 1 || baud_code_ > 3) return false; return send_command({0xAA, 0x55, 0x04, static_cast<uint8_t>(baud_code_), 0x0D, 0x0A}); }

  void write_hex(const std::shared_ptr<eli_cs_robot_serial::srv::WriteSerial::Request> request, const std::shared_ptr<eli_cs_robot_serial::srv::WriteSerial::Response> response)
  { const auto bytes = parse_hex(request->data); response->success = !bytes.empty() && send_command(bytes); response->message = response->success ? "Written " + std::to_string(bytes.size()) + " bytes" : "Invalid HEX or connection unavailable"; }

  void start_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  { response->success = start_stream(); response->message = response->success ? "Sensor streaming started" : "Start failed"; }

  void stop_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  { response->success = stop_stream(); response->message = response->success ? "Sensor streaming stopped" : "Stop failed"; }

  void zero_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  { response->success = send_command(zero_command()); response->message = response->success ? "Zero command sent" : "Zero failed"; }

  void unit_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const bool was_streaming = streaming_;
    streaming_ = false;
    const bool stopped = send_command(acquisition_stop_command());
    std::this_thread::sleep_for(std::chrono::milliseconds(unit_switch_delay_ms_));
    const bool toggled = stopped && send_command(unit_command());
    std::this_thread::sleep_for(std::chrono::milliseconds(unit_switch_delay_ms_));
    const bool zeroed = toggled && send_command(zero_command());
    std::this_thread::sleep_for(std::chrono::milliseconds(zero_wait_ms_));
    bool restarted = true;
    if (was_streaming) {
      restarted = zeroed && send_command(command(0x02));
      streaming_ = restarted;
    }
    response->success = stopped && toggled && zeroed && restarted;
    response->message = response->success ?
      "Unit toggled and acquisition restarted" : "Unit toggle sequence failed";
    if (response->success) unit_ = unit_ == "N" ? "kg" : "N";
  }

  void baud_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  { response->success = set_baud(); response->message = response->success ? "Baud command sent; update baudrate after device switches" : "Baud command failed"; }

  void publish_frame(const std::vector<uint8_t> &data)
  {
    if (data.empty()) return;
    std_msgs::msg::UInt8MultiArray raw;
    raw.data = data;
    raw_pub_->publish(raw);
    std_msgs::msg::String hex;
    hex.data = hex_string(data);
    hex_pub_->publish(hex);
    if (publish_wrench_) parse_wrench(data);
  }

  void publish_raw_chunk(const uint8_t *data, size_t size)
  {
    if (size == 0) return;
    std::vector<uint8_t> bytes(data, data + size);
    std_msgs::msg::UInt8MultiArray raw;
    raw.data = bytes;
    raw_rx_pub_->publish(raw);
    std_msgs::msg::String hex;
    hex.data = hex_string(bytes);
    raw_rx_hex_pub_->publish(hex);
  }

  static float read_float32_be(const uint8_t *data)
  {
    float value = 0.0F;
    const std::array<uint8_t, 4> host_bytes{data[3], data[2], data[1], data[0]};
    std::memcpy(&value, host_bytes.data(), sizeof(value));
    return value;
  }

  void parse_wrench(const std::vector<uint8_t> &data)
  {
    if (data_format_ != "float32_be" || data_offset_ < 0 ||
      data.size() < static_cast<size_t>(data_offset_ + 24) || !wrench_pub_) return;
    std::array<float, 6> values{};
    for (size_t i = 0; i < values.size(); ++i) {
      values[i] = read_float32_be(data.data() + data_offset_ + i * 4);
    }
    std::array<double, 6> wrench_values{};
    for (size_t i = 0; i < values.size(); ++i) wrench_values[i] = values[i];
    publish_wrench_values(wrench_values);
  }

  void publish_wrench_values(const std::array<double, 6> &values)
  {
    // unit 表示设备当前输出单位。传感器已经输出 N/Nm 时不再重复换算。
    const double scale = unit_ == "kg" ? 9.80665 : 1.0;
    geometry_msgs::msg::WrenchStamped message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    message.wrench.force.x = values[0] * scale;
    message.wrench.force.y = values[1] * scale;
    message.wrench.force.z = values[2] * scale;
    message.wrench.torque.x = values[3] * scale;
    message.wrench.torque.y = values[4] * scale;
    message.wrench.torque.z = values[5] * scale;
    wrench_pub_->publish(message);
  }

  void parse_ascii_line(const std::string &line)
  {
    const auto marker = line.find("channels:");
    if (marker == std::string::npos || !wrench_pub_) return;
    std::array<double, 6> values{};
    std::stringstream input(line.substr(marker + 9));
    std::string token;
    size_t index = 0;
    while (std::getline(input, token, ',') && index < values.size()) {
      try {
        const auto first = token.find_first_not_of(" \t");
        const auto last = token.find_last_not_of(" \t\r");
        if (first == std::string::npos) return;
        values[index++] = std::stod(token.substr(first, last - first + 1));
      } catch (const std::exception &) {
        return;
      }
    }
    if (index == values.size()) publish_wrench_values(values);
  }

  void extract_ascii_lines(const uint8_t *data, size_t size)
  {
    ascii_buffer_.append(reinterpret_cast<const char *>(data), size);
    size_t newline = 0;
    while ((newline = ascii_buffer_.find('\n')) != std::string::npos) {
      parse_ascii_line(ascii_buffer_.substr(0, newline));
      ascii_buffer_.erase(0, newline + 1);
    }
    if (ascii_buffer_.size() > 4096) ascii_buffer_.erase(0, ascii_buffer_.size() - 4096);
  }

  void extract_frames()
  {
    const size_t frame_length = static_cast<size_t>(frame_size_);
    if (frame_length < 29) return;
    static constexpr std::array<uint8_t, 2> frame_header{0xAA, 0x55};
    while (true) {
      const auto start = std::search(frame_buffer_.begin(), frame_buffer_.end(),
        frame_header.begin(), frame_header.end());
      if (start == frame_buffer_.end()) {
        if (frame_buffer_.size() > 1) frame_buffer_.erase(frame_buffer_.begin(), frame_buffer_.end() - 1);
        return;
      }
      if (start != frame_buffer_.begin()) frame_buffer_.erase(frame_buffer_.begin(), start);
      if (frame_buffer_.size() < frame_length) return;
      if (frame_buffer_[2] != 0x02 && frame_buffer_[2] != 0x03) {
        frame_buffer_.erase(frame_buffer_.begin());
        continue;
      }
      if (frame_buffer_[frame_length - 2] != 0x0D ||
        frame_buffer_[frame_length - 1] != 0x0A)
      {
        frame_buffer_.erase(frame_buffer_.begin());
        continue;
      }
      publish_frame(std::vector<uint8_t>(frame_buffer_.begin(), frame_buffer_.begin() + frame_length));
      frame_buffer_.erase(frame_buffer_.begin(), frame_buffer_.begin() + frame_length);
    }
  }

  void read_loop()
  {
    std::array<uint8_t, 2048> buffer{};
    while (running_ && rclcpp::ok()) {
      bool connected = false; ssize_t count = 0;
      { std::lock_guard<std::mutex> lock(mutex_); connected = fd_ >= 0; if (connected) count = read(fd_, buffer.data(), buffer.size()); }
      if (count > 0) {
        publish_raw_chunk(buffer.data(), static_cast<size_t>(count));
        extract_ascii_lines(buffer.data(), static_cast<size_t>(count));
        frame_buffer_.insert(frame_buffer_.end(), buffer.begin(), buffer.begin() + count);
        extract_frames();
      } else if (!connected ||
        (transport_ == "tcp" && count == 0) ||
        (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
      {
        if (connected) {
          RCLCPP_WARN(get_logger(), "Sensor connection lost; reconnecting");
        }
        streaming_ = false;
        frame_buffer_.clear();
        ascii_buffer_.clear();
        close_connection();
        if (running_ && open_connection()) {
          RCLCPP_INFO(get_logger(), "Sensor connection opened");
          if (auto_start_) start_stream();
        } else if (running_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Cannot connect to sensor endpoint %s:%d",
            transport_ == "tcp" ? host_.c_str() : device_.c_str(),
            transport_ == "tcp" ? port_ : baudrate_);
          std::this_thread::sleep_for(
            std::chrono::milliseconds(std::max(100, reconnect_period_ms_)));
        }
      }
      else std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  std::string transport_, device_, host_, parity_, frame_id_, data_format_, unit_;
  int port_, reconnect_period_ms_, baudrate_, data_bits_, stop_bits_, data_offset_,
    frame_size_, baud_code_;
  int command_delay_ms_, zero_wait_ms_, unit_switch_delay_ms_, stop_exit_delay_ms_;
  double request_period_;
  bool tcp_keepalive_, tcp_no_delay_, publish_wrench_, auto_start_, zero_before_request_;
  int fd_{-1}; std::atomic<bool> running_{true}, streaming_{false}, shutdown_started_{false}; std::mutex mutex_; std::thread reader_; rclcpp::TimerBase::SharedPtr timer_;
  std::vector<uint8_t> frame_buffer_;
  std::string ascii_buffer_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_pub_, raw_rx_pub_; rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hex_pub_, raw_rx_hex_pub_; rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Service<eli_cs_robot_serial::srv::WriteSerial>::SharedPtr write_srv_; rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, stop_srv_, zero_srv_, unit_srv_, baud_srv_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorNode>();
  rclcpp::spin(node);
  node->shutdown_sensor();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
