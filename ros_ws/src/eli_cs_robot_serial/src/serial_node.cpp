#include <array>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "eli_cs_robot_serial/srv/write_serial.hpp"
#include "eli_cs_robot_serial/srv/send_file.hpp"

class SerialNode final : public rclcpp::Node
{
public:
  SerialNode() : Node("serial_node")
  {
    // 线路参数必须和外部设备一致；默认值为最常见的 115200 8N1、无流控。
    device_ = declare_parameter<std::string>("device", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    data_bits_ = declare_parameter<int>("data_bits", 8);
    stop_bits_ = declare_parameter<int>("stop_bits", 1);
    parity_ = declare_parameter<std::string>("parity", "none");
    flow_control_ = declare_parameter<std::string>("flow_control", "none");
    tx_format_ = declare_parameter<std::string>("tx_format", "text");
    rx_format_ = declare_parameter<std::string>("rx_format", "text");

    // 下面的名称均可由 YAML 修改，便于多个串口节点使用不同的 ROS 接口。
    line_mode_ = declare_parameter<bool>("line_mode", true);
    publish_text_ = declare_parameter<bool>("publish_text", true);
    append_newline_ = declare_parameter<bool>("append_newline", true);
    reconnect_period_ = declare_parameter<double>("reconnect_period", 2.0);
    const auto text_topic = declare_parameter<std::string>("text_topic", "/serial/rx");
    const auto bytes_topic = declare_parameter<std::string>("bytes_topic", "/serial/rx_bytes");
    const auto write_service = declare_parameter<std::string>("write_service", "/serial/write");
    const auto send_file_service = declare_parameter<std::string>("send_file_service", "/serial/send_file");
    const auto tx_string_topic = declare_parameter<std::string>("tx_string_topic", "/serial/tx");
    const auto tx_float_topic = declare_parameter<std::string>("tx_float_topic", "/serial/tx_float");
    const auto tx_int_topic = declare_parameter<std::string>("tx_int_topic", "/serial/tx_int");
    publish_wrench_ = declare_parameter<bool>("publish_wrench", false);
    wrench_topic_ = declare_parameter<std::string>("wrench_topic", "/serial/wrench");
    wrench_frame_id_ = declare_parameter<std::string>("wrench_frame_id", "base_link");

    text_pub_ = create_publisher<std_msgs::msg::String>(text_topic, 10);
    bytes_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(bytes_topic, 10);
    if (publish_wrench_) {
      wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(wrench_topic_, 10);
    }
    tx_string_sub_ = create_subscription<std_msgs::msg::String>(
      tx_string_topic, 10, [this](const std_msgs::msg::String::SharedPtr message) {
        write_text(message->data);
      });
    tx_float_sub_ = create_subscription<std_msgs::msg::Float64>(
      tx_float_topic, 10, [this](const std_msgs::msg::Float64::SharedPtr message) {
        write_text(std::to_string(message->data));
      });
    tx_int_sub_ = create_subscription<std_msgs::msg::Int64>(
      tx_int_topic, 10, [this](const std_msgs::msg::Int64::SharedPtr message) {
        write_text(std::to_string(message->data));
      });
    write_srv_ = create_service<eli_cs_robot_serial::srv::WriteSerial>(
      write_service,
      [this](
        const std::shared_ptr<eli_cs_robot_serial::srv::WriteSerial::Request> request,
        std::shared_ptr<eli_cs_robot_serial::srv::WriteSerial::Response> response)
      {
        write_serial(request, response);
      });
    send_file_srv_ = create_service<eli_cs_robot_serial::srv::SendFile>(send_file_service,
      [this](const std::shared_ptr<eli_cs_robot_serial::srv::SendFile::Request> request,
        std::shared_ptr<eli_cs_robot_serial::srv::SendFile::Response> response) {
        send_file(request, response);
      });
    reader_ = std::thread([this]() { read_loop(); });
    RCLCPP_INFO(get_logger(), "Serial node ready: %s @ %d %d data bits, %d stop bit, parity=%s, flow_control=%s",
      device_.c_str(), baudrate_, data_bits_, stop_bits_, parity_.c_str(), flow_control_.c_str());
  }

  ~SerialNode() override
  {
    running_ = false;
    close_port();
    if (reader_.joinable()) reader_.join();
  }

private:
  static speed_t baud(int value)
  {
    switch (value) {
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      case 460800: return B460800;
      case 921600: return B921600;
      default: return 0;
    }
  }

  bool open_port()
  {
    const auto speed = baud(baudrate_);
    if (speed == 0) {
      RCLCPP_ERROR(get_logger(), "Unsupported baudrate: %d", baudrate_);
      return false;
    }
    if (data_bits_ < 5 || data_bits_ > 8 || stop_bits_ < 1 || stop_bits_ > 2) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Invalid serial format: data_bits must be 5..8 and stop_bits must be 1 or 2");
      return false;
    }
    std::transform(parity_.begin(), parity_.end(), parity_.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::transform(flow_control_.begin(), flow_control_.end(), flow_control_.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (parity_ != "none" && parity_ != "odd" && parity_ != "even") {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Invalid parity '%s'; use none, odd, or even", parity_.c_str());
      return false;
    }
    if (flow_control_ != "none" && flow_control_ != "software" && flow_control_ != "hardware") {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Invalid flow_control '%s'; use none, software, or hardware", flow_control_.c_str());
      return false;
    }
    if (tx_format_ != "text" && tx_format_ != "ascii" && tx_format_ != "hex") {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Invalid tx_format '%s'; use text or hex", tx_format_.c_str());
      return false;
    }
    if (rx_format_ != "text" && rx_format_ != "ascii" && rx_format_ != "hex") {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Invalid rx_format '%s'; use text or hex", rx_format_.c_str());
      return false;
    }
    // 非阻塞读取让线程可以周期性检查退出标志，并在设备拔插后自动重连。
    const int fd = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Cannot open %s: %s", device_.c_str(), std::strerror(errno));
      return false;
    }
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) { ::close(fd); return false; }
    // 清除终端默认的行缓冲、回显和特殊字符处理，再应用 YAML 指定的线路格式。
    cfmakeraw(&tty);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(CSTOPB | PARENB | PARODD | CSIZE);
    switch (data_bits_) {
      case 5: tty.c_cflag |= CS5; break;
      case 6: tty.c_cflag |= CS6; break;
      case 7: tty.c_cflag |= CS7; break;
      case 8: tty.c_cflag |= CS8; break;
    }
    if (stop_bits_ == 2) tty.c_cflag |= CSTOPB;
    if (parity_ == "odd") tty.c_cflag |= PARENB | PARODD;
    else if (parity_ == "even") tty.c_cflag |= PARENB;

    // 软件流控使用 XON/XOFF，硬件流控使用 RTS/CTS，两者默认都关闭。
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag &= ~CRTSCTS;
    if (flow_control_ == "software") tty.c_iflag |= IXON | IXOFF;
    else if (flow_control_ == "hardware") tty.c_cflag |= CRTSCTS;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { ::close(fd); return false; }
    std::lock_guard<std::mutex> lock(mutex_);
    fd_ = fd;
    RCLCPP_INFO(get_logger(), "Opened %s", device_.c_str());
    return true;
  }

  void close_port()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  }

  void write_serial(
    const std::shared_ptr<eli_cs_robot_serial::srv::WriteSerial::Request> request,
    const std::shared_ptr<eli_cs_robot_serial::srv::WriteSerial::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0) { response->success = false; response->message = "Serial port is not open"; return; }
    const auto data = encode_tx(request->data);
    if (data.empty() && !request->data.empty()) {
      response->success = false;
      response->message = "Invalid HEX data; use pairs such as '2E 64 30 3C 3A 3E'";
      return;
    }
    const auto count = ::write(fd_, data.data(), data.size());
    const auto size = data.size();
    response->success = count == static_cast<ssize_t>(size);
    response->message = response->success ? "Written " + std::to_string(size) + " bytes" :
      std::string("Write failed: ") + std::strerror(errno);
  }

  void write_text(const std::string &value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Serial port is not open");
      return;
    }
    const auto data = encode_tx(value);
    if (data.empty() && !value.empty()) {
      RCLCPP_WARN(get_logger(), "Invalid HEX data; use pairs such as '2E 64 30 3C 3A 3E'");
      return;
    }
    const auto count = ::write(fd_, data.data(), data.size());
    if (count != static_cast<ssize_t>(data.size())) {
      RCLCPP_WARN(get_logger(), "Serial write failed: %s", std::strerror(errno));
    }
  }

  void send_file(const std::shared_ptr<eli_cs_robot_serial::srv::SendFile::Request> request,
    const std::shared_ptr<eli_cs_robot_serial::srv::SendFile::Response> response)
  {
    std::ifstream file(request->path, std::ios::binary);
    if (!file) { response->success = false; response->message = "Cannot open file: " + request->path; return; }
    std::array<char, 4096> buffer{}; uint64_t total = 0;
    while (file && running_) {
      file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
      const auto count = file.gcount(); if (count <= 0) break;
      std::lock_guard<std::mutex> lock(mutex_);
      if (fd_ < 0) { response->success = false; response->bytes_sent = total; response->message = "Serial port closed during transfer"; return; }
      const auto written = ::write(fd_, buffer.data(), static_cast<size_t>(count));
      if (written != count) { response->success = false; response->bytes_sent = total; response->message = "Serial file write failed"; return; }
      total += static_cast<uint64_t>(written);
    }
    response->success = true; response->bytes_sent = total; response->message = "Sent file successfully";
  }

  void publish(const std::vector<uint8_t> &data)
  {
    if (data.empty()) return;
    std_msgs::msg::UInt8MultiArray bytes;
    bytes.data = data;
    bytes_pub_->publish(bytes);
    if (publish_text_) {
      std_msgs::msg::String text;
      text.data = format_rx(data);
      text_pub_->publish(text);
    }
    if (publish_wrench_) publish_wrench(data);
  }

  static int hex_digit(char c)
  {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  }

  std::vector<uint8_t> encode_tx(const std::string &value) const
  {
    std::string input = value;
    if (tx_format_ == "text" || tx_format_ == "ascii") {
      if (append_newline_ && (input.empty() || input.back() != '\n')) input.push_back('\n');
      return std::vector<uint8_t>(input.begin(), input.end());
    }

    // HEX 输入允许空格、逗号、分号和 0x 前缀，例如 "0x2E 64 30"。
    std::vector<uint8_t> result;
    int high = -1;
    for (size_t i = 0; i < input.size(); ++i) {
      const char c = input[i];
      if (std::isspace(static_cast<unsigned char>(c)) || c == ',' || c == ';') continue;
      if (c == '0' && i + 1 < input.size() && input[i + 1] == 'x') { ++i; continue; }
      const int digit = hex_digit(c);
      if (digit < 0) return {};
      if (high < 0) high = digit;
      else {
        result.push_back(static_cast<uint8_t>((high << 4) | digit));
        high = -1;
      }
    }
    if (high >= 0) return {};
    if (append_newline_) result.push_back('\n');
    return result;
  }

  std::string format_rx(const std::vector<uint8_t> &data) const
  {
    if (rx_format_ == "text" || rx_format_ == "ascii") {
      return std::string(data.begin(), data.end());
    }
    // /serial/rx 是给人看的显示话题；/serial/rx_bytes 始终保留真实字节。
    static const char digits[] = "0123456789ABCDEF";
    std::string result;
    for (size_t i = 0; i < data.size(); ++i) {
      if (i != 0) result.push_back(' ');
      result.push_back(digits[data[i] >> 4]);
      result.push_back(digits[data[i] & 0x0F]);
    }
    return result;
  }

  void publish_wrench(const std::vector<uint8_t> &data)
  {
    // 仅在启用 publish_wrench 时使用：按 Fx Fy Fz Tx Ty Tz 解析一帧文本。
    // 数值之间允许空格、逗号等常见分隔符。
    const std::string line(data.begin(), data.end());
    static const std::regex number_pattern(
      R"([-+]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][-+]?[0-9]+)?)");
    std::vector<double> values;
    for (std::sregex_iterator it(line.begin(), line.end(), number_pattern), end;
      it != end && values.size() < 6; ++it)
    {
      try { values.push_back(std::stod(it->str())); }
      catch (const std::exception &) { return; }
    }
    if (values.size() != 6) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Cannot parse serial wrench; expected Fx Fy Fz Tx Ty Tz");
      return;
    }
    geometry_msgs::msg::WrenchStamped message;
    message.header.stamp = now();
    message.header.frame_id = wrench_frame_id_;
    message.wrench.force.x = values[0];
    message.wrench.force.y = values[1];
    message.wrench.force.z = values[2];
    message.wrench.torque.x = values[3];
    message.wrench.torque.y = values[4];
    message.wrench.torque.z = values[5];
    wrench_pub_->publish(message);
  }

  void read_loop()
  {
    std::array<uint8_t, 512> buffer{};
    std::vector<uint8_t> line;
    while (running_ && rclcpp::ok()) {
      bool opened = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        opened = fd_ >= 0;
        if (opened) {
          const auto count = ::read(fd_, buffer.data(), buffer.size());
          if (count > 0) {
            // line_mode=false 立即发布数据块；true 则积累到换行符再发布。
            if (!line_mode_) {
              publish(std::vector<uint8_t>(buffer.begin(), buffer.begin() + count));
            } else {
              for (ssize_t i = 0; i < count; ++i) {
                const auto byte = buffer[static_cast<size_t>(i)];
                if (byte == '\n') { publish(line); line.clear(); }
                else if (byte != '\r') line.push_back(byte);
              }
            }
          } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            ::close(fd_); fd_ = -1;
          }
        }
      }
      if (!opened) {
        open_port();
        std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_period_));
      } else std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  std::string device_;
  int baudrate_;
  int data_bits_;
  int stop_bits_;
  std::string parity_;
  std::string flow_control_;
  std::string tx_format_;
  std::string rx_format_;
  bool line_mode_;
  bool publish_text_;
  bool publish_wrench_{false};
  bool append_newline_;
  double reconnect_period_;
  std::string wrench_topic_;
  std::string wrench_frame_id_;
  int fd_{-1};
  std::atomic<bool> running_{true};
  std::mutex mutex_;
  std::thread reader_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr bytes_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Service<eli_cs_robot_serial::srv::WriteSerial>::SharedPtr write_srv_;
  rclcpp::Service<eli_cs_robot_serial::srv::SendFile>::SharedPtr send_file_srv_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tx_string_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tx_float_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr tx_int_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialNode>());
  rclcpp::shutdown();
  return 0;
}
