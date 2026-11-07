#include <array>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <stdexcept>
#include <netinet/tcp.h>
#include <cstring>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "eli_cs_robot_serial/srv/write_serial.hpp"
#include "eli_cs_robot_serial/srv/send_file.hpp"

class TcpNode final : public rclcpp::Node
{
public:
  TcpNode() : Node("tcp_node")
  {
    mode_ = declare_parameter<std::string>("mode", "server");
    host_ = declare_parameter<std::string>("host", "0.0.0.0");
    port_ = declare_parameter<int>("port", 9000);
    publish_text_ = declare_parameter<bool>("publish_text", true);
    line_mode_ = declare_parameter<bool>("line_mode", true);
    append_newline_ = declare_parameter<bool>("append_newline", true);
    reconnect_period_ = declare_parameter<double>("reconnect_period", 2.0);
    connect_timeout_ = declare_parameter<double>("connect_timeout", 1.0);
    if ((mode_ != "client" && mode_ != "server") || port_ < 1 || port_ > 65535 ||
      !std::isfinite(connect_timeout_) || connect_timeout_ <= 0.0 ||
      !std::isfinite(reconnect_period_) || reconnect_period_ <= 0.0) {
      throw std::invalid_argument("Invalid TCP mode, port or timeout");
    }
    connected_pub_ = create_publisher<std_msgs::msg::Bool>(
      declare_parameter<std::string>("connected_topic", "/tcp/connected"),
      rclcpp::QoS(1).transient_local());
    connected_pub_->publish(std_msgs::msg::Bool().set__data(false));
    text_pub_ = create_publisher<std_msgs::msg::String>(declare_parameter<std::string>("text_topic", "/tcp/rx"), 10);
    const auto hex_topic = declare_parameter<std::string>("hex_topic", "");
    if (!hex_topic.empty()) {
      hex_pub_ = create_publisher<std_msgs::msg::String>(hex_topic, rclcpp::QoS(1).transient_local());
    }
    bytes_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(declare_parameter<std::string>("bytes_topic", "/tcp/rx_bytes"), 10);
    write_srv_ = create_service<eli_cs_robot_serial::srv::WriteSerial>(
      declare_parameter<std::string>("write_service", "/tcp/write"),
      [this](const std::shared_ptr<eli_cs_robot_serial::srv::WriteSerial::Request> req,
        std::shared_ptr<eli_cs_robot_serial::srv::WriteSerial::Response> res) { write_service(req, res); });
    send_file_srv_ = create_service<eli_cs_robot_serial::srv::SendFile>(
      declare_parameter<std::string>("send_file_service", "/tcp/send_file"),
      [this](const std::shared_ptr<eli_cs_robot_serial::srv::SendFile::Request> req,
        std::shared_ptr<eli_cs_robot_serial::srv::SendFile::Response> res) { send_file(req, res); });
    tx_string_sub_ = create_subscription<std_msgs::msg::String>(declare_parameter<std::string>("tx_string_topic", "/tcp/tx"), 10,
      [this](const std_msgs::msg::String::SharedPtr m) { send_text(m->data); });
    tx_float_sub_ = create_subscription<std_msgs::msg::Float64>(declare_parameter<std::string>("tx_float_topic", "/tcp/tx_float"), 10,
      [this](const std_msgs::msg::Float64::SharedPtr m) { send_text(std::to_string(m->data)); });
    tx_int_sub_ = create_subscription<std_msgs::msg::Int64>(declare_parameter<std::string>("tx_int_topic", "/tcp/tx_int"), 10,
      [this](const std_msgs::msg::Int64::SharedPtr m) { send_text(std::to_string(m->data)); });
    thread_ = std::thread([this]() { network_loop(); });
    RCLCPP_INFO(get_logger(), "TCP ready: mode=%s host=%s port=%d", mode_.c_str(), host_.c_str(), port_);
  }

  ~TcpNode() override { running_ = false; close_all(); if (thread_.joinable()) thread_.join(); }

private:
  void close_all()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (client_ >= 0) { ::shutdown(client_, SHUT_RDWR); ::close(client_); client_ = -1; }
    if (server_ >= 0) { ::shutdown(server_, SHUT_RDWR); ::close(server_); server_ = -1; }
  }

  // 调用方持有mutex_。短写继续发送剩余部分；失败后断开，不跨连接重发采集命令。
  bool send_locked(const std::string &data)
  {
    if (client_ < 0) return false;
    size_t offset = 0;
    while (offset < data.size() && running_) {
      const auto count = ::send(client_, data.data() + offset, data.size() - offset, MSG_NOSIGNAL);
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0) {
        ::shutdown(client_, SHUT_RDWR);
        return false;
      }
      offset += static_cast<size_t>(count);
    }
    return offset == data.size();
  }

  void send_text(const std::string &value)
  {
    const auto data = append_newline_ && (value.empty() || value.back() != '\n') ? value + "\n" : value;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!send_locked(data)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TCP peer disconnected or send failed");
    }
  }

  void write_service(const std::shared_ptr<eli_cs_robot_serial::srv::WriteSerial::Request> &req,
    const std::shared_ptr<eli_cs_robot_serial::srv::WriteSerial::Response> &res)
  {
    const auto data = append_newline_ && (req->data.empty() || req->data.back() != '\n') ? req->data + "\n" : req->data;
    std::lock_guard<std::mutex> lock(mutex_);
    res->success = send_locked(data);
    res->message = res->success ? "Written " + std::to_string(data.size()) + " bytes" :
      "TCP peer disconnected or send failed; command will not be replayed";
  }

  void publish(const std::vector<uint8_t> &data)
  {
    if (data.empty()) return;
    // 原始字节完整保留；仪器可能发送NUL填充，不能直接放入ROS String。
    std_msgs::msg::UInt8MultiArray bytes;
    bytes.data = data;
    bytes_pub_->publish(bytes);
    if (hex_pub_) {
      // 每次read的完整数据块，不截断、不删除NUL；TCP块不一定等于一个协议帧。
      std::ostringstream hex;
      hex << std::hex << std::setfill('0');
      for (size_t i = 0; i < data.size(); ++i) {
        if (i) hex << ' ';
        hex << std::setw(2) << static_cast<unsigned>(data[i]);
      }
      hex_pub_->publish(std_msgs::msg::String().set__data(hex.str()));
    }
    if (!publish_text_) return;
    if (std::find(data.begin(), data.end(), uint8_t{0}) != data.end()) {
      std::ostringstream preview;
      preview << std::hex << std::setfill('0');
      for (size_t i = 0; i < std::min<size_t>(data.size(), 64); ++i) {
        preview << std::setw(2) << static_cast<unsigned>(data[i]) << ' ';
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TCP RX contains NUL: %zu bytes kept on raw bytes topic; text skipped; HEX(first64)=%s",
        data.size(), preview.str().c_str());
      return;
    }
    std_msgs::msg::String text;
    text.data.assign(data.begin(), data.end());
    text_pub_->publish(text);
  }

  void send_file(const std::shared_ptr<eli_cs_robot_serial::srv::SendFile::Request> &req,
    const std::shared_ptr<eli_cs_robot_serial::srv::SendFile::Response> &res)
  {
    std::ifstream file(req->path, std::ios::binary);
    if (!file) { res->success = false; res->message = "Cannot open file: " + req->path; return; }
    std::array<char, 4096> buffer{}; uint64_t total = 0;
    while (file && running_) {
      file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
      const auto count = file.gcount(); if (count <= 0) break;
      std::lock_guard<std::mutex> lock(mutex_);
      if (client_ < 0) { res->success = false; res->bytes_sent = total; res->message = "TCP peer disconnected"; return; }
      const auto sent = ::send(client_, buffer.data(), static_cast<size_t>(count), MSG_NOSIGNAL);
      if (sent != count) { res->success = false; res->bytes_sent = total; res->message = "TCP file send failed"; return; }
      total += static_cast<uint64_t>(sent);
    }
    res->success = true; res->bytes_sent = total; res->message = "Sent file successfully";
  }

  void receive(int fd)
  {
    std::array<uint8_t, 1024> buffer{}; std::vector<uint8_t> line;
    while (running_ && rclcpp::ok()) {
      const auto count = ::recv(fd, buffer.data(), buffer.size(), 0);
      if (count <= 0) break;
      if (!line_mode_) publish(std::vector<uint8_t>(buffer.begin(), buffer.begin() + count));
      else for (ssize_t i = 0; i < count; ++i) {
        const auto byte = buffer[static_cast<size_t>(i)];
        if (byte == '\n') { publish(line); line.clear(); }
        else if (byte != '\r') line.push_back(byte);
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (client_ == fd) { ::close(client_); client_ = -1; }
  }

  int make_server()
  {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0); if (fd < 0) return -1;
    int reuse = 1; ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{}; address.sin_family = AF_INET; address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(static_cast<uint16_t>(port_));
    if (::bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0 || ::listen(fd, 1) < 0) { ::close(fd); return -1; }
    { std::lock_guard<std::mutex> lock(mutex_); server_ = fd; }
    const int client = ::accept(fd, nullptr, nullptr);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (server_ == fd) { ::close(fd); server_ = -1; }
    }
    return client;
  }

  int make_client()
  {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0); if (fd < 0) return -1;
    // Linux上发送超时也约束阻塞connect，避免未接仪器时Ctrl+C长时间卡住。
    timeval timeout{};
    timeout.tv_sec = static_cast<time_t>(connect_timeout_);
    timeout.tv_usec = static_cast<suseconds_t>((connect_timeout_ - timeout.tv_sec) * 1000000.0);
    if (timeout.tv_sec == 0 && timeout.tv_usec == 0) timeout.tv_usec = 1;
    if (::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0) {
      ::close(fd); return -1;
    }
    int enabled = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &enabled, sizeof(enabled));
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
    sockaddr_in address{}; address.sin_family = AF_INET; address.sin_port = htons(static_cast<uint16_t>(port_));
    if (::inet_pton(AF_INET, host_.c_str(), &address.sin_addr) != 1 ||
      ::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) { ::close(fd); return -1; }
    return fd;
  }

  void network_loop()
  {
    while (running_ && rclcpp::ok()) {
      const int fd = mode_ == "client" ? make_client() : make_server();
      if (fd >= 0) {
        { std::lock_guard<std::mutex> lock(mutex_); client_ = fd; }
        connected_pub_->publish(std_msgs::msg::Bool().set__data(true));
        RCLCPP_INFO(get_logger(), "TCP peer connected: %s:%d", host_.c_str(), port_);
        receive(fd);
        connected_pub_->publish(std_msgs::msg::Bool().set__data(false));
        RCLCPP_WARN(get_logger(), "TCP peer disconnected; waiting to reconnect");
      } else if (running_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "TCP connection unavailable: %s:%d; retrying", host_.c_str(), port_);
      }
      // 可中断退避，避免退出时等待整个重连周期。
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(reconnect_period_);
      while (running_ && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
  }

  std::string mode_, host_; int port_; bool line_mode_, append_newline_, publish_text_; double reconnect_period_, connect_timeout_;
  int client_{-1}, server_{-1}; std::atomic<bool> running_{true}; std::mutex mutex_; std::thread thread_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hex_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr bytes_pub_;
  rclcpp::Service<eli_cs_robot_serial::srv::WriteSerial>::SharedPtr write_srv_;
  rclcpp::Service<eli_cs_robot_serial::srv::SendFile>::SharedPtr send_file_srv_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tx_string_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tx_float_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr tx_int_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<TcpNode>()); rclcpp::shutdown(); return 0;
}
