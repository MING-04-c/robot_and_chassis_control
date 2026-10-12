#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <fstream>
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
    line_mode_ = declare_parameter<bool>("line_mode", true);
    append_newline_ = declare_parameter<bool>("append_newline", true);
    reconnect_period_ = declare_parameter<double>("reconnect_period", 2.0);
    text_pub_ = create_publisher<std_msgs::msg::String>(declare_parameter<std::string>("text_topic", "/tcp/rx"), 10);
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

  void send_text(const std::string &value) {
  // 处理换行符逻辑：如果开启了 append_newline_ 并且（字符串为空 或 最后一个字符不是 '\n'）
  // 则在数据末尾补上 "\n"，否则直接使用原字符串。
  const auto data = append_newline_ && (value.empty() || value.back() != '\n') ? value + "\n" : value;
  
  // 加锁，确保多线程下对 client_ 描述符和网络发送的操作是安全的
  std::lock_guard<std::mutex> lock(mutex_);
  
  // 如果当前没有连接，记录一次警告（为了防刷屏，使用 THROTTLE 限制每 5000ms 也就是 5秒最多打印一次），并退出
  if (client_ < 0) { RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TCP peer is not connected"); return; }
  
  // 调用底层 send 函数发送数据。MSG_NOSIGNAL 非常重要，它防止对端意外断开时系统抛出 SIGPIPE 导致程序直接崩溃。
  // 比较实际发送的字节数是否等于我们要发送的数据长度
  if (::send(client_, data.data(), data.size(), MSG_NOSIGNAL) != static_cast<ssize_t>(data.size())) {
    // 如果不相等（发送失败或未发全），打印警告
    RCLCPP_WARN(get_logger(), "TCP send failed");
  }}

  void write_service(const std::shared_ptr<eli_cs_robot_serial::srv::WriteSerial::Request> &req,
                   const std::shared_ptr<eli_cs_robot_serial::srv::WriteSerial::Response> &res) {
  // 与 send_text 逻辑相同，处理传入请求 (req->data) 的尾部换行符
  const auto data = append_newline_ && (req->data.empty() || req->data.back() != '\n') ? req->data + "\n" : req->data;
  
  std::lock_guard<std::mutex> lock(mutex_); // 加锁
  
  // 如果未连接，直接修改响应对象 (res) 的 success 为 false，并写明原因，结束处理
  if (client_ < 0) { res->success = false; res->message = "TCP peer is not connected"; return; }
  
  // 尝试发送，并保存实际发送出去的字节数给 count
  const auto count = ::send(client_, data.data(), data.size(), MSG_NOSIGNAL);
  
  // 判断服务是否成功：实际发送的字节数 == 预期需要发送的字节数
  res->success = count == static_cast<ssize_t>(data.size());
  
  // 三目运算符设置响应提示信息：成功返回 "Written xxx bytes"，失败返回 "TCP send failed"
  res->message = res->success ? "Written " + std::to_string(data.size()) + " bytes" : "TCP send failed";
  }

  void publish(const std::vector<uint8_t> &data) {
  if (data.empty()) return; // 容错处理，如果数据为空直接返回
  
  // 构造 UInt8MultiArray（字节数组）消息，把接收到的二进制数据直接填入，并通过 bytes_pub_ 发布
  std_msgs::msg::UInt8MultiArray bytes; bytes.data = data; bytes_pub_->publish(bytes);
  
  // 构造 String（文本）消息，利用迭代器将字节数据强制转换为 ASCII 字符串，并通过 text_pub_ 发布
  std_msgs::msg::String text; text.data.assign(data.begin(), data.end()); text_pub_->publish(text);
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
    sockaddr_in address{}; address.sin_family = AF_INET; address.sin_port = htons(static_cast<uint16_t>(port_));
    if (::inet_pton(AF_INET, host_.c_str(), &address.sin_addr) != 1 ||
      ::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) { ::close(fd); return -1; }
    return fd;
  }

  void network_loop()
  {
    while (running_ && rclcpp::ok()) {
      const int fd = mode_ == "client" ? make_client() : make_server();
      if (fd >= 0) { { std::lock_guard<std::mutex> lock(mutex_); client_ = fd; } RCLCPP_INFO(get_logger(), "TCP peer connected"); receive(fd); }
      else if (running_) std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_period_));
    }
  }

  std::string mode_, host_; int port_; bool line_mode_, append_newline_; double reconnect_period_;
  int client_{-1}, server_{-1}; std::atomic<bool> running_{true}; std::mutex mutex_; std::thread thread_;
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
