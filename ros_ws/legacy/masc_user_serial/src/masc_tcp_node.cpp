#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <sstream>
#include <cctype>
#include <vector>
#include <iomanip>

class TcpNode final : public rclcpp::Node {
public:
  TcpNode():Node("masc_tcp_node") {
    const auto ip=declare_parameter("listen_ip",std::string("0.0.0.0")); const int port=declare_parameter("listen_port",9000); debug_format_=declare_parameter("debug_format",std::string("auto"));
    rx_=create_publisher<std_msgs::msg::String>("tcp_rx",10);
    str_=create_subscription<std_msgs::msg::String>("tcp_send_string",10,[this](std_msgs::msg::String::ConstSharedPtr m){send_json("string",m->data);});
    flt_=create_subscription<std_msgs::msg::Float32>("tcp_send_float",10,[this](std_msgs::msg::Float32::ConstSharedPtr m){send_json("float",std::to_string(m->data));});
    num_=create_subscription<std_msgs::msg::Int32>("tcp_send_int",10,[this](std_msgs::msg::Int32::ConstSharedPtr m){send_json("int",std::to_string(m->data));});
    debug_=create_subscription<std_msgs::msg::String>("tcp_debug_tx",10,[this](std_msgs::msg::String::ConstSharedPtr m){send_debug(m->data);});
    server_=socket(AF_INET,SOCK_STREAM,0); int one=1; setsockopt(server_,SOL_SOCKET,SO_REUSEADDR,&one,sizeof(one)); sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(port);a.sin_addr.s_addr=inet_addr(ip.c_str());bind(server_,(sockaddr*)&a,sizeof(a));listen(server_,1);RCLCPP_INFO(get_logger(),"TCP server listening %s:%d",ip.c_str(),port);thread_ = std::thread([this]{accept_loop();});
  }
  ~TcpNode(){
    running_=false;
    if(server_>=0){shutdown(server_,SHUT_RDWR);close(server_);server_=-1;}
    {std::lock_guard<std::mutex> g(lock_);if(client_>=0){shutdown(client_,SHUT_RDWR);close(client_);client_=-1;}}
    if(thread_.joinable())thread_.join();
  }
private:
  int server_{-1},client_{-1}; bool running_{true}; std::string debug_format_; std::mutex lock_; std::thread thread_; rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rx_; rclcpp::Subscription<std_msgs::msg::String>::SharedPtr str_,debug_; rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr flt_; rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr num_;
  void send_debug(const std::string& in){std::stringstream ss(in);std::string x;std::vector<unsigned char>b;bool hex=!in.empty();while(ss>>x){if(x.size()!=2||!std::isxdigit(x[0])||!std::isxdigit(x[1])){hex=false;break;}b.push_back(static_cast<unsigned char>(std::stoul(x,nullptr,16)));}if(debug_format_=="ascii")hex=false;if(debug_format_=="hex")hex=true;std::lock_guard<std::mutex>g(lock_);if(client_<0)return;if(hex&&!b.empty())::send(client_,b.data(),b.size(),0);else ::send(client_,in.data(),in.size(),0);}
  void send_json(const std::string&t,const std::string&v){std::string s="{\"type\":\""+t+"\",\"value\":"+(t=="string"?"\""+v+"\"":v)+"}\n";std::lock_guard<std::mutex>g(lock_);if(client_>=0)::send(client_,s.data(),s.size(),0);}
  void accept_loop(){while(running_){int c=accept(server_,nullptr,nullptr);if(c<0){if(!running_)break;continue;}{std::lock_guard<std::mutex>g(lock_);client_=c;}unsigned char b[4096];while(running_){ssize_t n=recv(c,b,sizeof(b),0);if(n<=0)break;bool printable=true;for(ssize_t i=0;i<n;++i)if(b[i]<32||b[i]>126){printable=false;break;}std::string out;if(printable)out.assign(reinterpret_cast<char*>(b),n);else{std::ostringstream hex;hex<<std::uppercase<<std::hex<<std::setfill('0');for(ssize_t i=0;i<n;++i){if(i)hex<<' ';hex<<std::setw(2)<<(unsigned)b[i];}out=hex.str();}std_msgs::msg::String m;m.data=out;rx_->publish(m);RCLCPP_INFO(get_logger(),"TCP RX (%ld bytes): %s",static_cast<long>(n),out.c_str());}shutdown(c,SHUT_RDWR);close(c);std::lock_guard<std::mutex>g(lock_);client_=-1;}}
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<TcpNode>());rclcpp::shutdown();return 0;}
