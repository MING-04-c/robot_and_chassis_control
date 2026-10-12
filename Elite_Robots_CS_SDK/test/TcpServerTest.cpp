#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <chrono>
#include <cstring>
#include <atomic>
#include <functional>
#include "Common/TcpServer.hpp"
#include "boost/asio.hpp"
#include <iostream>

using namespace std::chrono;
using namespace ELITE;

#define SERVER_TEST_PORT (50001)

static bool waitUntil(const std::function<bool()>& predicate, std::chrono::milliseconds timeout,
                      std::chrono::milliseconds poll = std::chrono::milliseconds(5)) {
    auto start = std::chrono::steady_clock::now();
    while (!predicate()) {
        if (std::chrono::steady_clock::now() - start >= timeout) {
            return false;
        }
        std::this_thread::sleep_for(poll);
    }
    return true;
}

class TcpClient
{
public:
    boost::asio::io_context io_context;
    std::unique_ptr<boost::asio::ip::tcp::socket> socket_ptr;
    std::unique_ptr<boost::asio::ip::tcp::resolver> resolver_ptr;
    TcpClient() = default;
    
    TcpClient(const std::string& ip, int port) {
        connect(ip, port);
    }

    ~TcpClient() = default;

    void connect(const std::string& ip, int port) {
        try {
            socket_ptr.reset(new boost::asio::ip::tcp::socket(io_context));
            resolver_ptr.reset(new boost::asio::ip::tcp::resolver(io_context));
            socket_ptr->open(boost::asio::ip::tcp::v4());
            boost::asio::ip::tcp::no_delay no_delay_option(true);
            socket_ptr->set_option(no_delay_option);
            boost::asio::socket_base::reuse_address sol_reuse_option(true);
            socket_ptr->set_option(sol_reuse_option);
#if defined(__linux) || defined(linux) || defined(__linux__)
            boost::asio::detail::socket_option::boolean<IPPROTO_TCP, TCP_QUICKACK> quickack(true);
            socket_ptr->set_option(quickack);
#endif
            boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::make_address(ip), port);
            socket_ptr->async_connect(endpoint, [&](const boost::system::error_code& error) {
                if (error) {
                    throw boost::system::system_error(error);
                }
            });
            io_context.run();
        
        } catch(const boost::system::system_error &error) {
            throw error;
        }
    }
    
};



TEST(TCP_SERVER, TCP_SERVER_TEST) {
    auto tcp_resource = std::make_shared<TcpServer::StaticResource>();
    std::shared_ptr<TcpServer> server = std::make_shared<TcpServer>(SERVER_TEST_PORT, 4, tcp_resource);
    server->startListen();
    // Wait listen
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    TcpClient client("127.0.0.1", SERVER_TEST_PORT);
    // Wait connection
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    EXPECT_TRUE(server->isClientConnected());

    int send_data = 12345;
    std::atomic<bool> receive_flag{false};
    std::atomic<bool> recv_ok{false};
    std::atomic<int> recv_value{0};
    server->setReceiveCallback([&](const uint8_t data[], int nb) {
        if (nb != 4) {
            recv_ok.store(false, std::memory_order_release);
            receive_flag.store(true, std::memory_order_release);
            return;
        }
        int parsed_value = 0;
        std::memcpy(&parsed_value, data, sizeof(parsed_value));
        recv_ok.store(parsed_value == send_data, std::memory_order_release);
        recv_value.store(parsed_value, std::memory_order_release);
        receive_flag.store(true, std::memory_order_release);
    });

    client.socket_ptr->send(boost::asio::buffer(&send_data, sizeof(send_data)));
    // Wait send
    EXPECT_TRUE(waitUntil([&]() { return receive_flag.load(std::memory_order_acquire); }, std::chrono::milliseconds(200)));
    EXPECT_TRUE(recv_ok.load(std::memory_order_acquire));
    EXPECT_EQ(recv_value.load(std::memory_order_acquire), send_data);
    server->unsetReceiveCallback();
    tcp_resource->shutdown();
}

TEST(TCP_SERVER, TCP_SERVER_MULIT_CONNECT) {
    auto tcp_resource = std::make_shared<TcpServer::StaticResource>();
    const std::string client_send_string = "client_send_string\n";
    std::shared_ptr<TcpServer> server = std::make_shared<TcpServer>(SERVER_TEST_PORT, client_send_string.length(), tcp_resource);
    server->startListen();
    // Wait listen
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    TcpClient client1;
    TcpClient client2;

    client1.connect("127.0.0.1", SERVER_TEST_PORT);
    // Wait connected
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    EXPECT_TRUE(server->isClientConnected());
    std::atomic<int> recv_count{0};
    std::atomic<bool> recv_ok{true};
    server->setReceiveCallback([&](const uint8_t data[], int nb) {
        if (nb != static_cast<int>(client_send_string.length())) {
            recv_ok.store(false, std::memory_order_release);
            return;
        }
        std::string str(reinterpret_cast<const char*>(data), static_cast<size_t>(nb));
        if (str != client_send_string) {
            recv_ok.store(false, std::memory_order_release);
            return;
        }
        recv_count.fetch_add(1, std::memory_order_release);
    });

    EXPECT_EQ(client1.socket_ptr->write_some(boost::asio::buffer(client_send_string)), client_send_string.length());
    // Wait for recv
    EXPECT_TRUE(waitUntil([&]() { return recv_count.load(std::memory_order_acquire) >= 1; }, std::chrono::milliseconds(200)));
    EXPECT_TRUE(recv_ok.load(std::memory_order_acquire));
    
    // New connection
    client2.connect("127.0.0.1", SERVER_TEST_PORT);

    // Wait connected
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_TRUE(server->isClientConnected());

    EXPECT_EQ(client2.socket_ptr->write_some(boost::asio::buffer(client_send_string)), client_send_string.length());
    // Wait for recv
    EXPECT_TRUE(waitUntil([&]() { return recv_count.load(std::memory_order_acquire) >= 2; }, std::chrono::milliseconds(200)));
    EXPECT_TRUE(recv_ok.load(std::memory_order_acquire));

    client1.socket_ptr->close();
    client2.socket_ptr->close();

    // Wait close arrive
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    EXPECT_FALSE(server->isClientConnected());

    // Wait close
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // After client close, reconnect
    client1.connect("127.0.0.1", SERVER_TEST_PORT);
    // Wait connected
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    ASSERT_EQ(client1.socket_ptr->write_some(boost::asio::buffer(client_send_string)), client_send_string.length());
    // Wait for recv
    EXPECT_TRUE(waitUntil([&]() { return recv_count.load(std::memory_order_acquire) >= 3; }, std::chrono::milliseconds(200)));
    EXPECT_TRUE(recv_ok.load(std::memory_order_acquire));

    server->unsetReceiveCallback();
    tcp_resource->shutdown();
}

TEST(TCP_SERVER, TCP_MULIT_SERVERS) {
    auto tcp_resource = std::make_shared<TcpServer::StaticResource>();
    const std::string client_send_string = "client_send_string\n";   

    std::vector<std::shared_ptr<TcpServer>> servers;
    for (size_t i = 0; i < 5; i++) {
        servers.push_back(std::make_shared<TcpServer>(SERVER_TEST_PORT + i, client_send_string.length(), tcp_resource));
        servers[i]->startListen();
    }
    // Wait listen
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::vector<std::unique_ptr<TcpClient>> clients;
    for (size_t i = 0; i < 5; i++) {
        clients.push_back(std::make_unique<TcpClient>());
        clients[i]->connect("127.0.0.1", SERVER_TEST_PORT + i);
    }
    // Wait connected
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::atomic<int> recv_count{0};
    std::atomic<bool> recv_ok{true};
    for (size_t i = 0; i < 5; i++) {
        ASSERT_TRUE(servers[i]->isClientConnected());
        servers[i]->setReceiveCallback([&](const uint8_t data[], int nb) {
            if (nb != static_cast<int>(client_send_string.length())) {
                recv_ok.store(false, std::memory_order_release);
                return;
            }

            std::string str(reinterpret_cast<const char*>(data), static_cast<size_t>(nb));
            if (str != client_send_string) {
                recv_ok.store(false, std::memory_order_release);
                return;
            }
            recv_count.fetch_add(1, std::memory_order_release);
        });
    }

    for (size_t i = 0; i < 5; i++) {
        ASSERT_EQ(clients[i]->socket_ptr->write_some(boost::asio::buffer(client_send_string)), client_send_string.length());
    }
    // Wait for recv
    EXPECT_TRUE(waitUntil([&]() { return recv_count.load(std::memory_order_acquire) >= 5; }, std::chrono::milliseconds(500)));
    EXPECT_TRUE(recv_ok.load(std::memory_order_acquire));
    
    for (size_t i = 0; i < 5; i++) {
        clients[i]->socket_ptr->close();
        servers[i]->unsetReceiveCallback();
    }
    tcp_resource->shutdown();
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}