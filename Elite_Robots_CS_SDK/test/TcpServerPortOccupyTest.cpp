#include <gtest/gtest.h>
#include <atomic>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <functional>
#include <chrono>

#include "Common/TcpServer.hpp"
#include "EliteException.hpp"
#include "boost/asio.hpp"

using namespace ELITE;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
namespace {

// Ask the OS for a free IPv4 port by binding to port 0, read back the chosen
// port, then immediately close the socket before returning.  The port is free
// the moment this function returns – a small window exists, but it is
// sufficient for deterministic single-process tests.
int pickFreePort() {
    boost::asio::io_context ioc;
    boost::asio::ip::tcp::acceptor a(ioc);
    a.open(boost::asio::ip::tcp::v4());
    a.set_option(boost::asio::socket_base::reuse_address(true));
    a.bind(boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 0));
    int port = static_cast<int>(a.local_endpoint().port());
    a.close();
    return port;
}

// Synchronous TCP client – plain blocking connect for test simplicity.
class TcpClient {
   public:
    boost::asio::io_context io_context;
    std::unique_ptr<boost::asio::ip::tcp::socket> socket_ptr;

    void connect(const std::string& ip, int port) {
        socket_ptr = std::make_unique<boost::asio::ip::tcp::socket>(io_context);
        socket_ptr->open(boost::asio::ip::tcp::v4());
        socket_ptr->set_option(boost::asio::ip::tcp::no_delay(true));
        socket_ptr->set_option(boost::asio::socket_base::reuse_address(true));
        socket_ptr->connect(
            boost::asio::ip::tcp::endpoint(boost::asio::ip::make_address(ip), port));
    }
};

// Poll predicate until true or timeout expires.
bool waitUntil(const std::function<bool()>& pred,
               std::chrono::milliseconds timeout,
               std::chrono::milliseconds poll = std::chrono::milliseconds(5)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!pred()) {
        if (std::chrono::steady_clock::now() >= deadline) return false;
        std::this_thread::sleep_for(poll);
    }
    return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// Correctness tests
// ---------------------------------------------------------------------------

// After the occupying acceptor closes, TcpServer must be able to bind the same
// port via its internal retry mechanism.
TEST(TCP_SERVER_PORT_OCCUPY, RebindAfterRelease) {
    const int port = pickFreePort();

    boost::asio::io_context blocker_ctx;
    boost::asio::ip::tcp::acceptor blocker(
        blocker_ctx,
        boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port), true);

    auto tcp_resource = std::make_shared<TcpServer::StaticResource>();

    // Port is occupied – first attempt must fail.
    EXPECT_THROW((void)std::make_shared<TcpServer>(port, 4, tcp_resource), EliteException);

    // Release the blocker.
    boost::system::error_code ec;
    blocker.cancel(ec);
    blocker.close(ec);

    // TcpServer must now succeed (retry window covers the release gap).
    std::shared_ptr<TcpServer> server;
    ASSERT_NO_THROW(server = std::make_shared<TcpServer>(port, 4, tcp_resource));
    ASSERT_TRUE(server != nullptr);

    server->startListen();

    TcpClient client;
    EXPECT_NO_THROW(client.connect("127.0.0.1", port));
    EXPECT_TRUE(waitUntil([&] { return server->isClientConnected(); },
                          std::chrono::milliseconds(500)));

    server->unsetReceiveCallback();
    tcp_resource->shutdown();
}

// If the port is held for the entire retry window, TcpServer must throw.
TEST(TCP_SERVER_PORT_OCCUPY, ThrowWhenPortKeepsOccupied) {
    const int port = pickFreePort();

    boost::asio::io_context blocker_ctx;
    boost::asio::ip::tcp::acceptor blocker(
        blocker_ctx,
        boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port), true);

    auto tcp_resource = std::make_shared<TcpServer::StaticResource>();
    EXPECT_THROW((void)std::make_shared<TcpServer>(port, 4, tcp_resource), EliteException);

    boost::system::error_code ec;
    blocker.cancel(ec);
    blocker.close(ec);
    tcp_resource->shutdown();
}

// ---------------------------------------------------------------------------
// Stress test – repeated construct / destruct cycle
// ---------------------------------------------------------------------------

// Simulates the struct_test.cpp scenario: repeatedly create and destroy a
// TcpServer on the same port without a robot.  All iterations must succeed.
TEST(TCP_SERVER_PORT_OCCUPY, StressRebindCycle) {
    constexpr int ITERATIONS = 50;
    const int port = pickFreePort();

    for (int i = 0; i < ITERATIONS; ++i) {
        auto tcp_resource = std::make_shared<TcpServer::StaticResource>();
        std::shared_ptr<TcpServer> server;
        ASSERT_NO_THROW(server = std::make_shared<TcpServer>(port, 4, tcp_resource))
            << "Iteration " << i;
        ASSERT_TRUE(server != nullptr) << "Iteration " << i;

        server->startListen();

        TcpClient client;
        EXPECT_NO_THROW(client.connect("127.0.0.1", port)) << "Iteration " << i;
        EXPECT_TRUE(waitUntil([&] { return server->isClientConnected(); },
                              std::chrono::milliseconds(500)))
            << "Iteration " << i;

        // Simulate robot disconnect.
        boost::system::error_code ec;
        client.socket_ptr->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
        client.socket_ptr->close(ec);

        server->unsetReceiveCallback();
        server.reset();
        tcp_resource->shutdown();
    }
}

// Concurrently construct TcpServer instances on distinct ports from multiple
// threads to exercise the StaticResource / io_context shared path.
TEST(TCP_SERVER_PORT_OCCUPY, StressConcurrentConstruct) {
    constexpr int THREADS = 4;

    // Pre-allocate one free port per thread before starting any server.
    std::vector<int> ports(THREADS);
    for (int i = 0; i < THREADS; ++i) {
        ports[i] = pickFreePort();
    }

    std::atomic<int> failures{0};
    auto tcp_resource = std::make_shared<TcpServer::StaticResource>();
    std::vector<std::thread> workers;

    for (int i = 0; i < THREADS; ++i) {
        workers.emplace_back([&, i]() {
            try {
                auto server = std::make_shared<TcpServer>(ports[i], 4, tcp_resource);
                server->startListen();

                TcpClient client;
                client.connect("127.0.0.1", ports[i]);

                bool connected = waitUntil([&] { return server->isClientConnected(); },
                                            std::chrono::milliseconds(500));
                if (!connected) {
                    ++failures;
                }

                boost::system::error_code ec;
                client.socket_ptr->shutdown(
                    boost::asio::ip::tcp::socket::shutdown_both, ec);
                client.socket_ptr->close(ec);

                server->unsetReceiveCallback();
            } catch (const std::exception& e) {
                ++failures;
            }
        });
    }

    for (auto& t : workers) t.join();
    tcp_resource->shutdown();

    EXPECT_EQ(failures.load(), 0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
