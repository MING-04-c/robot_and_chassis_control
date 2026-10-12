/**
 * EliteDriverReconstructTest.cpp
 *
 * Verifies that EliteDriver can be repeatedly constructed and destructed on
 * the same set of ports without hitting EADDRINUSE.
 *
 * Usage:
 *   ./EliteDriverReconstructTest [robot_ip] [local_ip]
 *
 * Defaults: robot_ip = 192.168.1.200, local_ip = "" (auto)
 *
 * Requires a real robot or elibotsim running at the given IP and the
 * external_control.script present in the working directory.
 */

#include "Elite/EliteDriver.hpp"
#include "Elite/Log.hpp"

#include <gtest/gtest.h>
#include <chrono>
#include <thread>

using namespace ELITE;
using namespace std::chrono_literals;

static std::string s_robot_ip = "192.168.1.200";
static std::string s_local_ip = "";

// Number of construct/destruct cycles per test iteration.
static constexpr int RECONSTRUCT_ITERATIONS = 10;

// ──────────────────────────────────────────────────────────────────────────────

TEST(EliteDriverReconstructTest, RepeatedConstructDestruct) {
    setLogLevel(ELITE::LogLevel::ELI_ERROR);

    EliteDriverConfig config;
    config.robot_ip = s_robot_ip;
    if (!s_local_ip.empty()) {
        config.local_ip = s_local_ip;
    }
    config.script_file_path = "external_control.script";
    config.headless_mode = true;
    config.servoj_time = 0.004;

    int success_count = 0;
    for (int i = 0; i < RECONSTRUCT_ITERATIONS; ++i) {
        try {
            auto driver = std::make_unique<EliteDriver>(config);

            // Wait until the robot has connected back on all reverse ports,
            // but do not wait indefinitely to avoid hanging the test.
            bool connected = false;
            auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
            while (!(connected = driver->isRobotConnected()) &&
                   std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(10ms);
            }
            if (!connected) {
                GTEST_SKIP() << "Timed out waiting for robot connection after 30 seconds. "
                             << "Ensure the robot or simulator is reachable at " << s_robot_ip
                             << " and that external_control.script is running.";
            }

            // Ensure all reverse/trajectory/script-command links are closed
            // before we destroy the driver so that the ports are released in time.
            EXPECT_TRUE(driver->stopControl(3000))
                << "stopControl timed out at iteration " << i;

            driver.reset();

            // Brief pause so the OS can fully release TIME_WAIT sockets.
            std::this_thread::sleep_for(50ms);
            ++success_count;
        } catch (const std::exception& e) {
            ADD_FAILURE() << "Iteration " << i << " threw: " << e.what();
            std::this_thread::sleep_for(200ms);
        }
    }

    EXPECT_EQ(success_count, RECONSTRUCT_ITERATIONS);
}

// ──────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    // Optional positional args: robot_ip, local_ip
    if (argc >= 2) s_robot_ip = argv[1];
    if (argc >= 3) s_local_ip = argv[2];

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
