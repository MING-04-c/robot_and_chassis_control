// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include <Elite/EliteDriver.hpp>
#include <Elite/Log.hpp>

#include <atomic>
#include <boost/asio.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <iostream>
#include <thread>


using namespace ELITE;
using namespace std::chrono;
namespace po = boost::program_options;

// This is a flag that indicates whether to run or not
// When receive Ctrl+C or kill signal, the flag will be set false.
static std::atomic<bool> s_is_freedrive_running;

void freeDriveLoop(const std::string& robot_ip, const std::string& local_ip, bool headless_mode) {
    EliteDriverConfig config;
    config.robot_ip = robot_ip;
    config.script_file_path = "external_control.script";
    config.local_ip = local_ip;
    config.headless_mode = headless_mode;
    auto driver = std::make_unique<EliteDriver>(config);

    ELITE_LOG_INFO("Wait robot connect");
    while (!driver->isRobotConnected() && s_is_freedrive_running) {
        std::this_thread::sleep_for(4ms);
    }
    ELITE_LOG_INFO("Robot connected");

    ELITE_LOG_INFO("Start freedrive mode");
    driver->writeFreedrive(FreedriveAction::FREEDRIVE_START, 100);
    while (s_is_freedrive_running) {
        driver->writeFreedrive(FreedriveAction::FREEDRIVE_NOOP, 100);
        std::this_thread::sleep_for(10ms);
    }
    ELITE_LOG_INFO("End freedrive mode");
    driver->writeFreedrive(FreedriveAction::FREEDRIVE_END, 100);

    driver->stopControl();
}

int main(int argc, char* argv[]) {
    std::string robot_ip = "";
    std::string local_ip = "";
    bool headless_mode = false;

    // Parser param
    po::options_description desc(
        "Usage:\n"
        "\t./freedrive_example <--robot-ip=ip> [--local-ip=\"\"] [--use-headless-mode=true]\n"
        "Parameters:");
    desc.add_options()
        ("help,h", "Print help message")
        ("robot-ip", po::value<std::string>(&robot_ip)->required(),
            "\tRequired. IP address of the robot.")
        ("use-headless-mode", po::value<bool>(&headless_mode)->required()->implicit_value(true), 
            "\tRequired. Use headless mode.")
        ("local-ip", po::value<std::string>(&local_ip)->default_value(""), 
            "\tOptional. IP address of the local network interface.");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }
        po::notify(vm);
    } catch (const po::error& e) {
        std::cerr << "Argument error: " << e.what() << "\n\n";
        std::cerr << desc << "\n";
        return 1;
    }

    s_is_freedrive_running = true;
    // Freediver is running in another thread
    std::thread freedrive_thread([&](std::string r_ip, std::string l_ip, bool h_mode) { freeDriveLoop(r_ip, l_ip, h_mode); },
                                 robot_ip, local_ip, headless_mode);

    // Capture SIGINT (Ctrl+C) and SIGTERM (kill)
    boost::asio::io_context io_context;
    boost::asio::signal_set signal_set(io_context, SIGINT, SIGTERM);
    signal_set.async_wait([&](const boost::system::error_code& ec, int signal_number) {
        if (!ec) {
            ELITE_LOG_INFO("Received exit signal, exiting...");
            s_is_freedrive_running = false;
            io_context.stop();
        }
    });

    // Boost asio task run
    while (s_is_freedrive_running) {
        if (io_context.stopped()) {
            io_context.restart();
        }
        io_context.run();
    }
    // Wait freedrive thread finish
    freedrive_thread.join();

    return 0;
}
