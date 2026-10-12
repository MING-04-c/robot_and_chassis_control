// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include <Elite/DashboardClient.hpp>
#include <Elite/DataType.hpp>
#include <Elite/Log.hpp>

#include <boost/program_options.hpp>
#include <iostream>
#include <regex>
#include <thread>

using namespace ELITE;
namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    // Parse the ip arguments if given
    std::string robot_ip;

    // Parser param
    po::options_description desc(
        "Usage:\n"
        "\t./dashboard_example <--robot-ip=ip>\n"
        "Note:A task named \"test\" must be saved in the robot before running.\n"
        "Parameters:");
    desc.add_options()
        ("help,h", "Print help message")
        ("robot-ip", po::value<std::string>(&robot_ip)->required(),
            "\tRequired. IP address of the robot.");

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

    // Making the robot ready for the program by:
    // Connect the the robot Dashboard
    std::unique_ptr<DashboardClient> my_dashboard;
    my_dashboard.reset(new DashboardClient());
    if (!my_dashboard->connect(robot_ip)) {
        ELITE_LOG_FATAL("Could not connect to robot");
        return 1;
    } else {
        ELITE_LOG_INFO("Connect to robot");
    }

    if (!my_dashboard->echo()) {
        ELITE_LOG_FATAL("Echo not right response");
        return 1;
    } else {
        ELITE_LOG_INFO("Echo right response");
    }

    if (!my_dashboard->powerOff()) {
        ELITE_LOG_FATAL("Could not send power off");
        return 1;
    } else {
        ELITE_LOG_INFO("Power off");
    }

    my_dashboard->closeSafetyDialog();

    // Power it on
    if (!my_dashboard->powerOn()) {
        ELITE_LOG_FATAL("Could not send Power on command");
        return 1;
    } else {
        ELITE_LOG_INFO("Power on");
    }

    // Release the brakes
    if (!my_dashboard->brakeRelease()) {
        ELITE_LOG_FATAL("Could not send BrakeRelease command");
        return 1;
    } else {
        ELITE_LOG_INFO("Brake release");
    }

    // Load existing task
    const std::string task_file_name_to_be_loaded("test.task");
    if (!my_dashboard->loadTask(task_file_name_to_be_loaded)) {
        ELITE_LOG_FATAL("Could not load  %s", task_file_name_to_be_loaded.c_str());
        return 1;
    }
    std::string task = my_dashboard->getTaskPath();
    if (task != task_file_name_to_be_loaded) {
        ELITE_LOG_FATAL("Not load right task");
        return 1;
    } else {
        ELITE_LOG_INFO("Load task");
    }

    if (my_dashboard->getTaskStatus() != TaskStatus::STOPPED) {
        ELITE_LOG_FATAL("Task not stopped");
        return 1;
    } else {
        ELITE_LOG_INFO("Task stopped");
    }

    if (!my_dashboard->playProgram()) {
        ELITE_LOG_FATAL("Could not play task");
        return 1;
    } else {
        ELITE_LOG_INFO("Play task");
    }

    if (my_dashboard->getTaskStatus() != TaskStatus::PLAYING) {
        ELITE_LOG_FATAL("Task not running");
        return 1;
    } else {
        ELITE_LOG_INFO("Task running");
    }

    if (!my_dashboard->pauseProgram()) {
        ELITE_LOG_FATAL("Could not pause task");
        return 1;
    } else {
        ELITE_LOG_INFO("Pause task");
    }

    if (my_dashboard->getTaskStatus() != TaskStatus::PAUSED) {
        ELITE_LOG_FATAL("Task not pause");
        return 1;
    } else {
        ELITE_LOG_INFO("Task pause");
    }

    if (!my_dashboard->stopProgram()) {
        ELITE_LOG_FATAL("Could not stop task");
        return 1;
    } else {
        ELITE_LOG_INFO("Stop task");
    }

    if (my_dashboard->getTaskStatus() != TaskStatus::STOPPED) {
        ELITE_LOG_FATAL("Task not stop");
        return 1;
    } else {
        ELITE_LOG_INFO("Task stopped");
    }

    if (!my_dashboard->isTaskSaved()) {
        ELITE_LOG_FATAL("Task save status not right");
        return 1;
    } else {
        ELITE_LOG_INFO("Task saved");
    }

    if (!my_dashboard->popup("-s", "Hello Robot")) {
        ELITE_LOG_FATAL("Could not popup message box");
        return 1;
    } else {
        ELITE_LOG_INFO("Popup message box");
    }

    my_dashboard->disconnect();

    return 0;
}