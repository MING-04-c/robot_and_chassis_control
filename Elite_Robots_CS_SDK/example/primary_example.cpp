// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include <Elite/Log.hpp>
#include <Elite/PrimaryPortInterface.hpp>
#include <Elite/RobotConfPackage.hpp>
#include <Elite/RobotException.hpp>

#include <boost/program_options.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono;
namespace po = boost::program_options;

// When the robot encounters an exception, this callback will be called
void robotExceptionCb(ELITE::RobotExceptionSharedPtr ex) {
    if (ex->getType() == ELITE::RobotException::Type::SCRIPT_RUNTIME) {
        auto r_ex = std::static_pointer_cast<ELITE::RobotRuntimeException>(ex);
        ELITE_LOG_INFO("Robot throw exception: %s", r_ex->getMessage().c_str());
    }
}

int main(int argc, const char** argv) {
    // Parse the ip arguments if given
    std::string robot_ip;

    // Parser param
    po::options_description desc(
        "Usage:\n"
        "\t./primary_example <--robot-ip=ip>\n"
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

    auto primary = std::make_unique<ELITE::PrimaryPortInterface>();

    auto kin = std::make_shared<ELITE::KinematicsInfo>();

    primary->connect(robot_ip, 30001);

    primary->registerRobotExceptionCallback(robotExceptionCb);

    primary->getPackage(kin, 200);

    std::string dh_param = "\n\tDH parameter a: ";
    for (auto i : kin->dh_a_) {
        dh_param += std::to_string(i);
        dh_param += '\t';
    }
    dh_param += '\n';

    dh_param += "\n\tDH parameter d: ";
    for (auto i : kin->dh_d_) {
        dh_param += std::to_string(i);
        dh_param += '\t';
    }
    dh_param += '\n';

    dh_param += "\n\tDH parameter alpha: ";
    for (auto i : kin->dh_alpha_) {
        dh_param += std::to_string(i);
        dh_param += '\t';
    }
    dh_param += '\n';

    ELITE_LOG_INFO("%s", dh_param.c_str());

    std::string script = "def hello():\n\ttextmsg(\"hello world\")\nend\n";

    primary->sendScript(script);

    script = "def exFunc():\n\t1abcd\nend\n";
    primary->sendScript(script);

    // Wait robot exception
    std::this_thread::sleep_for(1s);

    primary->disconnect();

    return 0;
}
