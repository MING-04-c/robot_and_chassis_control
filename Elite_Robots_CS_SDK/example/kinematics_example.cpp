// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include <Elite/ClassLoader.hpp>
#include <Elite/KinematicsBase.hpp>
#include <Elite/Log.hpp>
#include <Elite/PrimaryPortInterface.hpp>
#include <Elite/RobotConfPackage.hpp>
#include <Elite/RobotException.hpp>
#include <Elite/RtsiIOInterface.hpp>

#include <boost/program_options.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono;
using namespace ELITE;
namespace po = boost::program_options;

void logVector6d(const std::string& prefix, const vector6d_t& vec) {
    std::string vec_str;
    vec_str += "[";
    for (const auto& val : vec) {
        vec_str += std::to_string(val) + ", ";
    }
    vec_str += "]";
    ELITE_LOG_INFO("%s %s", prefix.c_str(), vec_str.c_str());
}

int main(int argc, const char** argv) {
#if defined(ELITE_COMPILE_KIN_PLUGIN)
    // Parse the ip arguments if given
    std::string robot_ip;

    // Parser param
    po::options_description desc(
        "Usage:\n"
        "\t./kinematics_example <--robot-ip=ip>\n"
        "Parameters:");
    desc.add_options()("help,h", "Print help message")("robot-ip", po::value<std::string>(&robot_ip)->required(),
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

    auto primary = std::make_unique<PrimaryPortInterface>();

    auto kin_info = std::make_shared<KinematicsInfo>();

    // Connect robot 30001 port and get MDH param
    if (!primary->connect(robot_ip, 30001)) {
        ELITE_LOG_FATAL("Connect robot 30001 port fail.");
        return 1;
    }
    if (!primary->getPackage(kin_info, 200)) {
        ELITE_LOG_FATAL("Get robot kinematics info fail.");
        return 1;
    }
    primary->disconnect();
    ELITE_LOG_INFO("Got robot kinematics info.");

    std::unique_ptr<RtsiIOInterface> io_interface = std::make_unique<RtsiIOInterface>("output_recipe.txt", "input_recipe.txt", 250);
    if (!io_interface->connect(robot_ip)) {
        ELITE_LOG_FATAL("Connect robot RTSI port fail.");
        return 1;
    }

    auto current_joint = io_interface->getActualJointPositions();
    ELITE_LOG_INFO("Got robot actual joint positions.");

    auto current_tcp = io_interface->getActualTCPPose();
    ELITE_LOG_INFO("Got robot actual TCP positions.");
#if defined(__linux__)
    ClassLoader loader("../plugin/kinematics/libelite_kdl_kinematics.so");
#elif defined(_WIN32)
    ClassLoader loader("../kin_plugin/KdlKinematicsPlugin/elite_kdl_kinematics.dll");
#else
#error "Unsupported platform"
#endif
    if (!loader.loadLib()) {
        ELITE_LOG_FATAL("Load plugin fail.");
        return 1;
    }

    auto kin_solver = loader.createUniqueInstance<KinematicsBase>("ELITE::KdlKinematicsPlugin");
    if (kin_solver == nullptr) {
        ELITE_LOG_FATAL("Create KinematicsBase fail");
        return 1;
    }
    // Set MDH params
    kin_solver->setMDH(kin_info->dh_alpha_, kin_info->dh_a_, kin_info->dh_d_);

    // Get FK and IK
    vector6d_t fk_pose;
    if (!kin_solver->getPositionFK(current_joint, fk_pose)) {
        ELITE_LOG_FATAL("Get FK fail.");
        return 1;
    }

    vector6d_t ik_joints;
    KinematicsResult ik_result;
    if (!kin_solver->getPositionIK(current_tcp, current_joint, ik_joints, ik_result)) {
        ELITE_LOG_FATAL("Get IK fail.");
        return 1;
    }

    logVector6d("Current TCP Pose:", current_tcp);
    logVector6d("FK Pose:", fk_pose);

    logVector6d("IK Result Joints:", ik_joints);
    logVector6d("Current Joints:", current_joint);
#endif
    return 0;
}
