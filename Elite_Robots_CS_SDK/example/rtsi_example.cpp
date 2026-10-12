// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include <Elite/Log.hpp>
#include <Elite/RtsiIOInterface.hpp>

#include <boost/program_options.hpp>
#include <iostream>
#include <memory>
#include <chrono>

using namespace ELITE;
namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    // Parse the ip arguments if given
    std::string robot_ip;

    // Parser param
    po::options_description desc(
        "Usage:\n"
        "\t./rtsi_example <--robot-ip=ip>\n"
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

    std::unique_ptr<RtsiIOInterface> io_interface = std::make_unique<RtsiIOInterface>("output_recipe.txt", "input_recipe.txt", 250);

    if (!io_interface->connect(robot_ip)) {
        ELITE_LOG_FATAL("Couldn't connect RTSI server");
        return 1;
    }

    VersionInfo version = io_interface->getControllerVersion();
    ELITE_LOG_INFO("Controller is: %s", version.toString().c_str());

    if ((io_interface->getDigitalOutputBits() & 0x00000001)) {
        auto start_set_false = std::chrono::high_resolution_clock::now();
        io_interface->setStandardDigital(0, false);
        while (io_interface->getDigitalOutputBits() | 0x00000000) {
            ;
        }
        auto finish_set_false = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_set_false = finish_set_false - start_set_false;
        ELITE_LOG_INFO("Setting low level cost time: %d", elapsed_set_false.count());
    }

    auto start_set_true = std::chrono::high_resolution_clock::now();
    io_interface->setStandardDigital(0, true);

    while (!(io_interface->getDigitalOutputBits() & 0x00000001)) {
        ;
    }
    auto finish_set_true = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed_set_true = finish_set_true - start_set_true;
    ELITE_LOG_INFO("Setting high level cost time: %d", elapsed_set_true.count());

    io_interface->disconnect();

    return 0;
}
