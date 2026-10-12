#include <gtest/gtest.h>
#include <string>
#include <cstdint>
#include <chrono>

#include "Elite/RtsiIOInterface.hpp"

using namespace ELITE;

static std::string s_robot_ip;


TEST(RtsiIOTest, rw_io) {
    EXPECT_FALSE(s_robot_ip.empty());

    RtsiIOInterface io_interface("output_recipe.txt", "input_recipe.txt", 250);
    EXPECT_TRUE(io_interface.connect(s_robot_ip));

    EXPECT_TRUE(io_interface.setStandardDigital(0, false));
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
    uint32_t io_bits = io_interface.getDigitalOutputBits();
    EXPECT_FALSE(io_bits & 0X01);

    EXPECT_TRUE(io_interface.setStandardDigital(0, true));
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
    io_bits = io_interface.getDigitalOutputBits();
    EXPECT_TRUE(io_bits & 0X01);
}


TEST(RtsiMultipleConnect, connect) {  
    EXPECT_FALSE(s_robot_ip.empty());

    RtsiIOInterface io_interface("output_recipe.txt", "input_recipe.txt", 250);
    EXPECT_TRUE(io_interface.connect(s_robot_ip));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_TRUE(io_interface.connect(s_robot_ip));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_TRUE(io_interface.connect(s_robot_ip));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::thread connect_thread([&](){
        EXPECT_TRUE(io_interface.connect(s_robot_ip));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        EXPECT_TRUE(io_interface.connect(s_robot_ip));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        EXPECT_TRUE(io_interface.connect(s_robot_ip));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    });

    std::thread get_thread([&](){
        for (size_t i = 0; i < 10000; i++) {
            io_interface.getActualJointCurrent();
        }
    });

    connect_thread.join();
    get_thread.join();

    io_interface.disconnect();
}


int main(int argc, char** argv) {
    if (argc < 2 || argv[1] == nullptr) {
        return 1;
    }
    s_robot_ip = argv[1];
    
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
