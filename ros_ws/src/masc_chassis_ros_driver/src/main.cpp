/*
 * Copyright (C) 2026, MASC.
 * @Description: Main entry for the MASC chassis ROS driver.
 */
#include <clocale>
#include <memory>

#include "masc_chassis_ros_driver/chassis_ros_driver_node.hpp"
#include "masc_chassis_ros_driver/ros_api.hpp"

// 驱动进程入口。这里不直接处理 CAN 或 ROS 业务，只负责初始化 ROS、
// 创建底盘节点并把控制权交给 ROS 事件循环。
int main(int argc, char *argv[]) {
    if (std::setlocale(LC_ALL, "") == nullptr) {
        std::setlocale(LC_ALL, "C.UTF-8");
    }

    // RosInit 在 ROS1/ROS2 适配层中分别映射到 ros::init/rclcpp::init。
    masc_chassis_ros_driver::RosInit(argc, argv);
    masc_chassis_ros_driver::RosNodePtr node;

#if defined(MASC_CHASSIS_USE_ROS2)
    // ROS2 需要通过 NodeOptions 构造节点；ROS1 使用默认构造函数。
    node = std::make_shared<masc_chassis_ros_driver::ChassisRosDriverNode>(
        rclcpp::NodeOptions());
#else
    node = std::make_shared<masc_chassis_ros_driver::ChassisRosDriverNode>();
#endif

    // spin 会持续处理 cmd_vel、服务请求和所有定时器回调。
    masc_chassis_ros_driver::RosSpin(node);
    masc_chassis_ros_driver::RosShutdown();
    return 0;
}
