/*
 * Copyright (C) 2026, MASC.
 * @Description: ROS1/ROS2 API selection for the chassis ROS driver.
 */
#pragma once

// 通过编译宏选择 ROS2 或 ROS1 适配层，使上层驱动业务代码保持基本一致。
#if defined(MASC_CHASSIS_USE_ROS2)
#include "masc_chassis_ros_driver/ros_api_ros2.hpp"
#elif defined(MASC_CHASSIS_USE_ROS1)
#include "masc_chassis_ros_driver/ros_api_ros1.hpp"
#else
#error "Must define MASC_CHASSIS_USE_ROS2 or MASC_CHASSIS_USE_ROS1"
#endif
