cmake_minimum_required(VERSION 3.16)
# 本文件负责选择 ROS1/ROS2 构建方式、查找依赖并生成驱动可执行程序。
project(masc_chassis_ros_driver VERSION 1.0.0 LANGUAGES CXX)

find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(MASC_CHASSIS_DEFAULT_USE_ROS2 ON)
set(MASC_CHASSIS_DEFAULT_USE_ROS1 OFF)
if(DEFINED ENV{ROS_VERSION})
    if("$ENV{ROS_VERSION}" STREQUAL "1")
        set(MASC_CHASSIS_DEFAULT_USE_ROS2 OFF)
        set(MASC_CHASSIS_DEFAULT_USE_ROS1 ON)
    elseif("$ENV{ROS_VERSION}" STREQUAL "2")
        set(MASC_CHASSIS_DEFAULT_USE_ROS2 ON)
        set(MASC_CHASSIS_DEFAULT_USE_ROS1 OFF)
    endif()
endif()

option(MASC_CHASSIS_USE_ROS2 "Build ROS2 ament driver" ${MASC_CHASSIS_DEFAULT_USE_ROS2})
option(MASC_CHASSIS_USE_ROS1 "Build ROS1 catkin driver" ${MASC_CHASSIS_DEFAULT_USE_ROS1})

if(MASC_CHASSIS_USE_ROS1 AND MASC_CHASSIS_USE_ROS2)
    message(FATAL_ERROR "MASC_CHASSIS_USE_ROS1 and MASC_CHASSIS_USE_ROS2 cannot both be ON")
endif()

if(MASC_CHASSIS_USE_ROS2)
    # ROS2 使用 ament_cmake 和 rclcpp。
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(masc_chassis_can_sdk REQUIRED)
    find_package(masc_chassis_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(std_srvs REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)
    find_package(tf2_ros REQUIRED)

    add_compile_definitions(MASC_CHASSIS_USE_ROS2)
elseif(MASC_CHASSIS_USE_ROS1)
    # ROS1 使用 catkin 和 roscpp。
    find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        masc_chassis_can_sdk
        masc_chassis_msgs
        nav_msgs
        sensor_msgs
        roscpp
        std_msgs
        std_srvs
        tf2
        tf2_geometry_msgs
        tf2_ros
    )

    catkin_package(
        CATKIN_DEPENDS
            geometry_msgs
            masc_chassis_can_sdk
            masc_chassis_msgs
            nav_msgs
            sensor_msgs
            roscpp
            std_msgs
            std_srvs
            tf2
            tf2_geometry_msgs
            tf2_ros
    )

    add_compile_definitions(MASC_CHASSIS_USE_ROS1)
else()
    message(FATAL_ERROR "Enable MASC_CHASSIS_USE_ROS1 or MASC_CHASSIS_USE_ROS2")
endif()

set(MASC_CHASSIS_DRIVER_SOURCES
    src/auto_lock_controller.cpp
    src/chassis_ros_driver_node.cpp
    src/command_filter.cpp
    src/feedback_cache.cpp
    src/main.cpp
    src/odom_integrator.cpp
    src/ros_conversions.cpp
)

# 将多个 cpp 编译成一个 ROS 节点可执行文件。
add_executable(masc_chassis_ros_driver_node ${MASC_CHASSIS_DRIVER_SOURCES})

if(MASC_CHASSIS_USE_ROS2)
    add_executable(target_pid_controller_node src/target_pid_controller_node.cpp)
endif()

target_include_directories(masc_chassis_ros_driver_node
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(masc_chassis_ros_driver_node PRIVATE -Wall -Wextra -Wpedantic)

if(MASC_CHASSIS_USE_ROS2)
    ament_target_dependencies(masc_chassis_ros_driver_node
        rclcpp
        geometry_msgs
        masc_chassis_msgs
        nav_msgs
        sensor_msgs
        std_msgs
        std_srvs
        tf2
        tf2_geometry_msgs
        tf2_ros
    )
    target_link_libraries(
        masc_chassis_ros_driver_node
        masc_chassis_can_sdk::masc_chassis_can_sdk
    )
    ament_target_dependencies(target_pid_controller_node
        rclcpp geometry_msgs nav_msgs std_msgs masc_chassis_msgs)
else()
    target_include_directories(masc_chassis_ros_driver_node PRIVATE ${catkin_INCLUDE_DIRS})
    add_dependencies(masc_chassis_ros_driver_node ${catkin_EXPORTED_TARGETS})
    target_link_libraries(masc_chassis_ros_driver_node ${catkin_LIBRARIES} Threads::Threads)
endif()

if(MASC_CHASSIS_USE_ROS2)
    install(TARGETS masc_chassis_ros_driver_node
        DESTINATION lib/${PROJECT_NAME}
    )
    install(TARGETS target_pid_controller_node
        DESTINATION lib/${PROJECT_NAME}
    )
    install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
    )
    install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
    )
    install(FILES README.md COMMUNICATION_IMPLEMENTATION.md SUPERVISOR_PROMPT.md
        DESTINATION share/${PROJECT_NAME}
    )
    if(BUILD_TESTING)
        find_package(ament_cmake_gtest REQUIRED)
        ament_add_gtest(test_auto_lock_controller
            test/test_auto_lock_controller.cpp src/auto_lock_controller.cpp)
        target_include_directories(test_auto_lock_controller PRIVATE include)
        ament_target_dependencies(test_auto_lock_controller
            rclcpp geometry_msgs masc_chassis_msgs nav_msgs sensor_msgs std_msgs
            std_srvs tf2 tf2_geometry_msgs tf2_ros)
        target_link_libraries(test_auto_lock_controller
            masc_chassis_can_sdk::masc_chassis_can_sdk)
    endif()
    ament_package()
else()
    install(TARGETS masc_chassis_ros_driver_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    )
    install(FILES launch/masc_chassis_ros_driver_ros1.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )
endif()
