cmake_minimum_required(VERSION 3.8)
project(eli_cs_robot_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/WriteSerial.srv"
  "srv/SetSensorUnit.srv"
  "srv/SendFile.srv"
)

add_executable(serial_node src/serial_node.cpp)
target_compile_features(serial_node PUBLIC cxx_std_17)
ament_target_dependencies(serial_node rclcpp std_msgs geometry_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(serial_node "${cpp_typesupport_target}")

add_executable(tcp_node src/tcp_node.cpp)
target_compile_features(tcp_node PUBLIC cxx_std_17)
ament_target_dependencies(tcp_node rclcpp std_msgs)
target_link_libraries(tcp_node "${cpp_typesupport_target}")

add_executable(detactor_node device/detactor/src/detactor_node.cpp)
target_compile_features(detactor_node PUBLIC cxx_std_17)
ament_target_dependencies(detactor_node rclcpp std_msgs std_srvs)
target_link_libraries(detactor_node "${cpp_typesupport_target}")

add_executable(sensor_node
  device/sensor/src/sensor_node.cpp
  device/sensor/src/unit_calibration.cpp
)
target_compile_features(sensor_node PUBLIC cxx_std_17)
ament_target_dependencies(sensor_node rclcpp std_msgs geometry_msgs std_srvs)
target_link_libraries(sensor_node "${cpp_typesupport_target}")


install(TARGETS serial_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS tcp_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS detactor_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY device/detactor/config/
  DESTINATION share/${PROJECT_NAME}/device/detactor/config
)
install(FILES device/detactor/launch/detactor.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY device/sensor/config/
  DESTINATION share/${PROJECT_NAME}/device/sensor/config
)
install(FILES device/sensor/README.md
  DESTINATION share/${PROJECT_NAME}/device/sensor
)
install(FILES
  device/sensor/launch/sensor_usb.launch.py
  device/sensor/launch/sensor_tcp.launch.py
  device/sensor/launch/sensor_plot.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)
install(PROGRAMS device/sensor/src/sensor_plot.py
  DESTINATION lib/${PROJECT_NAME}
)

# 工控机结果通信及不依赖 ROS 的主机端工具。
install(PROGRAMS
  device/industrial_pc/src/industrial_pc_node.py
  device/industrial_pc/src/industrial_pc_client.py
  device/industrial_pc/src/industrial_pc_hk_transfer.py
  device/industrial_pc/src/industrial_pc_hk_receiver.py
  DESTINATION lib/${PROJECT_NAME}
)
# 旧 robot_task_control 入口导入此模块，避免反向依赖任务包。
install(FILES device/industrial_pc/src/industrial_pc_hk_transfer.py
  DESTINATION ${PYTHON_INSTALL_DIR}
)
install(FILES device/industrial_pc/src/industrial_pc_protocol.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY device/industrial_pc/config/
  DESTINATION share/${PROJECT_NAME}/device/industrial_pc/config
)
install(FILES device/industrial_pc/launch/industrial_pc.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)
install(FILES device/industrial_pc/README.md
  DESTINATION share/${PROJECT_NAME}/device/industrial_pc
)
install(FILES
  device/industrial_pc/src/industrial_pc_client.py
  device/industrial_pc/src/industrial_pc_protocol.py
  device/industrial_pc/src/industrial_pc_hk_receiver.py
  DESTINATION share/${PROJECT_NAME}/device/industrial_pc/host_tools
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
