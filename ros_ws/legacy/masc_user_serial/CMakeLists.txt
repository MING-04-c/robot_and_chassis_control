cmake_minimum_required(VERSION 3.16)
project(masc_user_serial LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
add_executable(masc_tcp_node src/masc_tcp_node.cpp)
target_compile_features(masc_tcp_node PRIVATE cxx_std_17)
ament_target_dependencies(masc_tcp_node rclcpp std_msgs)
install(TARGETS masc_tcp_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/camera_image_tcp_node.py
  scripts/status_tcp_node.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES
  tools/receive_camera_tcp.py
  tools/test_chassis_task_tcp.py
  DESTINATION share/${PROJECT_NAME}/tools)
install(FILES CAMERA_TCP_PROTOCOL.md MASC_COMMUNICATION_PROTOCOL.md
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
