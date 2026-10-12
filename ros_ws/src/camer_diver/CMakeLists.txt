cmake_minimum_required(VERSION 3.16)
project(camer_diver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui)

add_library(camera_capture src/v4l2_capture.cpp src/frame_conversion.cpp)
target_include_directories(camera_capture PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_capture PUBLIC ${OpenCV_LIBS})

add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node camera_capture)
ament_target_dependencies(camera_node
  rclcpp rcl_interfaces sensor_msgs std_msgs cv_bridge camera_info_manager)

add_library(line_tracking src/line_tracking.cpp)
target_include_directories(line_tracking PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${OpenCV_INCLUDE_DIRS})
target_link_libraries(line_tracking PUBLIC ${OpenCV_LIBS})
add_executable(line_tracker_node src/line_tracker_node.cpp)
target_link_libraries(line_tracker_node line_tracking)
ament_target_dependencies(line_tracker_node
  rclcpp rcl_interfaces sensor_msgs std_msgs geometry_msgs cv_bridge)

install(TARGETS camera_capture camera_node line_tracking line_tracker_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/line_demo.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_camera test/test_camera.cpp)
  target_link_libraries(test_camera camera_capture)
  ament_add_gtest(test_line_tracking test/test_line_tracking.cpp)
  target_link_libraries(test_line_tracking line_tracking)
endif()

ament_package()
