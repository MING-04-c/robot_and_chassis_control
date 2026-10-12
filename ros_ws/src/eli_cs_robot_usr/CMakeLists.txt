cmake_minimum_required(VERSION 3.8)
project(eli_cs_robot_usr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(X11 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(moveit_pose_executor src/moveit_control.cpp)
target_compile_features(moveit_pose_executor PUBLIC cxx_std_17)
ament_target_dependencies(
  moveit_pose_executor
  rclcpp
  geometry_msgs
  std_srvs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
)

add_executable(pose_target_publisher src/target_topic.cpp)
target_compile_features(pose_target_publisher PUBLIC cxx_std_17)
ament_target_dependencies(
  pose_target_publisher
  rclcpp
  geometry_msgs
  std_srvs
  tf2
)

add_executable(pose_keyboard_teleop src/pose_keyboard_teleop.cpp)
target_compile_features(pose_keyboard_teleop PUBLIC cxx_std_17)
ament_target_dependencies(
  pose_keyboard_teleop
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
)
target_link_libraries(pose_keyboard_teleop X11::X11)

add_executable(admittance_pose_controller src/admittance_pose_controller.cpp)
target_compile_features(admittance_pose_controller PUBLIC cxx_std_17)
ament_target_dependencies(
  admittance_pose_controller
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  std_srvs
  tf2
  tf2_ros
  moveit_core
  moveit_ros_planning
)

add_executable(staged_pose_coordinator src/staged_pose_coordinator.cpp)
target_compile_features(staged_pose_coordinator PUBLIC cxx_std_17)
ament_target_dependencies(
  staged_pose_coordinator
  rclcpp
  controller_manager_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)

install(TARGETS
  moveit_pose_executor
  pose_target_publisher
  pose_keyboard_teleop
  admittance_pose_controller
  staged_pose_coordinator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
