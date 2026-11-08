cmake_minimum_required(VERSION 3.16)
project(masc_chassis_user_control)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/line_follow_node.py scripts/line_exposure_node.py scripts/tow_mode_node.py scripts/keyboard_teleop_node.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/forward_navigation.py scripts/dock_departure.py scripts/charging_dock.py scripts/start_alignment.py scripts/reverse_visual.py scripts/line_follow_workers.py scripts/recorded_route.py scripts/stop_marker.py scripts/line_follow_control.py scripts/exposure_control.py scripts/exposure_backend.py scripts/keyboard_control.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(NAME forward_navigation
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_forward_navigation.py)
  add_test(NAME line_follow_control
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_line_follow_control.py)
  add_test(NAME marker_latency
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_marker_latency.py)
  add_test(NAME line_follow_ros
    COMMAND ${CMAKE_COMMAND} -E env
      ROS_DOMAIN_ID=178 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_line_follow_ros.py)
  set_tests_properties(line_follow_ros PROPERTIES TIMEOUT 90)
  add_test(NAME reverse_tracking_limits_ros
    COMMAND ${CMAKE_COMMAND} -E env
      ROS_DOMAIN_ID=183 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ${Python3_EXECUTABLE} -c
      "import sys; sys.path.insert(0, '${CMAKE_CURRENT_SOURCE_DIR}/test'); from test_line_follow_ros import reverse_tracking_limits_ros; reverse_tracking_limits_ros()")
  set_tests_properties(reverse_tracking_limits_ros PROPERTIES TIMEOUT 15)
  add_test(NAME start_alignment_ros
    COMMAND ${CMAKE_COMMAND} -E env
      ROS_DOMAIN_ID=182 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ${Python3_EXECUTABLE} -c
      "import sys; sys.path.insert(0, '${CMAKE_CURRENT_SOURCE_DIR}/test'); from test_line_follow_ros import start_alignment_ros; start_alignment_ros(); start_alignment_ros(replay=True)")
  set_tests_properties(start_alignment_ros PROPERTIES TIMEOUT 30)
  add_test(NAME standard_odom_replay_ros
    COMMAND ${CMAKE_COMMAND} -E env
      ROS_DOMAIN_ID=184 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ${Python3_EXECUTABLE} -c
      "import sys; sys.path.insert(0, '${CMAKE_CURRENT_SOURCE_DIR}/test'); from test_line_follow_ros import standard_route_ros; standard_route_ros(); [standard_route_ros(check_loss=case) for case in ('recover', 'timeout', 'distance', 'finish', 'odom', 'deviation', 'curve', 'ramp')]")
  set_tests_properties(standard_odom_replay_ros PROPERTIES TIMEOUT 80)
  add_test(NAME standard_visual_finish_ros
    COMMAND ${CMAKE_COMMAND} -E env
      ROS_DOMAIN_ID=185 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ${Python3_EXECUTABLE} -c
      "import sys; sys.path.insert(0, '${CMAKE_CURRENT_SOURCE_DIR}/test'); from test_line_follow_ros import standard_route_ros; standard_route_ros(check_finish=True); standard_route_ros(check_finish='timeout')")
  set_tests_properties(standard_visual_finish_ros PROPERTIES TIMEOUT 65)
  add_test(NAME return_visual_finish_ros
    COMMAND ${CMAKE_COMMAND} -E env
      ROS_DOMAIN_ID=181 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ${Python3_EXECUTABLE} -c
      "import sys; sys.path.insert(0, '${CMAKE_CURRENT_SOURCE_DIR}/test'); from test_line_follow_ros import recorded_route_camera_ros; recorded_route_camera_ros(True); recorded_route_camera_ros(True, True)")
  set_tests_properties(return_visual_finish_ros PROPERTIES TIMEOUT 90)
  add_test(NAME line_follow_launch_scope
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch_scope.py)
  add_test(NAME exposure_control
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_exposure_control.py)
  add_test(NAME exposure_ros
    COMMAND ${CMAKE_COMMAND} -E env
      ROS_DOMAIN_ID=179 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_exposure_ros.py)
  set_tests_properties(exposure_ros PROPERTIES TIMEOUT 30)
  add_test(NAME tow_session
    COMMAND ${CMAKE_COMMAND} -E env
      ROS_DOMAIN_ID=180 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_tow_session.py)
  set_tests_properties(tow_session PROPERTIES TIMEOUT 40)
  add_test(NAME keyboard_teleop
    COMMAND ${CMAKE_COMMAND} -E env
      ROS_DOMAIN_ID=181 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ROS_LOG_DIR=/tmp/masc-keyboard-test-logs
      ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_keyboard.py)
  set_tests_properties(keyboard_teleop PROPERTIES TIMEOUT 35)
endif()

ament_package()
