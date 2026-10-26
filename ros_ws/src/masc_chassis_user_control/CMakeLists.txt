cmake_minimum_required(VERSION 3.16)
project(masc_chassis_user_control)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/line_follow_node.py scripts/line_exposure_node.py scripts/tow_mode_node.py scripts/keyboard_teleop_node.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/line_follow_control.py scripts/exposure_control.py scripts/exposure_backend.py scripts/keyboard_control.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(NAME line_follow_control
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_line_follow_control.py)
  add_test(NAME line_follow_ros
    COMMAND ${CMAKE_COMMAND} -E env
      ROS_DOMAIN_ID=178 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_line_follow_ros.py)
  set_tests_properties(line_follow_ros PROPERTIES TIMEOUT 30)
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
