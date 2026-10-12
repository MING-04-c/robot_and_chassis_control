cmake_minimum_required(VERSION 3.16)
project(masc_chassis_user_control)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/line_follow_node.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/line_follow_control.py DESTINATION lib/${PROJECT_NAME})
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
endif()

ament_package()
