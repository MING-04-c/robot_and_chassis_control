cmake_minimum_required(VERSION 3.16)
project(camer_calibration)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/calibrate_camera.py
  scripts/tag_pose_node.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/tag_geometry.py scripts/tag_panel.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
