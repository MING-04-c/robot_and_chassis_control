cmake_minimum_required(VERSION 3.16)
project(geminipro_camer_calibration)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/rgbd_view_node.py scripts/capture_batch.py scripts/capture_transfer.py
  scripts/target_transform_node.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/rgb_calibration.py scripts/capture_v54.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
