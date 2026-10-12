cmake_minimum_required(VERSION 3.8)
project(masc_chassis_msgs VERSION 1.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(MASC_CHASSIS_MSG_FILES
  "TaskStatus.msg"
  "BmsStatus.msg"
  "CanFrameStats.msg"
  "CanFrameStatsArray.msg"
  "ChassisStatus.msg"
  "CommStatus.msg"
  "DriveMotor.msg"
  "DriveMotorArray.msg"
  "DriveMotorStatus.msg"
  "DriveMotorStatusArray.msg"
  "RcStatus.msg"
  "ServoMotor.msg"
  "ServoMotorArray.msg"
  "ServoMotorStatus.msg"
  "ServoMotorStatusArray.msg"
)

set(MASC_CHASSIS_MSG_FILES_ROS2
  "msg/TaskStatus.msg"
  "msg/BmsStatus.msg"
  "msg/CanFrameStats.msg"
  "msg/CanFrameStatsArray.msg"
  "msg/ChassisStatus.msg"
  "msg/CommStatus.msg"
  "msg/DriveMotor.msg"
  "msg/DriveMotorArray.msg"
  "msg/DriveMotorStatus.msg"
  "msg/DriveMotorStatusArray.msg"
  "msg/RcStatus.msg"
  "msg/ServoMotor.msg"
  "msg/ServoMotorArray.msg"
  "msg/ServoMotorStatus.msg"
  "msg/ServoMotorStatusArray.msg"
)

set(MASC_CHASSIS_SRV_FILES
  "TaskControl.srv"
  "ClearFault.srv"
  "CalibrateServo.srv"
  "SetMotionMode.srv"
  "SetTowMode.srv"
)

set(MASC_CHASSIS_SRV_FILES_ROS2
  "srv/TaskControl.srv"
  "srv/ClearFault.srv"
  "srv/CalibrateServo.srv"
  "srv/SetMotionMode.srv"
  "srv/SetTowMode.srv"
)

if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "1")
  find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
  )

  add_message_files(FILES ${MASC_CHASSIS_MSG_FILES})
  add_service_files(FILES ${MASC_CHASSIS_SRV_FILES})

  generate_messages(DEPENDENCIES std_msgs)

  catkin_package(
    CATKIN_DEPENDS
      message_runtime
      std_msgs
  )
else()
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${MASC_CHASSIS_MSG_FILES_ROS2}
    ${MASC_CHASSIS_SRV_FILES_ROS2}
    DEPENDENCIES
      std_msgs
    ADD_LINTER_TESTS
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
endif()
