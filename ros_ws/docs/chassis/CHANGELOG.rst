CHANGELOG
=========

All notable changes to this repository are documented in this file.

1.0.1 (2026-07-11)
------------------

Changed:

* Added ROS2 launch arguments for ``cmd_vel`` topic remapping, odometry topic
  remapping, and ``command.stamped_control`` override.
* Changed the default ROS2 odometry publish check rate to ``100 Hz``.
* Improved chassis fault diagnostics. When ``chassis_state`` is fault, the ROS
  driver now prints a throttled warning with ``chassis_fault``,
  ``system_status_flags``, BMS health/SOC/voltage/current/max temperature, and
  any drive or servo motor driver entries whose ``driver_fault`` is non-zero.
* Made generated systemd CAN services tolerate failed ``modprobe`` calls for
  CAN modules, which helps on systems where the modules are built into the
  kernel or unavailable under the same name.

1.0.0 (2026-05-21)
------------------

Initial release.

Main features:

* ``masc_chassis_can_sdk`` pure C++ CAN V10 SDK with SocketCAN transport,
  typed frame packing/parsing, runtime cache, callbacks, health snapshots, and
  CAN frame diagnostics.
* ``masc_chassis_msgs`` ROS1/ROS2 interfaces for chassis status, communication
  status, BMS status, drive motor feedback, servo motor feedback, RC feedback,
  CAN frame statistics, and one-shot control services.
* ``masc_chassis_ros_driver`` ROS1/ROS2 driver with ``cmd_vel`` control,
  odometry, optional TF publishing, and chassis/BMS/motor/servo/IMU/RC feedback
  topics.
* One-shot services for motion mode switching, tow mode, fault clearing, and
  servo auto-calibration.
* Automatic standstill park-mode request with configurable auto-lock and
  auto-unlock behavior.
* CAN setup and usage documentation for SocketCAN, SLCAN, gs_usb, and
  systemd-networkd deployment.
