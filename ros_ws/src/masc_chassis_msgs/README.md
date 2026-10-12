# MASC Chassis ROS Interfaces

MASC is the English company name of 多机协同（苏州）有限公司.

`masc_chassis_msgs` defines ROS1/ROS2 messages and services for the MASC chassis CAN driver.

The messages are aligned with upper-layer ROS driver outputs. Motion command/feedback and raw CAN
frames stay inside the CAN SDK or driver implementation instead of being exposed as ROS messages.

## Message Groups

| Group | Interfaces |
|---|---|
| One-shot services | `SetMotionMode`, `SetTowMode`, `ClearFault`, `CalibrateServo` |
| Core feedback | `ChassisStatus` |
| BMS feedback | `BmsStatus` |
| Indexed feedback | `DriveMotor`, `DriveMotorArray`, `DriveMotorStatus`, `DriveMotorStatusArray`, `ServoMotor`, `ServoMotorArray`, `ServoMotorStatus`, `ServoMotorStatusArray` |
| Operator feedback | `RcStatus` |
| Communication/debug | `CommStatus`, `CanFrameStats`, `CanFrameStatsArray` |

## Conventions

- Published ROS values use driver-facing units: `m`, `m/s`, `rad`, `rad/s`, `V`, `A`, `degC`.
- `ServoMotor.angle` is published in `rad`. The CAN SDK may decode `servo_feedback (0x271-0x278)`
  from the firmware/docx degree unit, and the ROS driver should convert it before publishing.
- IMU feedback uses the standard `sensor_msgs/Imu` message in the ROS driver. This interface
  package does not define a custom IMU message.
- `RcStatus.online` comes from Byte0 of `rc_feedback (0x301)`. `RcStatus.channels` keeps raw
  `CH0-CH6` values from Byte1-Byte7, range `-100` to `100`. Channel meanings are not interpreted
  by this interface package.
- `BmsStatus` is a unified power-system interface. CAN fills the available battery fields; missing
  values keep defaults, or are filled by serial protocol / upper-layer power manager.
- `ChassisStatus` is the runtime status interface: current motion mode, mode switching flag,
  primary chassis state, commonly used decoded status flags, obstacle status flags, raw system
  status word, and chassis fault code.
- `ChassisStatus.current_motion_mode` uses upper-layer ROS semantics: `0` means default chassis
  mode, and `1` means park mode. These values match the CAN default/park mode values.
- `CommStatus` is the CAN SDK communication interface: communication state, failure reason,
  transport counters, command timeout protection state, UUID, system software/hardware version byte,
  chassis type version, and protocol version.
- `CommStatus.system_software_hardware_version` keeps raw Byte5 from `system_version_feedback`;
  `chassis_type_version` comes from Byte6 and is published as the raw chassis type value defined
  by the current CAN V10 docs.
- `index` is the protocol index computed from the CAN ID range. It is not a physical wheel/servo
  position.
- Fault fields are raw protocol values until the firmware fault bit definitions are finalized.
- One-shot commands use ROS services. The service response only reports whether the driver accepted
  and sent the request; protocol ACK/result confirmation should be handled by observing feedback.
- `SetMotionMode` is available only when `CommStatus.chassis_type_version` is `0x61`, `0x62`, or
  `0x63`.
- `SetTowMode` sends the one-shot tow mode command. The service response only reports whether the
  command was sent.
- `CalibrateServo` starts firmware automatic calibration. The request does not provide a calibration
  value and does not set the current position as the zero point.
