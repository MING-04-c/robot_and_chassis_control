# Function: Servo motor feedback for one protocol index.
# Msg Name: ServoMotor

std_msgs/Header header

uint8 SERVO_CALIBRATION_NOT_CALIBRATED = 0
uint8 SERVO_CALIBRATION_CALIBRATING = 1
uint8 SERVO_CALIBRATION_CALIBRATED = 2

uint8 index                  # 单位：无，协议索引，由 CAN ID 范围计算
uint8 calibration_state      # 单位：无，舵机自动标定状态，见 SERVO_CALIBRATION_* 常量
float64 angle                # 单位：rad，舵机角度
float64 current              # 单位：A，舵机电流
