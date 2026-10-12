# Function: Servo motor runtime status for one protocol index.
# Msg Name: ServoMotorStatus

std_msgs/Header header

uint8 index                  # 单位：无，协议索引，由 CAN ID 范围计算
float64 motor_temperature    # 单位：degC，舵机电机温度
float64 driver_temperature   # 单位：degC，驱动器温度
uint32 driver_fault          # 单位：无，驱动器故障码 bitfield/raw code
