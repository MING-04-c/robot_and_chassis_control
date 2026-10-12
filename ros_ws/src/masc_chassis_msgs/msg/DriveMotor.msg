# Function: Drive motor feedback for one protocol index.
# Msg Name: DriveMotor

std_msgs/Header header

uint8 index                  # 单位：无，协议索引，由 CAN ID 范围计算
float64 speed                # 单位：m/s，驱动轮线速度
float64 current              # 单位：A，驱动电机电流
float64 position             # 单位：m，驱动轮累计位置
