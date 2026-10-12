# Function: RC channel feedback.
# Msg Name: RcStatus

std_msgs/Header header

bool online     # 单位：无，RC 遥控器是否在线
int8[7] channels # 单位：无，RC 通道 CH0-CH6 原始值，范围 -100~100
