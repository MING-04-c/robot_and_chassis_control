# Function: Chassis runtime status from motion mode and system info feedback.
# Msg Name: ChassisStatus

std_msgs/Header header

uint8 CHASSIS_STATE_UNKNOWN = 0
uint8 CHASSIS_STATE_ESTOP = 1
uint8 CHASSIS_STATE_TOW = 2
uint8 CHASSIS_STATE_FAULT = 3
uint8 CHASSIS_STATE_SELF_CHECK = 4
uint8 CHASSIS_STATE_PARK = 5
uint8 CHASSIS_STATE_REMOTE_CONTROL = 6
uint8 CHASSIS_STATE_COMM_CONTROL = 7

uint8 MOTION_MODE_DEFAULT_CHASSIS = 0
uint8 MOTION_MODE_PARK = 1
uint8 MOTION_MODE_ACKERMANN = 2
uint8 MOTION_MODE_SPIN = 3
uint8 MOTION_MODE_LATERAL = 4
uint8 MOTION_MODE_DIAGONAL = 5

uint8 chassis_state            # 单位：无，底盘显示状态，见 CHASSIS_STATE_* 常量
uint8 current_motion_mode      # 单位：无，当前底盘运动模式，见 MOTION_MODE_* 常量
bool motion_mode_switching     # 单位：无，是否处于运动模式切换中
bool brake_active              # 单位：无，抱闸是否激活
bool low_battery               # 单位：无，是否低电量
bool excessive_tilt            # 单位：无，是否倾角过大
bool manual_charging           # 单位：无，是否手动充电
bool dock_charging             # 单位：无，是否充电桩充电
bool main_power_active         # 单位：无，动力电源是否接通，来自主接触器状态
uint8 obstacle_status_flags    # 单位：无，障碍状态 flags，bit0 前，bit1 后，bit2 左前，bit3 左后，bit4 右前，bit5 右后
uint32 system_status_flags     # 单位：无，0x204 Byte0-Byte3 原始状态值，Byte0 主状态，Byte1 系统 flags，Byte2 障碍 flags，Byte3 预留
uint32 chassis_fault           # 单位：无，底盘故障码 bitfield/raw code
