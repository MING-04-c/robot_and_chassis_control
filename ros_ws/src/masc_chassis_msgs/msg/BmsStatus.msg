# Function: Unified BMS / power-system status.
# Msg Name: BmsStatus

std_msgs/Header header

uint8 BMS_HEALTH_TIMEOUT = 255
uint8 BMS_HEALTH_UNKNOWN = 0
uint8 BMS_HEALTH_GOOD = 1
uint8 BMS_HEALTH_OVERHEAT = 2
uint8 BMS_HEALTH_DEAD = 3
uint8 BMS_HEALTH_OVERVOLTAGE = 4
uint8 BMS_HEALTH_UNSPEC_FAILURE = 5
uint8 BMS_HEALTH_COLD = 6
uint8 BMS_HEALTH_WATCHDOG_TIMER_EXPIRE = 7
uint8 BMS_HEALTH_SAFETY_TIMER_EXPIRE = 8


uint8 bat_soc             # 单位：%，电池剩余电量，范围 0-100
uint8 bat_soh             # 单位：%，电池健康度，范围 0-100
float64 bat_voltage       # 单位：V，电池电压
float64 bat_current       # 单位：A，电池电流，放电可为负值
float64 bat_max_voltage   # 单位：V，电池满电电压/电源系统允许最大电压
float64 bat_max_current   # 单位：A，电源系统允许最大电流
float64 bat_temp_max      # 单位：degC，电池最高温度
float64 bat_temp_min      # 单位：degC，电池最低温度
uint8 bms_health          # 单位：无，BMS 健康状态，见 BMS_HEALTH_* 常量
uint16 bms_cycle_count    # 单位：次，电池循环次数
