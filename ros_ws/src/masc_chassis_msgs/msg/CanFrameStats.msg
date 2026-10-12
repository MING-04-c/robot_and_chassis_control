# Function: Receive and parse statistics for one CAN ID.
# Msg Name: CanFrameStats

string can_id                         # 单位：无，CAN ID，格式示例：0x200
uint64 rx_count                       # 单位：帧，接收成功次数
uint64 parse_error_count              # 单位：帧，解析失败次数
uint64 drop_count                     # 单位：帧，丢弃次数
float64 frequency                     # 单位：Hz，低通平滑后的接收频率

uint64 last_rx_age_ms                 # 单位：ms，距离最后一次收到该 CAN ID 的时间；rx_count 为 0 时填 0
