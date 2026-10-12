# Function: CAN SDK communication status and chassis identity/version data.
# Msg Name: CommStatus

std_msgs/Header header

uint8 COMM_STATE_UNKNOWN = 0
uint8 COMM_STATE_DISCONNECTED = 1
uint8 COMM_STATE_CONNECTING = 2
uint8 COMM_STATE_PROTOCOL_MISMATCH = 3
uint8 COMM_STATE_CONNECTED = 4

uint8 FAIL_REASON_NONE = 0
uint8 FAIL_REASON_PORT_OPEN_FAILED = 1
uint8 FAIL_REASON_VERSION_TIMEOUT = 2
uint8 FAIL_REASON_PROTOCOL_VERSION_MISMATCH = 3
uint8 FAIL_REASON_READ_ERROR = 4
uint8 FAIL_REASON_WRITE_ERROR = 5
uint8 FAIL_REASON_TRANSPORT_ERROR = 6
uint8 FAIL_REASON_UNKNOWN = 255

# Chassis type values from system_version_feedback (0x205) Byte6.
# Newer CAN V10 docs define currently supported values directly.
uint8 CHASSIS_TYPE_RESERVED = 0                  # 0x00
uint8 CHASSIS_TYPE_DIFF_2WD = 1                  # 0x01
uint8 CHASSIS_TYPE_DIFF_4WD = 2                  # 0x02
uint8 CHASSIS_TYPE_DIFF_6WD = 3                  # 0x03
uint8 CHASSIS_TYPE_DIFF_8WD = 4                  # 0x04

uint8 CHASSIS_TYPE_TRICYCLE_UNSPECIFIED = 32     # 0x20
uint8 CHASSIS_TYPE_TRICYCLE_FRONT = 33           # 0x21
uint8 CHASSIS_TYPE_TRICYCLE_REAR = 34            # 0x22

uint8 CHASSIS_TYPE_ACKERMANN_UNSPECIFIED = 64    # 0x40
uint8 CHASSIS_TYPE_ACKERMANN_FRONT = 65          # 0x41
uint8 CHASSIS_TYPE_ACKERMANN_REAR = 66           # 0x42
uint8 CHASSIS_TYPE_ACKERMANN_FRONT_REAR = 67     # 0x43

uint8 CHASSIS_TYPE_AWS_4WS = 97                  # 0x61
uint8 CHASSIS_TYPE_AWS_6WS = 98                  # 0x62
uint8 CHASSIS_TYPE_AWS_8WS = 99                  # 0x63

uint8 comm_state                    # 单位：无，CAN 通讯状态，见 COMM_STATE_* 常量
uint8 fail_reason                   # 单位：无，通讯失败原因，见 FAIL_REASON_* 常量
bool motion_command_timeout_active  # 单位：无，运动控制命令是否处于超时保护状态

uint64 connection_attempt_count     # 单位：次，连接尝试次数
uint64 transport_error_count        # 单位：次，CAN transport 错误次数
uint64 tx_error_count               # 单位：次，发送错误次数
uint64 rx_error_count               # 单位：次，接收错误次数

uint8[5] chassis_uuid               # 单位：无，底盘 UUID
uint8 system_software_hardware_version # 单位：无，系统软硬件版本原始字节，来自 0x205 Byte5
uint8 chassis_type_version          # 单位：无，底盘类型版本，见 CHASSIS_TYPE_* 常量
uint8 chassis_protocol_version      # 单位：无，通讯协议版本
