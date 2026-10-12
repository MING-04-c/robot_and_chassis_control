# Function: Receive and parse statistics for all tracked CAN IDs.
# Msg Name: CanFrameStatsArray

std_msgs/Header header                 # 单位：无，本组 CAN 统计信息的发布时间

masc_chassis_msgs/CanFrameStats[] stats
