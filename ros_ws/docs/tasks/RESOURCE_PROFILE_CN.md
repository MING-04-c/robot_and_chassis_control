# 树莓派联合任务资源测试

资源采样必须和 ROS 2 launch 在同一个进程环境内运行，否则 `pidstat` 可能只能看到采样器，拿不到 ROS 节点。推荐使用：

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

scripts/profile_ros_command.sh /tmp/line_charge_$(date +%Y%m%d_%H%M%S) 300 -- \
  bash -lc 'source /opt/ros/jazzy/setup.bash; source install/setup.bash; \
  exec ros2 launch robot_task_control line_follow_only.launch.py \
    round_trip:=true charging_mode:=true line_show_window:=true \
    line_stop_marker_target_ratio:=0.5'
```

脚本结束后会生成 `meta.txt`、`summary.csv`、`pidstat.txt`、`ps.txt`、`mpstat.txt`、`vmstat.txt`、`thermal.csv` 和 `launch.log`。`summary.csv` 按 PID 给出平均/峰值 CPU、最大 RSS、线程数和完整命令行；`ps.txt` 用于区分同为 `python3` 的不同 ROS 节点。

联合机械臂、底盘和循迹的完整入口是 `line_follow_arm_test.launch.py`。在真实设备上测“全开但不执行动作”的启动基线：

```bash
scripts/profile_ros_command.sh /tmp/full_stations_idle_$(date +%Y%m%d_%H%M%S) 180 -- \
  bash -lc 'source /opt/ros/jazzy/setup.bash; source install/setup.bash; \
  exec ros2 launch robot_task_control line_follow_arm_test.launch.py \
    test_profile:=stations remote_control:=false auto_start:=false \
    launch_rviz:=false'
```

然后在同一路线、同一速度下分别测以下四组，每组至少 3 分钟并保留同一路线日志：

1. 只有 `line_follow_only`，预览关闭。
2. 只有 `line_follow_only`，预览开启（就是上面的用户命令）。
3. `line_follow_arm_test`，RViz 关闭、预览关闭。
4. 完整任务实际执行，机械臂在站点动作期间保留采样。

比较时看同一时间段的控制延迟日志和 `summary.csv`，不要只看任务结束后的平均值。现有控制器会记录“控制循环调度延迟”，循迹识别状态会记录处理耗时；这两个时间戳应和 `mpstat`、`vmstat`、`thermal.csv` 对齐。

判断是否是树莓派计算资源造成滞后：

- 4 核机器若某个核心长期接近 100%，或 `mpstat` 的运行队列/CPU PSI 长时间升高，同时控制循环延迟出现，优先按 CPU 争用处理。
- `vmstat` 的 `si/so` 非零、可用内存持续下降、memory PSI 或 major fault 持续升高，说明内存/交换区造成停顿。交换区不是性能余量，出现持续换入换出就应先减小图像缓存和后台任务。
- 温度持续接近降频阈值、`vcgencmd get_throttled` 不为 `0x0`，或频率明显下降，检查散热、供电和电源线。没有 `/dev/vcio` 时 `thermal.csv` 的时钟和 throttled 会是 `NA`，需要在实体树莓派上用 `vcgencmd` 补测。
- CPU、内存和温度都正常，但速度命令断流或响应仍滞后，转查物理链路：`ip -s -d link show can0` 的 CAN 错误计数、`dmesg` 的 USB 重连/掉帧、相机实际帧率，以及机器人网卡丢包和重传。此时不要继续通过提高线程数解决。

每次测试还应保存：

```bash
ros2 topic echo /line_follow/status > /tmp/line_status.log
ros2 topic echo /task_controller/status > /tmp/task_status.log
ip -s -d link show can0 > /tmp/can_status.txt
dmesg -T > /tmp/dmesg.txt
```

真实设备上要先退出其他会发布底盘速度的 launch，并确认 `can0`、`/dev/video0`、机械臂网卡和串口均存在。当前开发环境没有这些设备，已经完成的是脚本验证和无硬件启动基线；当前环境的 CAN/网卡/文件系统错误不能作为真实任务负载结论。
