# 多点漆膜厚度接触检测

真机检测测试及本包 `detect.launch.py` 默认启动测厚仪串口节点，关闭周期轮询，
在接触保持开始时发一次采集触发；已有驱动时传 `start_thickness_detector:=false`。
`detactor_config_file` 可指定串口配置，默认设备 `/dev/ttyCH341USB0`。
配置中的 `measurement_unit` 应按仪器设置填写 um/mm，未确认时保持 unknown。

仪器 `/detactor/measurement` 发布最新单值 JSON，包括 value、unit、sequence 和 received_at_ros；
即使数值相同，新有效回包也更新序号和时间。`/detactor/latest_measurement`（Trigger）读取缓存。
任务只关联本次 HOLD 触发后收到的回包，发布 `/detect_controller/thickness_result`；
保持结束将该点最后一次读数追加到 `~/.ros/detect_control/thickness_results.jsonl`，
也可通过 `measurement_results_file` 配置路径。没有新读数时记录 available=false，
不使用上一点读数。缺少仪器不阻塞现有运动测试，但不表示完成了真实厚度测量。
串口时间只是回包接收时间；设备返回历史窗口时无法证明物理测量发生时间。

工控机可通过现有 9010 签名通道查询最新仪器单值；详见
`eli_cs_robot_serial/device/industrial_pc/README.md`。查询不会触发测量。

`terminal_link_1`（Joint_1 子坐标系）是相机坐标，`terminal_link_2`（Joint_2 子坐标系）是探头检测点；机器人控制点仍为 `tool0`。相机相对坐标先转换到基座并保存，再执行整组检测。

## 执行流程

```
记录一次原位（六轴关节位置）
保存检测点 1、2、3……的基座坐标
开始整组任务
  原位 → 规划到点 1 上方 → Z 向导纳接触 → 20 N 停止 → 保持 5 秒 → +Z 撤离 → 原位
  原位 → 规划到点 2 上方 → Z 向导纳接触 → 20 N 停止 → 保持 5 秒 → +Z 撤离 → 原位
  ……
  原位 → 最后一个点 → 接触检测 → 撤离 → 同一原位 → DONE
```

可在拍照存点前用 `record_home` 显式记录原位；没有提前记录时，`start` 自动记录开始时的实际关节位置。整组只使用这一份原位，不在每点结束后覆盖。若开始时已离开显式记录的原位，会先规划回原位再检测第一点。

默认按输入顺序逐点执行。MoveIt 为每次原位到预接近点、撤离和回原位规划路径，考虑新增末端碰撞几何；不会把检测点直接连成穿过桌面的直线。每次返回都确认实际关节到位、外力回落后才开始下一点。任何点失败即停止后续所有点，状态为 `FAULT`，不跳过失败点继续测。

## 坐标与保存

位置为米、角度为弧度、四元数为 XYZW。所有存储目标都代表**基座下的探头检测点位姿**，不是 tool0 位姿：

```
base_T_tool0_goal = base_T_probe_target × inverse(tool0_T_probe)
```

相机转换接口见 `geminipro_camer_calibration/README.md`：

- 单点输入 `camera_point`（PointStamped），转换后 `target_base` 逐个追加到待测列表；相同位姿不重复追加。
- 整组输入 `camera_points`（PoseArray，仅用 position），转换后 `targets_base` 一次替换待测列表。需要每点独立接触朝向时，用 `camera_targets`。
- 检测控制器仅接受基座坐标，默认最多 100 点。整组有非法点则整组拒绝，不部分接收。
- 接收到的检测点自动原子保存到 `targets_file`，默认 `~/.ros/detect_control/targets.yaml`。保存失败时不更新待测列表。
- 收到新相机点时检查观测时间，但**存好的基座点不会因检测耗时超过 30 秒而失效**，也不会随机器人移动重新变换。前提是检测期间物体与机器人基座关系没有改变；底盘或工件移动后需清空并重新采集。
- 执行中锁存整组，拒绝修改、加载、清空点列表；运行中发来的新点不会排入当前或下一组。
- 完成后保留点文件和列表供核对；再次 start 会从第一点重新执行整组。程序重启不会自动加载旧点或恢复旧原位。

YAML 多点格式见 `config/targets.example.yaml`。可将其修改为现场基座检测点，通过 `load_targets` 加载。示例数值不能直接作为现场目标。旧的 `target_source: fixed` + `fixed_position/fixed_quaternion_xyzw` 仍支持单点测试；多点文件请使用默认 `target_source: camera` 加载列表。

## 启动和操作

先按原流程启动真机驱动、MoveIt、力传感器、更新后的导纳。其他任务协调器、目标发布器和键盘伺服不能同时输出运动。首次测试须确认无接触时的力零偏及负载补偿。

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch detect_control detect.launch.py
```

指定任务参数和检测点保存位置：

```bash
ros2 launch detect_control detect.launch.py \
  config:=/绝对路径/detect.yaml \
  targets_file:=/绝对路径/targets.yaml
```

只启动接口不会运动。建议先显式记录原位，再采集相机点：

```bash
ros2 service call /detect_controller/record_home std_srvs/srv/Trigger '{}'
ros2 launch geminipro_camer_calibration target_transform.launch.py
```

随后由主机/定位程序发布相机检测点，控制器自动保存。若使用已有基座多点 YAML，调用：

```bash
ros2 service call /detect_controller/load_targets std_srvs/srv/Trigger '{}'
```

查看 `queued_points` 确认数量，然后启动整组：

```bash
ros2 topic echo /detect_controller/status
ros2 service call /detect_controller/start std_srvs/srv/Trigger '{}'
```

取消、清空（清空同时更新 YAML）：

```bash
ros2 service call /detect_controller/cancel std_srvs/srv/Trigger '{}'
ros2 service call /detect_controller/clear_targets std_srvs/srv/Trigger '{}'
```

开始服务的成功只表示接收任务，最终看 `DONE` 或 `FAULT`。状态包含 `queued_points`、`batch_points`、`current_point`（从 1 开始）、`completed_points`；只有接触检测完成且回到原位才计为已完成一个点。

## 每点接触与异常处理

预接近规划根据探头长度收紧 tool0 姿态误差预算，并检查实际探头是否到位，避免长探头放大姿态偏差。预接近点默认在目标上方 20 mm；沿 **base_link −Z** 以示例速度 2 mm/s 推进，每个名义目标先检查无碰撞 IK，姿态和 XY 保持。`abs(base_link Fz) >= 20 N` 时请求关闭导纳；底层发送当前位置保持轨迹，确认停止后计时 **5 秒**。保持开始发布接触实际位姿，并向 `/detactor/acquire_trigger` 发一次采集触发，复用原测厚串口节点。未连接测厚节点会提示，不能把接触完成当成已获得真实厚度数据。

20 N 是停止推进阈值，不是恒力控制；5 秒保持位置时接触力可能改变。滤波/通信/轨迹均有延迟，不能保证瞬时力恰好止于 20 N。导纳让位也不等于连续扫掠碰撞检测。

`max_penetration` 示例 5 mm、合力保护 40 N、速度和工作空间均可在 `config/detect.yaml` 配置，须与现场探头弹性、刚度和方向匹配。未达到 20 N 就到行程上限时停止，不继续探入。数据超时、无解/碰撞、超力、执行失败、取消均保持并停止后续点，不盲目自动返回；ROS 请求不是硬件急停。

原始 `/sensor/wrench` 用于断流检查，`/admittance_controller/measured_wrench` 的去零滤波基座力用于接触判断。测量窗口 `/detect_controller/measurement_active`、接触位姿 `/detect_controller/contact_pose` 可结合状态中的当前点序号记录。

导纳 `/tracking_target_pose` 连续更新不重置柔顺偏移；默认仅 Z 向柔顺，偏移最多 3 mm、偏移变化最多 2 mm/s。参数为 `tracking_vertical_only`、`tracking_max_compliance_offset`、`tracking_max_compliance_speed`。连续目标超时 0.5 秒或有效力超时 0.3 秒会保持。原单点/多点导纳接口行为保留。

## 固定 tool0 初始位置（测试任务可选）

检测测试可启用 `initial_position_enabled`，配置 `initial_position` 为基座下的 tool0 XYZ，并调用 `/detect_controller/prepare_initial`（Trigger）。该服务先关闭导纳、由 MoveIt 保持请求时朝向规划到初始点，等待执行后的新 TF 确认到位后记录关节原位，并返回 IDLE；开始服务成功仅代表已接收，状态中的 `initial_position_ready` 才表示准备完成。普通检测配置默认不启用此功能，不会因启动节点自动移动。

本轮测试初始点与检测点均按 tool0 输入；测试节点通过 `real_target_frame: tool0` 完成接口转换。检测器内部的目标列表仍统一表示探头检测点，避免与正式多点/相机流程混用。

到达检测目标后进入 `COMPLIANT_HOLD`：固定最后的目标，持续发布并保持导纳使能，不再向下推进，也不会因等待接触力而触发阶段超时。`max_penetration` 仅作为目标下方的实际位置保护余量，不再用于主动超出目标。接近和柔顺保持期间达到 20 N 均会停止输出，保持位置 5 秒后撤离并返回原位。数据断流、外控断开、超载和位置偏离保护仍生效；可通过 cancel 服务或退出任务结束等待。测试任务在柔顺等待期间暂停累计运动超时，其他阶段仍有时间限制。
