# 机械臂与小车整体往返实车测试

入口复用 `robot_task_control/line_follow_arm_test.launch.py`，选择 `test_profile:=return_charge`。
本轮验证机械臂姿态切换、视觉循迹、实际路线录制与返程，以及TCP通信。
不执行四个笛卡尔点、拍照、测厚或20N触碰动作；不加载导纳、力传感器、HK相机、深度相机和RViz。

9010 TCP桥开启，额外正常/异常状态汇总关闭（不订阅 `/rosout`，不启动汇总定时器）。
机械臂实际到位、设备就绪、保护停止和底盘运动保护继续生效。
基础任务状态仍可经TCP查询；关闭汇总不代表设备健康。

联合入口、公共任务栈和单独机械臂驱动入口默认`ros_discovery_range:=LOCALHOST`。
该参数在节点启动前统一设置`ROS_AUTOMATIC_DISCOVERY_RANGE`，隔离局域网同名ROS服务，
不限制机械臂厂商TCP/RTSI连接、工控机TCP 9010或其他设备的直接网口通信。
需要跨机器ROS通信时显式设置`ros_discovery_range:=SUBNET`，并统一使用专用`ROS_DOMAIN_ID`。
launch只设置子进程环境，不修改终端或`.bashrc`；另开终端查看状态时先执行：

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

## 动作顺序与通过条件

| 阶段 | 实际动作 | 通过条件 |
|---|---|---|
| 1 | 加载机械臂驱动、底盘和TCP，先等待机械臂控制器 | 6个活动控制器及2个备用控制器状态正确，外控连接连续稳定5秒；随后加载MoveIt与USB循迹，设备检查通过 |
| 2 | 机械臂运动到行车位 | 本次关节任务实际到位，才放行底盘 |
| 3 | 摄像头停车观察、估距后分轴对齐出发横线 | `start_alignment_complete`，随后才建立记录首点 |
| 4 | 小车沿已有标准路线前进，视觉辅助并记录实际里程计路线 | 忽略出发横线，在第一条有效停止横线到位停车 |
| 5 | 确认底盘停稳并封存路线 | 保存完成；机械臂动作期间不重置里程计、不覆盖已封存路线 |
| 6 | 机械臂运动到过渡位 | 本次关节任务实际到位 |
| 7 | 等待5秒 | 从过渡位实际到位后开始计时，保持底盘停车 |
| 8 | 机械臂运动到行车位 | 本次实际到位后，才允许返程 |
| 9 | 小车沿本次记录路线倒车返回 | 通过原弯道；末段视觉摆正、横移回中、找横线并接近充电位 |
| 10 | 充电确认、底盘停稳，结束任务 | 连续新反馈确认充电；工作流完成，机械臂保持行车位，不自动循环 |

默认全程自动执行，不需要在到站、等待结束或返程前另发开始指令。
本测试复用充电返程：最终以充电反馈确认完成，单独看见横线或里程计回到起点不算整体通过。

两组姿态共用 `eli_cs_robot_moveit_config/config/arm_joint_postures.yaml`，角度单位为度：

| 名称 | 基座、肩部、肘部、手腕1、手腕2、手腕3 |
|---|---|
| 行车位 `travel` | -45, 15, -71, 4.4, 148, 75 |
| 过渡位 `detection_ready` | -45, 0, 0, 0, 90, 0 |

姿态通过MoveIt规划和碰撞检查；规划失败、关节反馈失效或未实际到位时停止流程。
当前场景包含地面，现场桌面及其他障碍仍需与实际碰撞场景一致。

## 启动前

1. 结束旧的机械臂/底盘/联合任务，避免重复启动驱动或占用9010端口。
2. 将小车放回起步横线附近，保证相连主线与横线都在画面内。不能从上次倒车卡住的位置直接重启整套往返。
3. 确认机械臂外控条件、底盘通信和现场运动空间。需要时手动配置CAN：

```bash
cd ~/project/robot_and_chassis_control/ros_ws
sudo bash scripts/setup_can.sh auto
```

源码修改后构建一次；本次修改已完成构建时可跳过构建命令：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select masc_chassis_user_control eli_cs_robot_driver eli_cs_robot_serial robot_task_control --symlink-install
source install/setup.bash
```

构建后，**每个用于启动或诊断的旧终端都要重新执行 `source install/setup.bash`**。
尤其从普通安装切换到 `--symlink-install` 后，旧终端缺少 build 目录的 Python 路径，
即使 ROS 能找到可执行文件，也可能报 `PackageNotFoundError: robot-task-control`。
可先验证入口导入，下面命令不会启动节点或发送运动指令：

```bash
python3 - <<'PY'
from importlib.metadata import distribution
package = distribution('robot-task-control')
for name in ('velocity_mux', 'arm_joint_postures', 'task_controller'):
    entry = next(e for e in package.entry_points
                 if e.group == 'console_scripts' and e.name == name)
    assert callable(entry.load())
    print('OK:', name)
PY
```

如果 TCP 报 `Address already in use`，先用 `ss -ltnp 'sport = :9010'` 确认占用进程，
在原启动终端正常结束旧任务；不要直接再次启动一套驱动，也不要通过更换端口掩盖冲突。
端口占用能证明 TCP 监听冲突，不能单独证明机械臂控制器重复。

如果 `joint_state_broadcaster` 仍然报 `Failed loading controller`，工作流尚未开始。
这是加载服务返回失败，与前面的 `list_controllers` 等待超时是两件事，
不能只靠延长超时认定修复。确认旧驱动已退出后，可在原启动命令增加
`controller_manager_log_level:=debug`，保存完整日志进一步定位。

## 本轮推荐启动命令

**启动后设备就绪即自动动作。** TCP开启不要求工控机在线；没有连接也会自动执行。
下面命令使用已有标准轨迹，前进上限0.10m/s、倒车上限0.05m/s；起步对齐、离桩、弯道和末段可能更慢。
标准文件只读，倒车使用本次实际前进记录。

联测入口默认`line_processing_fps:=8.0 line_opencv_threads:=2 line_marker_opencv_threads:=1`，
分别限制视觉处理频率、主线图像线程数和横线检测线程数；相机仍采集15帧，控制和里程计频率不变。
两个循迹入口共用公共任务栈，均接受这些参数；单独循迹未指定横线线程时沿用主线线程数。
线程数是OpenCV算法的并行上限，不是专用CPU核或进程总线程数。
视觉C++包未指定构建类型时默认`RelWithDebInfo`（`-O2`）；显式指定Debug等类型时尊重调用者。
状态、中线和掩膜的接收队列只保留最新一条；后台仍使用一个执行任务加一个可替换的待处理槽。
排查不动时先关闭`line_show_window`减少预览开销，视觉新鲜度和横线几何检查保持生效。

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch robot_task_control line_follow_arm_test.launch.py \
  test_profile:=return_charge \
  start_tcp:=true enable_status_monitor:=false remote_control:=false \
  launch_admittance:=false launch_force_sensor:=false launch_rviz:=false \
  line_navigation_mode:=replay \
  standard_route_file:="$PWD/routes/standards/shuttle_test_01/standard_route.json" \
  prepare_line_camera_early:=false line_processing_fps:=8.0 \
  line_opencv_threads:=2 line_marker_opencv_threads:=1 \
  line_show_window:=false line_follow_speed:=0.1 line_reverse_speed:=0.05 \
  line_stop_marker_target_ratio:=0.5 \
  reverse_route_max_curvature:=5.0 \
  charging_status_layout:=byte_flags \
  route_file:=~/.ros/line_follow/arm_chassis_round_trip_test.json \
  2>&1 | tee /tmp/arm_chassis_round_trip_test.log
```

`byte_flags` 与当前循迹实车配置一致。只有已核实固件使用整个uint32状态码bit3充电时，
才改用 `charging_status_layout:=bitfield32`；底盘控制器和TCP桥会使用同一个参数。

测试文件与默认 `last_route.json` 分开。再次使用同名测试文件会覆盖该测试文件，
任务封存的快照另存于 `~/.ros/line_follow/snapshots/`；多轮对比可分别使用
`arm_chassis_round_trip_test_01.json`、`_02.json`、`_03.json`。

## 现场怎么观察

- 机械臂未确认到行车位前，小车应始终停车。
- 起步出现 `waiting_for_stop`、`settling` 或单步完成时，尚未完成整体对齐；以 `start_alignment_complete` 为准。
- 若一直`waiting_for_line`/`waiting_for_marker`，检查“起步对齐诊断”的
  `upstream_age_ms`（源图到提交横线任务，含上游处理/传输）、`tracker_processing_ms`（主线计算）、
  `worker_wait_ms`（后台槽等待）、`detector_processing_ms`（横线任务，含图像转换）与
  `result_collect_ms`（计算完成到控制线程取走）。过期结果也会打印分段耗时，但不会刷新有效观测。
  这些是含调度影响的墙钟耗时，不能直接当CPU占用率；`bar_angle`等计数是被拒绝的候选数，
  不能仅凭它断言真实横线角度有问题。降低负载后若候选仍不稳定，须查看对应原图与掩膜。
- 去程重点看直线是否持续偏到一侧、出弯是否反复左右摆动。记录的是实际行驶轨迹，不是胶带中心线；明显偏线不能算录制合格。
- 到站先确认小车停稳，再观察机械臂到过渡位。完整5秒应从实际到位后计算，不能从发出运动请求时算起。
- 等待结束后先看机械臂回行车位，再看小车倒车。相机在机械臂动作/等待期间按需暂停，恢复返程前须等有效图像；不会重置路线。
- 倒车重点观察上次剩余约0.85m时的弯道。途中曲率上限现在独立为5/m，末段仍保持原2/m限制；角速度、15cm偏离和目标方向保护继续有效。
- 最终检查充电确认及整个工作流完成，随后至少观察5秒，确认无再次运动。

需要看基础运行状态时，在另一个已source的终端选择一个话题观察即可：

```bash
ros2 topic echo /task_controller/status
ros2 topic echo /arm_joint_postures/status
ros2 topic echo /line_follow/status
```

## TCP通信检查

树莓派默认监听 `0.0.0.0:9010`；网络示例为工控机 `169.254.199.120`、树莓派 `169.254.199.100`。
可先在树莓派检查监听：

```bash
ss -ltnp 'sport = :9010'
```

工控机使用已有工具目录的 `start_task.py` 查询基础任务状态：

```bash
python3 start_task.py --host 169.254.199.100 --action status
```

应收到成功回包，其中 `robot_status.status_monitor_enabled` 为false，
`normal_status` / `abnormal_status` 为null；基础 `task`、`line_follow` 和底盘电量数据仍按实际反馈提供。
此时不要运行额外的 `monitor_status.py`；专门的normal/abnormal查询会明确返回“未启用”，而非伪造正常状态。

本轮自动开始，工控机只查询即可。若要先联机检查、再由工控机开始，将启动命令中的
`remote_control:=false` 改成true；随后只发送一次：

```bash
python3 start_task.py --host 169.254.199.100 --action start
```

该参数只改变首次开始方式；到站后的过渡、等待、回行车位及返程仍自动执行。

## 控制器加载失败导致整套退出

2026-11-08排查中，在本机驱动关闭后，局域网发现仍返回3个
`/controller_manager/load_controller`服务端，且出现`gz_ros2_control`及
`jaka_zu3_controller`的其他机器人服务；限制本机发现后同名服务端数为0。
恢复独立spawner后，仅切换UDP仍失败，而限制本机发现的实车日志显示
8个spawner均正常退出、6个活动控制器全部激活，随后由用户Ctrl+C结束。
这些结果支持局域网同名服务干扰的判断；尚未定位其他服务端的具体机器。
因此默认采用上述本机发现配置，保留显式跨机器ROS配置入口。

若先出现`Failed loading controller joint_state_broadcaster`，随后所有节点收到SIGINT，
表示机械臂驱动的控制器启动失败，启动保护主动终止整体任务；不是循迹到站或路线录制完成。
该返回值不能单凭调度延迟就归因CPU不足，也不能按“缺包”处理，须检查此前管理器的具体报错。

联合入口的`stations`与`return_charge`共用`internal/robot_system.launch.py`及
`eli_cs_robot_driver/elite_control.launch.py`，不分别维护机械臂驱动。
本轮按提交`66f897c`及2026-11-06整体联动的成功日志，恢复8个独立spawner同时启动；
不再使用批量spawner或通过退出事件逐个串行启动。实际加载先后由官方spawner的锁决定。
launch先声明scaled轨迹控制器，再声明joint_state、IO、speed_scaling、force_torque、TCP，
以及保持inactive的备用forward_position和freedrive。任一spawner失败仍终止整套启动。
默认info级别时恢复原来的controller_manager命令行，不附加日志参数；显式debug时才附加。
这不改变已有任务的机械臂硬件插件、控制周期或到位检查。
联合入口默认不提前启动循迹相机，机械臂就绪门控与检测任务一样检查完整控制器状态；
无反馈或查询取消均保持未就绪，不触发运动。发生加载失败仍终止，不自动重试控制器命令。

用户原有总任务命令`ros2 launch robot_task_control line_follow_arm_test.launch.py launch_rviz:=true`
继续使用相同共用启动链；往返测试仅选择`test_profile:=return_charge`并保留相应轻量组件配置。

需要定位时，在上面的launch命令、管道之前加`controller_manager_log_level:=debug`。
日志会记录控制器内部加载过程。正常运行可恢复`info`，减少日志负载。
这项调整与静态启动检查不能证明此次实车失败的底层原因已修复，仍需复测。

## 中止与复测记录

普通任务停止可调用：

```bash
ros2 service call /task_controller/stop std_srvs/srv/Trigger '{}'
```

也可在工控机执行 `python3 start_task.py --host 169.254.199.100 --action stop`。
需要立即停止实体设备时使用设备停止/急停。异常后先保留日志、查明原因，再回到实物起点重测；不要在途中反复start。

每轮记录：路线文件名、去程是否居中/摆动、到站位置、过渡位到位时间、5秒等待是否满足、
回行车位确认、原故障弯道是否通过、充电确认及失败状态。保留
`/tmp/arm_chassis_round_trip_test.log` 和对应路线/快照；下一轮换日志名和路线名，便于对比。

流程来源：`robot_task_control/config/line_follow_arm_charge_test.yaml`。
`arm_postures_before_chassis` 自动在两次底盘行驶前插入行车位并等待实际到位。
软件回归检查不代表本整体流程已通过真机验收，最终以以上实车观察结果为准。
