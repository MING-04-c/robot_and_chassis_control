# MASC 蓝线循迹控制

本包将 `ruier_camer_diver` 的观测中线转换为底盘 `linear.x` 和 `angular.z` 指令。
默认前进速度 0.075 m/s（7.5 cm/s），支持在线调速。

## 车头相机与车身参考

以当前画面里轮子的位置为依据：车身位于画面上方，前方朝画面下方。
默认 `image_forward_direction=down`，虚拟车身参考为
`(图宽×0.5, 图高×-0.05)`，先放在画面顶边上方约5%，再按实际轮轴位置校准。
前进速度为正；`drive_direction=reverse` 输出负的 `linear.x`。倒车保持同一相机的 `image_forward_direction=down` 和车身参考，不伪造后置视角。

该位置是可调的图像参考，不是已经标定的底盘中心投影。单张图和可见轮子
不足以精确确定车身中心或地面尺度。控制不在不可见区域外推蓝线。
新方向组合为 `down` + `steering_sign=-1`，有效图像转向符号仍为 +1，
保持此前经过现场修正的左右关系。

控制分别计算：

1. 靠近车身的真实可见中线与车身参考纵轴之间的横向误差，纠正车身偏离。
2. 前进方向反馈用近端实测切线，弯道连续加入前瞻走向；记录路线倒车使用米制里程计路径。

不再直接用识别模块的远处绿色目标点作为车身中心误差。
仅剩远处胶带、近端中线缺失过大时报告 `near_line_missing` 并停车。

## 开源算法比较与本项目选型

对照的是下面两个项目的原始实现；本包没有复制算法源码或引入完整导航栈。

| 思路 | 对本项目的帮助 | 不能直接照搬的地方 |
|---|---|---|
| [Nav2 Jazzy Regulated Pure Pursuit](https://api.nav2.org/nav2-jazzy/html/md_nav2_regulated_pure_pursuit_controller_README.html) | 连续选择前瞻点、按运行情况调整前瞻，并约束速度 | 它处理机器人坐标系下的米制路径，带TF、costmap和碰撞检查；本相机仅有透视像素，不应把像素曲率当1/m使用 |
| [PythonRobotics Stanley](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/stanley_control/stanley_control.py) | 把方向误差与横向误差分开，避免仅追回图像中心 | 原实现是自行车模型的转角控制，横向误差为米且用车速归一；这里输出的是底盘角速度，不能直接代入像素或照搬倒车符号 |

选用：**Stanley式分项方向反馈＋连续自适应图像前瞻**。这是工程适配，并非完整Stanley或Nav2 RPP复现；选择依据是已有输入接口和计算量，没有实车对照成绩，不能声称必然优于所有算法。

- 前进时，固定预览范围识别线方向变化，再将实际跟踪前瞻从图高0.45连续缩到0.40，直线保留较远预览。目标在实测行之间插值；只在两端已验证的缺口内续接，不向无观测区域外推。
- 横向死区、弯道回中降权与方向增益继续生效；位置反馈用有界的 `atan(中心增益×死区外误差)`，方向反馈单独叠加，再执行低通和角加速度约束。
- 记录路线倒车：在米制路径上按弧长选取车后目标，计算 `curvature=2*y/(x*x+y*y)`，用负线速度计算角速度；独立限制曲率，不使用相机像素转向反号。
- 相机丢线、输入过期、急停及底盘禁止运动，仍立即零速。控制计算复用同一节点，不新增预览、点云或后台优化器。

## 前进记录，倒车沿原路线返回

默认 `reverse_mode=recorded_route`。前进保持视觉循迹算法，同时从 `/wheel_odom` 记录实际位姿；倒车改用记录路线 Pure Pursuit 转向，并默认叠加前置相机的横线停车保险。无需新增常驻进程或完整导航栈。

先前进一次：

```bash
ros2 launch robot_task_control line_follow_only.launch.py drive_direction:=forward line_show_window:=true
```

到站锁停并实际停稳，看到“前进路线已停稳保存”后关闭旧 launch，保持小车位置和朝向不变，再倒车：

```bash
ros2 launch robot_task_control line_follow_only.launch.py drive_direction:=reverse line_show_window:=true
```

前进每隔约2cm记录一个里程计点，并保存准确终点；默认路线文件 `~/.ros/line_follow/last_route.json`，每秒至多保存一次草稿，结束/退出时再保存；只有最新里程计有效且已停稳的结束记录才标记为可返回，行驶中的草稿不可作为返回路线。新前进任务先撤销旧记录；不足10cm、记录中坐标跳变或保存失败时不能作为有效返回路线。文件用临时文件原子替换，避免写到一半被读取。`route_file:=/绝对路径/route.json` 可在两个命令中指定同一文件。

底盘驱动随 launch 重启时里程计可能归零。倒车启用时把记录终点及其朝向对齐到当前静止里程计位姿，只对齐一次；因此必须从前进停车的实际终点、原朝向开始，中间不要搬车或手动挪动。软件不能仅凭归零后的里程计验证车是否被搬过。运行中再出现里程计重置、坐标系切换、时间倒退或不连续位姿时停车，不自动重新对齐。

记录路线倒车默认7.5cm/s、上限8cm/s；旧camera模式仍限制4cm/s。目标在车后路径上按弧长连续插值；角速度由带符号线速度乘曲率计算，曲率默认不超过2/m（最小指令半径0.5m，需结合实际底盘与轨迹验证）。起步及减速沿用线/角速度协调限制。默认剩余60cm内连续确认可纠偏小弯/直线且转向速率稳定后进入视觉阶段，未满足时最后30cm减速对齐，不按距离强制接管，以最多3cm/s对线并找横线；该阶段由相机横线到位结束，里程计到起点不会冒充正常完成。关闭 `route_visual_finish` 时才沿用原来的最后10cm减速、里程计2cm容差停车。这里的2cm是里程计判定容差，不是实车返回精度保证；轮滑及里程计比例误差会累积。

默认参数位于 `config/line_follow.yaml`：`route_lookahead=0.20`m、`route_max_speed=0.08`m/s、`route_approach_distance=0.10`m、`route_max_curvature=2.0`/m、`route_max_tracking_error=0.15`m、`route_goal_tolerance=0.02`m、`route_odom_timeout=0.5`s。偏离路线超限、目标不在车后、底盘故障或里程计失效均输出零速；偏离/目标方向异常锁定，需停止任务检查后重新启动。前进记录也要求新鲜里程计。缺失/损坏文件会在就绪检查明确报告，不退回旧相机倒车控制。

倒车期间前段相机不参与转向，默认 `route_camera_stop=true` 时参与途中横线停车；最后视觉阶段则使用图像辅助小幅纠偏。只配对同一源时间戳和坐标系的中线与横线掩膜，最多10Hz检查，复用多帧平均和到位锁停。出发时已在停车区或行驶后侧的原横线按原离站规则去重；前段新横线作为途中暂停；进入视觉阶段且剩余不超过30cm后，新横线才作为终点。不凭前置视野中的尾线缺失认定最终终点。相机新鲜、没有活动横线时仍可按记录路线行驶；主线短暂识别不到本身不宣告返程完成。新横线靠近画面参考线时减速，到位后输出零速并锁定，横线漏检不会自动继续；已跟踪横线失踪按既有停止规则处理。相机数据过期或缺失时暂停，恢复新鲜配对帧后未锁停的任务可继续。默认末段改为相机找横线终停；关闭视觉末段时，记录起点仍是最终停车条件。状态为 `reverse_route_tracking` / `reverse_route_complete`；`/line_follow/status` 的 `route` 包含剩余弧长、横向误差与车后目标。`/line_follow/recorded_path` 发布对齐后的 `nav_msgs/Path`，可在 RViz 中以当前里程计坐标系查看；相机窗口显示返回剩余距离、横线停车参考线及候选交点。

相机不是倒车障碍物传感器；此功能也没有新增车后障碍物检测，已有底盘障碍/故障保护继续有效。验证覆盖直线、曲线及S线的简化运动模型收敛、闭合路径不提前完成、路线原子保存、里程计重启对齐、图像丢线、里程计超时与坐标跳变、任务完成衔接。未执行实车测试。

旧的前置相机倒车仅通过显式 `reverse_mode:=camera` 保留给回归测试，不作为推荐运行方式；`reverse_steering_sign` 仅作用于这个旧模式，新末段横向标定独立使用 `route_finish_lateral_sign=1`，对前段里程计路径追踪不生效。前进参数不受它影响。

## 编译与启动

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select ruier_camer_diver masc_chassis_user_control
source install/setup.bash
```

停止之前的底盘、TCP launch 和其它 `/cmd_vel` 控制器，再启动：

```bash
ros2 launch masc_chassis_user_control line_follow.launch.py \
  show_window:=true forward_speed:=0.075 image_forward_direction:=down
```

一条命令包含 USB 相机、蓝线识别、底盘驱动和循迹控制。
距离任务 PID 显式关闭，避免两个控制器同时发速度。CAN 接口由系统 CAN 服务配置。
默认 enabled=true：连续有效中线与底盘反馈到齐后自动运动。
不启动海康和 TCP 服务。

首次校准时先保持静止：

```bash
ros2 launch masc_chassis_user_control line_follow.launch.py \
  show_window:=true enabled:=false image_forward_direction:=down
```

确认虚拟参考和转向方向后启用；暂停同样使用此服务：

```bash
ros2 service call /line_follow_controller/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /line_follow_controller/enable std_srvs/srv/SetBool '{data: false}'
```

只查看输出、不启动底盘：

```bash
ros2 launch masc_chassis_user_control line_follow.launch.py dry_run:=true show_window:=true
ros2 topic echo /line_follow/cmd_vel
```

`dry_run` 强制速度输出到 `/line_follow/cmd_vel`，不发到 `/cmd_vel`，并跳过底盘状态检查。
已有相机和识别节点时用 `start_camera:=false`，其默认话题须为 `/camera/line/*`。

## 可视化和参考校准

`show_window=true` 打开控制器标注后的四格画面（相机识别节点不另外开重复窗口）。
标注与实际控制一致：

- 紫红色竖线/十字：虚拟车身参考，参考点在画面外时十字贴在相应边缘显示。
- 青色圆圈：靠近车身的真实中线点，用于横向纠偏。
- 灰色 GROUND CHECK START 横线：近端地面检查起点，默认跳过轮子侧画面25%的区域。
- 灰色 FRONT 箭头：当前画面里的车头前进轴。
- 绿色 MOTION CMD 箭头：按当前 vx/wz 指令绘制的前进和转弯趋势，仅在有前进指令时显示。
- 橙色 GOAL 圆圈/箭头：追踪目标，不表示轮子的转角或最终速度方向。
- 青色横段：近端点与车身参考纵轴之间的位置偏差；该偏差也参与转向。
- 原有绿色点：识别模块的检测目标，保留作识别诊断，控制器不直接追它。

`/line_follow/debug_image` 最多以5 Hz发布相同画面，减少预览对控制周期的影响，可通过兼容 Best effort 的查看器显示。
按 q/Esc 暂停循迹并关闭控制窗口；Ctrl+C 退出整个 launch。
控制窗口不提供识别节点原有的鼠标颜色取样功能；颜色取样时单独启动
`ros2 launch ruier_camer_diver line_tracking.launch.py show_window:=true`，并停止循迹总 launch。

把底盘中心线实际对准直胶带、车头与胶带平行，然后观察近端青色点与紫红色竖线。
如果相机有左右安装偏移，修改 `vehicle_reference_x_ratio` 使两者重合。
例如近端点在 x=350、处理图宽640时，比例可从 `350/639≈0.548` 开始：

```bash
ros2 param set /line_follow_controller vehicle_reference_x_ratio 0.548
```

纵向参考与前瞻距离也可调整：

```bash
ros2 param set /line_follow_controller vehicle_reference_y_ratio -0.05
ros2 param set /line_follow_controller lookahead_distance_ratio 0.45
```

这些值是图像比例，不能当成米。不能仅凭当前图像精确确定不可见车身中心的位置。

## 速度和控制参数

```bash
ros2 param set /line_follow_controller forward_speed 0.08
# 0.08 m/s = 8 cm/s；0.0 停车
ros2 param set /line_follow_controller forward_speed 0.0
```

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `forward_speed` | 0.075 | m/s，前进速度 |
| `max_forward_speed` | 0.30 | m/s，参数上限 |
| `image_forward_direction` | down | 从车身参考出发选择前方中线；不表示光流方向 |
| `vehicle_reference_x_ratio` | 0.5 | 虚拟车身参考横坐标/图宽 |
| `vehicle_reference_y_ratio` | -0.05 | 虚拟车身参考纵坐标/图高 |
| `lookahead_distance_ratio` | 0.45 | 从参考点向前的图像距离/图高 |
| `near_edge_margin_ratio` | 0.25 | 从车身侧边缘向前跳过轮子遮挡区的图高比例 |
| `max_near_gap_ratio` | 0.40 | 从可见地面起点计算缺线距离，超过此图高比例停车 |
| `steering_gain` | 0.65 | 近端横向位置纠偏比例 |
| `heading_gain` | 0.35 | 从车身参考指向前瞻点的方向纠偏比例 |
| `steering_sign` | -1.0 | 当前 down 方向下保持有效转向符号 +1；与方向共同决定转向 |
| `max_angular_speed` | 0.25 | rad/s，直线温和纠偏上限；弯道使用 curve_max_angular_speed |
| `error_deadband` | 0.04 | 归一化近端偏差连续死区 |
| `heading_deadband_rad` | 0.05 | 方向死区，约2.9度 |
| `steering_filter_time_sec` | 0.16 | 转向平滑时间常数，秒；0关闭 |
| `angular_acceleration` | 0.35 | rad/s²，直线转向变化上限；弯道使用 curve_angular_acceleration |
| `linear_acceleration` | 0.08 | m/s²，正常加减速限制 |
| `require_bottom_connected` | false | 可见中线不必贴到底边 |

左右方向已按现场反馈校正；当前 down 配 -1.0 保持校正后的有效映射。
该符号同时反转位置纠偏和前瞻转向，前进速度保持不变。
运行中修正方向前先暂停，再设置：

```bash
ros2 service call /line_follow_controller/enable std_srvs/srv/SetBool '{data: false}'
ros2 param set /line_follow_controller steering_sign -1.0
```

横向漂移明显时先校准参考横坐标，随后调位置纠偏比例；不要只把前进速度调高。
正常加速有上限，丢线或禁止运动时直接输出零，实际停车仍受惯性影响。
弯道、疑似遮挡和前瞻回退时会减速。在线修改不会自动保存，长期使用修改
`config/line_follow.yaml` 并重新构建。

如果另一种安装的车身位于画面下方、前方朝上，需同步设置
`image_forward_direction:=up vehicle_reference_y_ratio:=1.05`，并重新检查左右映射。
车身参考和前方方向矛盾时会报告 `no_forward_line`，不继续运动。

绿色运动轨迹由当前速度指令积分生成，并使用固定像素示意比例；它不是已标定的
真实地面轨迹或轮子角度。底盘实际运动仍应结合轮速/里程计反馈观察。

## 数据与停车条件

```text
相机 image_raw -> ruier_camer_diver 蓝线识别
  -> line/status + line/centerline（观测点，像素坐标）
  -> 近端中线纠偏 + 车身参考到前瞻目标的转向
  -> cmd_vel 50 Hz -> 底盘驱动 -> CAN -> 底盘
```

JSON 状态用于检查识别有效性、图像尺寸及颜色证据；中线 PolygonStamped 用于
计算实际控制几何，不使用估计中线。状态、路径和原始图像时间戳均须新鲜。
丢线、超时、前后跨度不足的横线、近端线缺失、没有前方路径、CAN 未连接、底盘故障/急停/
遥控模式、障碍或充电状态时停车。默认允许通信控制和驻车状态，驱动自动解锁。
横向扫描标签不再直接等同于禁止运动：弯线必须同时满足真实中线前后跨度至少图高15%、
近端可见、前瞻方向有效、方向误差在限制内才能通过。纯横线仍停车。
不具备交叉口路线选择、导航定位或障碍物规划能力。

```bash
ros2 topic echo /line_follow/status
ros2 topic echo /cmd_vel
ros2 topic echo /camera/line/status
ros2 topic echo --once /masc_chassis/chassis_status
```

`/line_follow/status` 的 `geometry` 提供参考、近端、前瞻坐标和实际控制误差。

## 左右摆动与舵轮反复调整

参考 [Nav2 Velocity Smoother](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother/) 的速度死区、加速度约束与比例协调思路，控制节点做了以下适配，没有新增平滑器进程：

1. 直线方向项用实测切线，避免横向位置已由中心项纠正后，又在“指向偏移前瞻点”的朝向项重复纠正。弯道仍按实际线走向控制，不强制立即追回画面中心。
2. 用配置速度将原始角速度需求换成转弯曲率，对其做迟滞和低通，再与实际目标线速度相乘。视觉减速、起步和横线接近时，角速度同步调整，避免线速度下降而角速度不变、轮向突然打大。
3. `steering_curvature_enter=0.30`、`steering_curvature_exit=0.15`，单位1/m；这是运动命令的曲率门槛，不是舵机角度。进入与退出采用不同门槛，小噪声不反复启动转向。恢复直行后保留上次方向记忆，但不会继续施加旧转向。
4. 轻微反方向需 `steering_reverse_frames=3` 张新源图确认，重复发布同一帧不累计。绝对曲率超过 `steering_fast_reverse_curvature=1.5` 时缩为两张新图；在20 FPS下分别约100ms和50ms，仍取决于实际图像到达时间。确认期间向直行衰减，不继续加大旧转向；明确弯道不会被永久锁住。
5. 按线/角加速度所需的最小共同插值比例推进，尽量保持转弯比例。停车横线的限速需要立刻生效时，同步缩小两项；真正停车、丢线、急停直接零速，不等待滤波结束。

[WPILib 舵轮优化](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html) 在模块层根据当前角度优化目标轮速/角度，减少转向行程。当前SDK发送整车速度，ROS层没有每轮独立目标控制接口，因此没有冒充实现底层舵机优化。驱动的已有低通用于自动驻车判断，并不平滑正常运动命令。如果上层命令已稳定但轮子仍反复摆动，需要结合 `/masc_chassis/servo_motors` 反馈检查下位机角度控制。

`/line_follow/status` 新增 `raw_yaw_target_radps`、`steering_guard`、`raw_curvature_per_m` 和 `filtered_curvature_per_m`。移动时每5秒自动打印一行 `Steering: raw_w=...; cmd_v=...; cmd_w=...; curvature=...; guard=...`，便于区分视觉需求频繁反号、确认器抑制反号，或命令稳定后仍存在的轮子问题。无需另外开节点。

模拟检查包含交替方向噪声、零值间隔后的反号、同一帧不能重复确认、真实反向弯响应、前后进的协调加减速、停车及简化平面模型收敛。没有实测轮子角度，不能据此认定硬件抖动已完全消除。

## 同帧控制与短暂中断恢复

识别器在JSON中附带源图 `source_stamp_sec/source_stamp_nanosec`。控制器以此时间戳配对状态、中线和停车掩膜，三个通道到齐才提交一帧；不使用新掩膜配上一帧的中线。最多缓存3帧ROS消息，不复制整幅图，不累计重复帧；旧帧或源图过期仍停车。旧版不带时间戳的状态保留兼容路径，`frame_synchronized=false` 可直接看出未启用配对。

弯道短暂出现有效行不足、前方跨度不足或无连续可信段时，该帧仍立即零速。如果已正常跟踪，且0.25秒内有效同帧观测恢复，第一张恢复帧即可恢复跟踪，不额外再等三帧，转弯强度过渡也保留这段短时记忆。中断期间不输出历史运动命令；真正丢线、空中线、急停和断联仍走严格恢复。初始启动仍要求连续三张有效新帧。

`/line_follow/status` 提供 `frame_synchronized` 和 `source_stamp_ns`。跨度不足的日志增加当前点数、原始纵向跨度和是否同帧配对，供继续定位真实识别问题。停车已经确认时，无论输入怎样变化，任务状态保持已到站；输入诊断与任务状态分开显示。

## 转向平滑与手动拖动

默认对近端横向误差和方向误差分别使用连续死区，再对转弯曲率做
0.16秒时间常数的低通平滑。线速度与角速度共享插值比例，分别满足0.08 m/s²和直线0.35 / 弯道0.50 rad/s²的加速度约束；停车与硬限速优先。
持续偏离仍会纠偏；细小识别波动不再立即触发明显转向。
`cmd_vel` 保持50 Hz，不通过降低发送频率来抑制抖动。
丢线、禁用、超时和底盘状态禁止运动时绕过平滑，立即输出零指令。
这些参数控制车身角速度，不是直接指定各个舵轮角度。

在线设置，不需重启：

```bash
ros2 param set /line_follow_controller steering_filter_time_sec 0.16
ros2 param set /line_follow_controller angular_acceleration 0.35
ros2 param set /line_follow_controller error_deadband 0.04
ros2 param set /line_follow_controller heading_deadband_rad 0.05
```

需要更平滑可将时间常数小幅提高到0.3；若弯道反应变慢或偏离增大，调回0.15～0.2，
并降低前进速度。不要靠无限扩大死区来压住转向，否则会允许持续偏离。
没有实车几何标定，不能保证所有弯道都无需进一步调参。

暂停循迹只输出零速度，不代表电机失能。底盘协议提供拖车模式
`/masc_chassis/set_tow_mode`，应通过它请求允许手动拖动。
保持底盘驱动运行，并暂停所有可能发送非零运动指令的上层控制器。
在平地扶住车，先暂停循迹、等实际停稳，再进入拖车模式：

```bash
ros2 service call /line_follow_controller/enable std_srvs/srv/SetBool '{data: false}'
# 确认实际停稳后执行下一条：
ros2 service call /masc_chassis/set_tow_mode masc_chassis_msgs/srv/SetTowMode '{enable: true}'
ros2 topic echo --once /masc_chassis/chassis_status
```

状态中 `chassis_state: 2` 表示 `CHASSIS_STATE_TOW`。
服务返回 `success: true` 仅说明指令发送成功，必须结合此反馈确认底盘确实进入拖车状态。
`brake_active` 可辅助检查抱闸反馈；若仍抱闸或推不动，不要硬拖。
舵向电机是否也释放由底盘固件决定，当前接口没有单独的舵向失能指令。

拖动结束后先退出，再确认状态恢复，最后按需要重新开启循迹：

```bash
ros2 service call /masc_chassis/set_tow_mode masc_chassis_msgs/srv/SetTowMode '{enable: false}'
ros2 topic echo --once /masc_chassis/chassis_status
# 确认退出拖车状态、周围可运动后，再开启：
ros2 service call /line_follow_controller/enable std_srvs/srv/SetBool '{data: true}'
```

## 按识别质量自动补偿曝光

`line_follow.launch.py` 默认启动 `line_exposure_node.py`。它订阅原始图像和蓝线
识别状态，通过 `v4l2-ctl` 调节同一个 USB 相机，不额外打开采集流。
相机置于手动曝光模式，由程序反馈调节，避免相机内置自动曝光与程序互相干扰。
需要系统安装 `v4l-utils`；不支持曝光控制时会报告 `hardware_error` 并停止调节。

先暂停底盘观察曝光效果：

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch masc_chassis_user_control line_follow.launch.py \
  show_window:=true enabled:=false forward_speed:=0.03 \
  image_forward_direction:=down adaptive_exposure:=true
```

`enabled:=false` 仅禁止底盘循迹，自动曝光仍工作。窗口顶部显示 `EXP=曝光值`。
确认识别和前进方向正确后，在另一个终端加载 ROS 环境并启用循迹：

```bash
source /opt/ros/jazzy/setup.bash
source ~/project/robot_and_chassis_control/ros_ws/install/setup.bash
ros2 service call /line_follow_controller/enable std_srvs/srv/SetBool '{data: true}'
```

策略配置位于 `config/adaptive_exposure.yaml`：

- 连续三次识别不确定，且图像有偏暗或过亮证据才调节；稳定时保持曝光。
- 默认限制为相机原生曝光值 20–200，每步约15%，最小5，至少间隔1.5秒。
- 持续不确定时最多调整8次；重新稳定后才能开始新一轮补偿。
- 保留当前曝光作为起点；超出配置范围才限制到范围内。
- 输入过期、处理超时或相机控制失败时停止补偿。

这些曝光值是 V4L2 控件整数，不能直接当成微秒；实际单位与设备控件定义有关。
调整曝光上限可能影响实际采集帧率。修改配置后需重新构建并重启 launch。

状态查看与临时冻结：

```bash
ros2 topic echo /line_follow/exposure_status
ros2 param set /line_exposure_controller adaptive_enabled false
ros2 param set /line_exposure_controller adaptive_enabled true
```

冻结或退出时保留最后曝光值，不自动恢复相机内置自动曝光。
启动时完全关闭此程序用 `adaptive_exposure:=false`，此时可自行手动设置曝光。
自动调节运行期间不要同时用另一个程序设置相机曝光。
如相机不在 `/dev/video0`，优先修改相机配置里的 `video_device`，
也可用 `exposure_device:=/dev/videoN` 指定曝光控制设备，但必须与实际采集设备一致。

曝光补偿不能恢复画面外的胶带或完全失去颜色的反光；明暗正常但识别丢失时不会
盲目增加曝光。局部反光与大片阴影同时出现，也可能需要调整照明或相机角度。
它不绕过近端缺线等停车条件：`near_line_missing` 仍表示车身附近缺少可用中线。

## 测试

```bash
colcon test --packages-select masc_chassis_user_control
colcon test-result --verbose
```

测试覆盖方向、车身参考校准、近端线缺失停车、远处检测目标与近端位置不一致时的
纠偏，以及 ROS 断流/急停/断连/启停/在线调速。测试速度仅输出到隔离测试话题。
模拟测试通过不代表真实车身中心已标定，实车需低速检查安装与控制增益。

## 键盘控制小车

先关闭循迹、TCP任务等其他运动控制器及其底盘驱动，在交互终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/project/robot_and_chassis_control/ros_ws/install/setup.bash
ros2 launch masc_chassis_user_control keyboard_teleop.launch.py linear_speed:=0.05 angular_speed:=0.2
```

默认一起启动底盘驱动，不启动相机或距离PID；`linear_speed` 单位m/s，
0.05表示5cm/s；`angular_speed` 单位rad/s。
已有独立驱动时加 `start_driver:=false`，不要重复启动驱动。
该驱动须正常启用运动帧发送；拖车专用的 `command.transmit_enabled=false` 配置无法控制运动。
等待 `KEYBOARD READY` 后，在启动该命令的终端内操作：

| 按键 | 操作 |
| --- | --- |
| W / S | 前进 / 后退 |
| A / D | 向左 / 向右横移，速度与前进相同 |
| Q / E | 原地左转 / 右转 |
| Z / C | 后退并左转 / 右转（左右指车身旋转方向） |
| 空格 / X | 立即发送停车指令 |
| + / - | 平移和旋转速度同时增减约10%，并停车等待下一个移动键 |
| Ctrl+C | 发送停车指令并退出 |

终端无法直接感知松键，默认0.6秒没有新的移动按键就发零速度。
长按依靠操作系统的键盘连发；首次连发前可能短暂停车。
等待时间可通过 `key_timeout` 调整；默认0.6秒兼容约0.5秒的首次按键连发延迟，松键后最多等待0.6秒才发停车指令，空格/X仍立即发停车指令。
方向键不支持。任何未知按键会停车；速度保持限制在0.01～0.3m/s、0.05～0.8rad/s。
退出会恢复终端设置。Ctrl+C退出时发送零指令，实际停稳仍受底盘滤波和惯性影响。

键盘节点输出到 `/cmd_vel`，驱动负责CAN发送；CAN接口仍需先配置好。
键盘控制属于速度控制，不会让电机失能。没有交互终端时会报错并退出。

## 本机机械臂与底盘统一工作区

统一任务入口、USB `/line_camera/*` 话题映射和三种独立流程见 `robot_task_control/README.md`。统一工作流中的循迹默认关闭，由任务控制器开启；本文件独立 line_follow.launch.py 默认自动循迹。键盘也可通过 `ros2 launch robot_task_control keyboard_teleop.launch.py` 启动。请先退出其他底盘启动会话，再启动键盘控制。
## 连续可信段循迹

循迹每帧从车身近端开始，仅使用第一段连续实测中线，纵向缺口超过
`max_trusted_gap_ratio`（默认图高 0.06）时检查缺口两端。只有两端各有至少图高10%的
连续实测支撑、切向角差不超过0.25 rad、双向切线位置偏差不超过图宽4%，
且前方累计缺口不超过 `max_bridge_gap_ratio`（默认图高0.30），才允许作为同一条线续接。
若切线不满足同向对齐，但延长线在两端之间的缺口内部、图像范围内相交，
也允许按同一条有转角的线低速续接；转角上限 `bridge_intersection_angle_rad` 为0.85 rad（约49°）。
仅在缺口以外相交不放行。上游视觉 YAML 的 extended_join_* 同时保留对应两段实测候选，
避免第二段在进入控制器前被单候选筛选删除。
几何验证通过后默认 `bridge_speed_factor=1.0`，按同一条线跟随，不额外乘缺口系数或上游疑似遮挡系数。前瞻点落在已确认缺口内部时，在两端之间插值控制目标，避免回退点跳变；插值不写入实测蓝线掩膜。橙线和 GAP: JOINED 表示已确认续接。
两端不吻合、仅有单端或缺口过大则截断。把 max_bridge_gap_ratio 设为0可关闭缺口通过。
两端方向改用实测支撑点的中位数成对斜率和拟合位置，降低单个端点毛刺的影响；
上游已加入局部明度补偿和按实测点数自适应的霍夫线段拟合，规则与参数见
工作区 `docs/chassis/LINE_FOLLOW_REFERENCE_20261029.md`。
前瞻落在已接受的范围内，段变短时减速，不足最小跨度或有效比例时发零速度。
近端跨度门槛 `min_forward_span_ratio` 为0.05，有效比例门槛 `min_valid_row_fraction`
为0.06，允许尚有短实测线段时继续低速推进。两侧支撑长度通过
`bridge_support_span_ratio` 配置，默认0.10；这些比例不是概率或米制距离。
新画面中线变清楚、实测段变长，前瞻范围随之延伸；不沿缓存路线盲行。
曝光节点与控制使能独立，停车后仍可继续调整曝光。
预览红叉为实测段末端。阈值是图像比例，不是标定后的地面停车距离。

`robot_task_control/line_follow_only.launch.py` 使用 `until_untrusted: true`，
开始后跟随可信段，不再按2秒结束；视觉不足先零速等待5秒，恢复后继续，未恢复才结束本轮，
结束后需再次 start。预览及 `/line_follow/status` 同时显示 input_state（当前输入是否可走）
和 last_blocked_reason（上次阻塞原因），禁用后仍保留。INPUT: tracking 且 CONTROL: disabled
表示输入已恢复，但任务未使能；不代表此刻图像仍不可行。
默认单轮总超时 90 秒。机械臂联合入口仍保留原来的 2 秒测试时长。

为减轻直线 S 形摆动，当前采用较宽的回中死区（横向归一化0.04、方向0.05 rad）和较低纠偏增益（位置0.65、方向0.35）；最大转速0.25 rad/s，角加速度0.35 rad/s²。正常跟线的线速度变化率0.08 m/s²，偏差减速系数0.4。未确认区域仍保留置信度减速；已通过两端几何验证的缺口默认系数1.0，并清除该续接路径上的重复疑似遮挡减速；丢线、故障和数据失效仍立即停车。树莓派实车效果需要沿现有轨迹再验证，弯道跟不上时应结合画面状态调整。

## 弯道优先跟随走向

当前可信线段的实际走向偏离车身前方约7°，或近端切线与前瞻走向相差约7°后，逐渐启用转弯优先控制；约14°时达到完整强度。近端拟合使用实测点的稳健斜率，不把整条直线仅向左/右平移误当成弯道；出弯后车身尚未对准倾斜直线时，仍优先跟随其走向。`geometry.bend_angle_rad`、`curve_strength`、`path_heading_rad` 可在 `/line_follow/status` 查看，预览标题显示 CURVE。图像角度不是已标定的地面转角。

弯道时中心容差由半图宽4%逐渐放到30%（640宽约96像素），中心纠偏力度降到原来的20%；方向增益从0.35逐渐提高至0.70，并优先使用近端到前瞻点的路径走向。转速上限由0.25过渡到0.34 rad/s（约19.5°/秒），弯道角加速度限制0.50 rad/s²。这样允许沿弯道有一定偏离，避免往中线拉的动作抵消转弯；直线保持原来的柔和参数。中心容差内的弯道偏差不再额外引起同等程度减速，正常前进设定仍为0.075 m/s。

相关参数在 `config/line_follow.yaml` 的 `curve_*` 项。模式按0～1强度连续过渡，并以0.20秒时间常数渐进进入，减少第一次转向过大，仍经过原来的转向滤波；单端/不合格缺口、丢线、过期、底盘故障与紧急停止规则不放宽。

## 90°停止横线

启用 `stop_marker_enabled` 后同时订阅 `/camera/line/stop_marker_mask`，用实测蓝线掩膜检测与局部主线垂直、两侧均有横向延伸的停止条，普通 L 形拐弯不算站点。横线后沿 `image_forward_direction` 仍有主线为 intermediate，没有继续线且视野足够为 final；证据含糊或标记仍在画面边缘时为 unknown。横线类别连续2张有效新图像确认，不用重复旧帧累计。

接近目标时从图高25%的距离范围内开始减速，最低接近速度0.008 m/s。接近和停车都使用同一横线最近3张新图像的算术平均位置；超过1秒的样本剔除，缺失帧不按零误差计入。误差按前进方向定义：正值表示尚未到达，负值表示已经越过。

尚未确认到站时，平均误差大于图高3%就继续低速前进；平均误差位于图高−6%～+3%时直接零速，确认2次有效新平均值在此范围内即到达。轻微越线默认算到达，不输出倒车命令；越过超过图高6%则停车报告 overshoot。480高图像对应允许越过28.8像素、尚未到达容差14.4像素，属于图像位置要求，不能换算为固定厘米，也不能保证底盘零像素误差。

到站确认之前，接近容差区的短暂漏检仍输出零速、禁止继续消息解锁，但保留有效期内的到位计数；下一有效帧仍在容差内即可继续确认。漏检帧不算到位帧，旧样本过期或位置离开容差后重新确认。

确认到站后锁定 `arrived_latched=true`，状态保持途中站/终点；后续横线漏检、平均位置变化或图像超时不会撤销到站或自动重新行走。当前漏检由 `marker_currently_missing` 报告，输入过期/底盘故障由 `input_state` 报告；始终保持零速。途中站只有收到新的继续消息，且当前输入有效、横线可见时才解锁；最终站不接受继续。

停车目标由 `stop_marker_target_ratio` 设置，基础配置为图高50%，单独循迹任务默认46%。`stop_marker.error_ratio` 为平均误差，`raw_error_ratio` 为当前帧误差，`sample_count` 为有效平均样本数；状态日志同时显示平均/原始像素误差和不对称容差。

途中站到位后等待新的开始消息，提前发送不会留存；最终站不允许这个消息继续。

```bash
ros2 topic pub --once /line_follow/resume std_msgs/msg/Empty '{}'
```

已恢复通过的同一根横线会去重。整体任务只在最终站完成循迹，途中站等待时不启动后续机械臂。`/line_follow/status` 提供 `stop_marker` 对象，以及 stop_marker_approaching/averaging/settling/intermediate/final/unknown/missing/overshoot 状态。标记缺失时按下述短时缓行条件处理，图像过期、底盘故障仍停车；不能用旧横线位置确认到达。尚未确认到站时，停车确认、缺失、越线和类别未知共用5秒确认期限，期间状态切换不重置；无法确认最终站不当作正常完成。

图像中 STOP TARGET 画出停车目标，STOP intermediate/final/unknown 标出横线交点。标记长度、垂直角度容差、两侧最小长度、停车位置和接近限速均可在 YAML 的 `stop_marker_*` 调整。若明确不使用站点标记，可关闭此功能，随后不再要求掩膜流。

弯道连续性余量：近端缺线容差调整为图高40%；识别层逐行中心跳变允许65像素、连续漏检容差6行、中线形状跳变系数2.5。两侧实测线在缺口内相交时，接合转角上限统一为0.85 rad；缺口总长和两侧支撑长度限制仍保留。完全丢线不沿历史方向盲走，停止横线仍以配置的目标比例为停车位置。以上是图像参数，需实车验证。

长停止横线在图像层与循迹主线分离：`line/mask` 显示去除侧翼后的主线，`line/stop_marker_mask` 保留完整真实蓝线供停车判断。开关为相机 `separate_stop_bars`；状态 `separated_stop_bars` 报告本帧分离数量。横线两侧缺少可靠主线支撑时不会盲目切割。转弯前瞻点在相邻实测行间连续插值，减少逐行选点跳变；仅已验证缺口可插值，不向可见范围以外外推。当前不增加横线之后的路径规划。

停止横线定位使用两侧实体像素按列取中心并稳健拟合，与主线参考轴求交，不直接采用某一条 Canny 边缘。发出到位零速后固定主线参考轴，横线位置仍从每张新图像重测，因此主线提取抖动不会直接改变停车原点，真实横线位移仍可报告。停车后已确认的途中/终点类别锁存，避免继续线短暂反光改变类别。

### 减少误减速与短暂停顿

控制选段先排除 `near_edge_margin_ratio` 指定的车轮/车壳遮挡区域。若最前面只有不足以控制的碎点，只有后方连续实测段足够长、间距受限且走向与碎点相容时才改选后段；不跳过本来有效的近段，也不跳到横向偏移的另一根线。可信距离仍由真实线段及已经验证的缺口决定。

可信距离的速度系数按“可见近端到请求前瞻点”的距离计算，不再把车身参考点到可见地面的遮挡距离也算成缺线。当前实测范围覆盖前瞻点或缺口已通过几何验证时，上游 Canny 的疑似遮挡不再重复降速。前瞻确实缺少支撑、明显转弯及接近停止横线时仍使用各自速度约束。这个实现参考了 [Nav2 Jazzy RPP 的速度约束与终点接近减速分离](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp)，具体视觉证据条件由本项目实现。

`stop_marker_dropout_grace_sec` 默认 0.6 秒：同一停止横线已经获得足够的多帧确认、最近平均及单帧误差都大于正向停车容差的两倍，并且主线与底盘输入仍新鲜有效时，横线短暂漏检可维持不超过 `stop_marker_creep_speed`（默认 0.008 m/s）的缓行。状态为 `stop_marker_missing_approaching`，不把漏检帧算成到位证据；按速度指令计算，这段最多约 4.8 mm，并另受 `stop_marker_dropout_max_distance=0.005`m 的同次漏检指令积分限值约束，不代表实车制动精度。超过期限、接近到位区或主线丢失则输出零速。已确认到站仍锁定停车。设为 0 可关闭此余量。

每 5 秒的 `Steering` 日志新增 `trusted_speed`、`occlusion`、`fallback` 字段，用来区分可信距离、疑似遮挡与目标回退导致的减速；正常前进速度和已验证的转向防摆动参数没有调整。

### 直线缓慢回中，弯道允许偏移

[Stanley 原论文](https://robots.stanford.edu/papers/thrun.stanley05.pdf) 将方向误差与横向位置误差分别反馈。本程序继续采用图像中的分项反馈，并针对本项目的防摆动曲率门槛增加持续偏移确认；这不是把像素直接当米，也不是直接输出论文中的车轮转角。

直线中心连续死区 `error_deadband=0.04`，相当于640像素宽图像的约13像素；允许在中心附近行走，不追求每帧零误差。如果中线连续 `straight_center_confirmation_frames=5` 张新图都偏向同一侧，且偏移至少达到直线死区的1.5倍（约19像素），温和回中模式才允许较小转向通过：`straight_center_curvature_enter=0.10`（1/m），退出门槛不大于其一半。重复处理同一源图不累计，左右交替的偏差不算持续偏移。回到直线死区或进入明显弯道（平滑后的曲线强度大于0.25）时撤销该模式；丢线、禁用及停车时随转向控制器复位。前置单相机倒车保留普通防摆动门槛。

弯道仍采用半图宽30%的中心死区，中心纠偏力度仍逐渐降到直线的20%，优先沿线的走向转弯。回中模式不提高速度、不取消反向转向的新帧确认、低通或加速度限制。停车横线容差不受本次直线中心调整影响。上述像素数描述图像参考中心，不代表实车厘米精度。

`Steering` 日志的 `centering` 表示是否确认了直线持续偏移，`offset` 为归一化近端偏差；`/line_follow/status` 也提供 `straight_centering` 与 `straight_center_frames`，便于实车判断是否在缓慢回中。

### 旧相机倒车模式

原相机转向逻辑保留在显式 `reverse_mode=camera` 中；新末段使用分项视觉反馈与记录路线曲率前馈。前段里程计路径追踪不受 `reverse_steering_sign` 影响；相机横线保险沿用出发横线去重规则。当前操作流程以“前进记录，倒车沿原路线返回”一节为准。

### 停止横线诊断

已撤回短横臂两两合并，恢复长线候选检测，减少同步图像回调的工作量。转向滤波、曲率确认、加减速限制和记录路线倒车保持原有设置。反光断开的横线仍可能漏检，尚需现场画面验证。

`/line_follow/status` 的 `stop_detection` 保留候选数量和拒绝原因统计；`joined_segments` 为0。移动时5秒一次的 `Steering` 日志显示 `stop_candidates`、`stop_rejected`。停车容差和到位锁停不变。

### 键盘与循迹共同出现间歇停车时

键盘节点在运动转零速时打印按键超时或停车键原因，循环间隔超过100ms时打印发送延迟；底盘驱动在进入指令超时时打印 `cmd_vel timeout`，恢复时打印 `cmd_vel reception recovered`。底盘指令超时仍为100ms。已移除发送循环内的节点发现查询和无订阅执行器轮询，避免诊断影响发送节奏；发布者冲突可另外用 `ros2 topic info /cmd_vel -v` 查看。独立键盘测试前应关闭旧循迹任务和残留的 `velocity_mux`。

记录路线倒车的速度和终点接近距离参考 [Nav2 Regulated Pure Pursuit](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/) 的独立速度、前瞻、接近减速设置，保留本项目原有轻量追踪器，不新增Nav2节点。参数按当前小车尺度设置，不照搬汽车参数；[Autoware Pure Pursuit](https://autowarefoundation.github.io/autoware_universe/latest/control/autoware_pure_pursuit/) 也单独提供倒车前瞻配置。默认转向前瞻仍20cm，减速范围缩为10cm，默认末段由相机横线终停，里程计只作搜索行程围栏；关闭视觉末段才使用原里程计2cm到位逻辑。每5秒日志显示剩余弧长、距起点距离及指令速度，实际精度及平顺性需实车验证。

### 倒车横线暂停与继续

标准倒车命令默认启用横线保险，无需增加进程。到横线锁停后，沿用既有继续话题（工控机协议动作是 `resume`，不是重新 `start`）：

```bash
ros2 topic pub --once /line_follow/resume std_msgs/msg/Empty '{}'
```

仅途中横线等待继续；最后视觉阶段要求横线到位且对齐确认通过才完成返程。继续必须有新鲜相机/里程计和可用底盘，且当前横线仍可见；出发横线随后去重，避免同一站反复停车。状态 `stop_marker_intermediate` 表示等候继续，`reverse_route_complete` 表示已回到记录起点、返程完成。`/line_follow/status` 还包含 `route_camera_stop`、`stop_marker` 和 `stop_detection`。需要单独回归原里程计模式时，启动参数 `route_camera_stop:=false` 关闭横线保险，此时相机不再作为停车条件。横线必须进入前置相机可见区域才可能检测，此功能不提供后方障碍物检测。

### 前置相机倒车：横移回中、旋转摆正、横线停车

相机安装不随倒车改变：画面上方靠近车头/车轮，下方更远，保持 `image_forward_direction=down`。底盘已确认可左右横移，默认 `route_visual_strafe=true`。前进控制不受此设置影响，不需要新增运行节点。

倒车主体仍跟踪记录在车后的路线。仅在有实测地面映射、记录路径局部曲率小于0.5/m、近端朝向不超过约23度且同帧视觉可靠时，加入不超过0.01m/s的横移修正，角速度仍由记录路径控制。未标定图像模式取消途中横移混入，避免依据车前图像让舵轮不断切换蟹行角。其余弯道按记录路线走，不拿车前远端弯道当成车后轨迹。

最后60cm内，连续3张同帧配对图像确认局部轴可靠、车后记录路径为直线或小弯，实际及指令转速均低于0.06rad/s后，进入末段。近端只取可见地面起点之后图高24%的窗口；剔除停止横线造成的过宽蓝色行，再用中位数初估及内点拟合主线。不用整幅图像或远端弯道计算末段朝向。

末段依次执行“摆正→横移→倒车”，同一时刻只激活一个运动轴：`linear.y` 负责回中，上限0.025m/s；`angular.z` 负责摆正，上限0.06rad/s；`linear.x` 负责倒向停止横线，上限0.03m/s。未对齐时纵向零速，先摆正再横移，状态为 `reverse_visual_finish_aligning`；上层任务不把这个状态当丢线或完成，也不累计纵向行走时长。阶段切换需3张新帧确认，再保持至少0.35秒零速并确认实际停稳；纠偏反向也需3张新帧及停稳间隔。恢复倒车后允许原到位容差1.5倍内的小偏差，避免在0.079/0.087边界反复横移/倒车切换；终点仍用原严格容差。连续确认对齐后再倒向横线。横线到位后只锁住纵向，仍可横移及摆正，直到对齐才报 `reverse_route_complete`。一次连续对齐超过20秒则锁停报 `reverse_finish_alignment_timeout`，避免方向错误时无限调整。

默认没有地面标定，状态明确显示 `visual_geometry_mode=image_uncalibrated`：只按近端图像轴偏移低速伺服，不再外推到画面外的虚拟参考行，不声称得到真实车轮中心的米制偏差。图像到位要求为 `route_finish_goal_lateral=0.08`（半图宽比例，640宽约±26像素）、图像轴角度≤6度；它不等于实际车身偏差2cm或实际偏航6度。到完成容差内停止该纠偏轴，不再追逐微小误差。有效横移速度最低0.012m/s、摆正速度最低0.02rad/s，高于现有驱动0.01的命令零速判定；不会一直发送0.003m/s而底盘已驻车。启动时仍受加速度限制，未修改底盘锁车及命令超时保护。

#### 可选实测地面映射

固定相机、图像分辨率及相机图像处理方式。保持车体静止，在地面取至少4个不共线点，量出它们相对 `base_link` 原点的 `(X前,Y左)` 米制坐标，并记录同一处理画面中的 `(u,v)` 像素。独立留出若干验证点检查映射误差；不要仅凭4点拟合误差判断可用。以下只是JSON结构和虚构示例，必须替换成实测数据：

```json
{
  "frame_id": "base_link",
  "units": "m",
  "image_size": [640, 480],
  "image_points_px": [[200,100], [440,100], [440,400], [200,400]],
  "ground_points_m": [[0.1,-0.24], [0.1,0.24], [0.7,0.24], [0.7,-0.24]]
}
```

将实测JSON路径写入 `config/line_follow.yaml` 的 `reverse_ground_mapping_file`，重新编译该包并重启。加载后显示 `ground_meters`，以局部地面直线 `Y=aX+b` 的 `b` 作为车身中心横向误差，方向取 `atan(a)`；最终横向容差由 `route_visual_goal_lateral_m=0.02` 配置，非未经验证的定位精度承诺。拒绝退化对应点、分辨率不符、非有限投影和拟合误差过大的文件；指定错误文件不会静默回退。镜头畸变明显时应先提供矫正图像，并在同一种图像上标定，不能混用原图像素与矫正图像像素。

输入失效或图像丢失时三个速度分量均归零，途中站仍等新的继续消息。末段之前的横线仍为途中站，最后30cm内横线才作为最终站。保留原点外最多10cm搜索及0.15m记录路线围栏；横移不扩大这些行程限制。前置相机不提供倒车后方未知障碍物观测。

`route_visual_strafe=false` 仅保留旧的“纵向+转向”控制用于回归；不建议该底盘继续用它追回中心。当前借鉴 [Duckietown的地面映射](https://docs.duckietown.com/ente/duckietown-manual/40-demonstrations/lane-following-lf.html)、[ViSP的线特征建模](https://github.com/lagadic/visp/blob/master/modules/visual_features/src/visual-feature/vpFeatureLine.cpp)及[Nav2分阶段对接](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_docking/README.md)，没有移植或额外启动整个框架。

### 前进横线目标保持与到位确认

已选中的停止横线漏检或位置跳变时保留原目标，不改追后面的另一条横线；原横线连续重新可见才恢复接近。远处短暂漏检仍受0.6秒、0.008m/s及5mm指令积分限制约束，持续漏检或临近到位时零速。最新一帧进入停车容差就先零速等待平均值和连续帧确认，避免平均滞后继续越线；单帧不直接算到站。确认期间单帧跳回区外不重新开车，连续一个平均窗口的新帧均在区外才撤销暂时停车，减少临近横线的启停。已确认到位仍锁定，途中站等待 `resume`，最终站完成任务。`stop_marker.settling_on_fresh_position` 表示正在等多帧确认。识别不到任何横线的检测问题仍需现场图像验证，本次没有扩大候选或重新引入短线组合。

现场日志出现连续 `approaching → missing_approaching → missing → approaching` 时表示已选横线间歇漏检，不能认定为正常到站。当前允许确认过的远处横线在图像、主线与底盘有效时跨越最多0.6秒的短时漏检；同次漏检指令累计超过5mm也停。没有足够目标确认、已触发到位待确认、接近到位区、相机过期或主线丢失均不使用这一余量。有效观测回来后重测位置并清零该次漏检计数，不补造到位样本，也不改追后续横线。

### 启动时自动离开当前横线

默认 `forward_ignore_start_marker=true`：前进启动时，第一条横线只要在整幅画面中可见，就将其当作出发线自动忽略，不用先发送 `resume`。出发线离开后下一条仍减速、确认并停车；途中站停车后的继续规则不变。若启动画面有多根横线，只忽略最靠近停车参考位置的一根，其他横线正常处理。离站识别窗口沿用 `reverse_start_marker_max_travel=0.01`m（前后共用、保留旧参数名），按已发速度指令积分，禁止把行驶途中才出现的横线误认成出发线。关闭该行为可在YAML设 `forward_ignore_start_marker=false`。日志显示“横线#1 忽略出发线”，状态字段 `start_marker_ignored` 可观察去重过程。

前进停在途中横线时，即使任务仍启用并等待继续，实际反馈速度归零后也自动保存有效返回路线，日志显示“前进路线已停稳保存”。发送继续消息前会立即把快照恢复为行驶中无效状态。退出ROS时保留已确认的停稳快照，不因时钟已关闭或最后一帧超时清除有效标记；未确认停稳的记录不会因此升级为可返回。

### 控制循环与后台处理

横线检测、预览绘制、路线 JSON 写盘分别使用有界后台工作槽；每项最多一个执行中任务和一个最新待处理任务，避免积压旧图。控制、里程计和停车状态机仍由一个执行器串行更新，后台检测只接收同帧数据快照，结果回到控制线程后才更新停车状态。窗口在进程主线程显示，桌面卡顿不会阻塞控制线程。

横线检测结果必须通过源时间戳、参数版本及既有0.5秒输入有效期检查，过期结果不会刷新有效期。后到的旧相机帧不覆盖已提交的新帧；持续收不到新帧仍停车。短暂输入超时期间输出零速，连续有效观测恢复后继续；既有任务进度超时仍限制等待时间。已经确认的横线停车保持锁定。

周期路线快照异步原子保存；开始/继续前进前确认旧路线已撤销有效标记，停用、切换倒车或退出时等待最终快照完成。写盘失败仍停车。真实里程计时间倒退、超过1秒的源数据间隔、坐标系变化或位姿跳变仍使路线失效，不能用于倒车；不通过扩大超时补猜缺失位移。`/line_follow/status` 的 `stop_detection.detector_processing_ms` 和 `result_age_ms` 分别表示检测耗时和使用结果时的源图年龄，单位毫秒；控制循环间隔超过100ms时另有节流警告，方便区分运算耗时和整体调度延迟。

倒车每5秒输出当前视觉反馈来源、归一化偏移、朝向误差（度）、指令角速度和可纠偏确认帧数。`reverse_finish_line_not_straight` 不再作为末段停车条件；看到横线前的停车应根据新的具体输入/范围故障排查。

小弯倒车实现参考 [Nav2 Graceful Controller 的倒车坐标和带符号曲率控制](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_graceful_controller/src/smooth_control_law.cpp) 与 [Autoware Pure Pursuit 的倒车独立前瞻配置](https://autowarefoundation.github.io/autoware_universe/latest/control/autoware_pure_pursuit/)。本项目相机仅提供像素误差，米制曲率来自已记录路线，不能把图像弯曲程度直接当作1/m；不额外启动Nav2或Autoware。

末段方向回归使用实测日志中的组合：偏移−0.136/−0.199/−0.295、朝向+6.3/+8.1/+11.7度。在当前down图像安装和前进已验证的方向映射下，这些组合需要正角速度纠正。状态 `visual_finish` 增加 `lateral_yaw_radps`、`heading_yaw_radps`、`feedback_yaw_radps`、`reference_curvature_per_m`、`target_curvature_per_m` 和 `lateral_sign`；5秒日志输出横向、朝向分项，原有速度和曲率上限不变。启动日志 `finish_lateral_sign=+1` 可用于确认已加载新末段标定。

### 横线编号、站点计数与离站去重

横线按一次任务内的行驶顺序编号，重复图像和同线漏检不产生新编号。准备阶段不登记站点；启用本轮任务后清空本轮账本，途中站 `resume` 保留账本。终端逐条输出 `横线#1 忽略出发线`、`横线#2 识别新横线`、`到站停车`、`收到继续，离站`、`确认通过`。画面显示 `BARS`（识别）、`SKIP`（直接忽略的出发线）、`STOP`（到站）、`PASS`（确认通过）和当前跟踪编号；这些计数不是互斥类别，例如一根横线可先到站、后通过。

“通过”必须有实测横线越过后侧参考区且已经移动，或随后一个独立站点确认到达；相应依据在事件 `basis` 中。不把单纯漏检当作通过，未确认通过的去重释放单独计为 `uncertain`。后续真实途中站仍按原规则停车等新的开始/继续消息，不因为增加计数就自动略过。

首次忽略资格窗口仍为 `reverse_start_marker_max_travel=0.01`m。已经忽略的出发线或继续后的离站线，在画面内保持身份，直到后侧画面边缘才解除可见线去重；抖到停车参考线后方不会成为新站。持续漏检时用独立的 `stop_marker_departure_hold_distance=0.25`m解除去重，优先使用通过连续性检查的真实里程计，无里程计时才估算指令行程。保持身份期间不会吞掉已经选中的下一站。

`/line_follow/status` 的 `station_counts` 提供 `encountered/ignored/arrived/resumed/passed/uncertain`；`stop_marker.station_history` 保留每根横线的动作，`current_station_number` 和 `ignored_station_number` 标明当前目标和离站旧线。`/line_follow/station_events` 为事件流（`std_msgs/String`，JSON），晚订阅者可从状态查看本轮历史。

本轮事件和计数异步原子保存到 `~/.ros/line_follow/last_station_log.json`，可通过只读启动参数 `station_log_file` 修改。前进路线 `last_route.json` 还会保存 `station_events`；下一次倒车可打印前进已记录站点的逆序供核对，实际倒车仍独立编号，不凭期望总数伪造站点或停车。旧路线无事件时明确报告“不能推断总横线数”。事件中的 `odom_pose_at_receipt` 是接收时车体里程计位置，用于追踪，不是标定后的横线世界坐标。账本表示已观测站点，不保证是场地全部横线的完整地图。
