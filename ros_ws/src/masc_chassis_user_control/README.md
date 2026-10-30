# MASC 蓝线循迹控制

本包将 `ruier_camer_diver` 的观测中线转换为底盘 `linear.x` 和 `angular.z` 指令。
默认前进速度 0.075 m/s（7.5 cm/s），支持在线调速。

## 车头相机与车身参考

以当前画面里轮子的位置为依据：车身位于画面上方，前方朝画面下方。
默认 `image_forward_direction=down`，虚拟车身参考为
`(图宽×0.5, 图高×-0.05)`，先放在画面顶边上方约5%，再按实际轮轴位置校准。
前进速度仍为正，不改变底盘的 +X 约定。

该位置是可调的图像参考，不是已经标定的底盘中心投影。单张图和可见轮子
不足以精确确定车身中心或地面尺度。控制不在不可见区域外推蓝线。
新方向组合为 `down` + `steering_sign=-1`，有效图像转向符号仍为 +1，
保持此前经过现场修正的左右关系。

控制分别计算：

1. 靠近车身的真实可见中线与车身参考纵轴之间的横向误差，纠正车身偏离。
2. 从车身参考指向真实可见前瞻点的方向误差，跟随弯道。

不再直接用识别模块的远处绿色目标点作为车身中心误差。
仅剩远处胶带、近端中线缺失过大时报告 `near_line_missing` 并停车。

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
| `error_deadband` | 0.06 | 归一化近端偏差连续死区 |
| `heading_deadband_rad` | 0.05 | 方向死区，约2.9度 |
| `steering_filter_time_sec` | 0.2 | 转向平滑时间常数，秒；0关闭 |
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

## 转向平滑与手动拖动

默认对近端横向误差和方向误差分别使用连续死区，然后对目标角速度进行
0.2秒时间常数的低通平滑，并限制角加速度为0.35 rad/s²。
持续偏离仍会纠偏；细小识别波动不再立即触发明显转向。
`cmd_vel` 保持50 Hz，不通过降低发送频率来抑制抖动。
丢线、禁用、超时和底盘状态禁止运动时绕过平滑，立即输出零指令。
这些参数控制车身角速度，不是直接指定各个舵轮角度。

在线设置，不需重启：

```bash
ros2 param set /line_follow_controller steering_filter_time_sec 0.2
ros2 param set /line_follow_controller angular_acceleration 0.35
ros2 param set /line_follow_controller error_deadband 0.06
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

终端无法直接感知松键，默认0.35秒没有新的移动按键就发零速度。
长按依靠操作系统的键盘连发；首次连发前可能短暂停车。
可用 `key_timeout:=0.5` 加长等待，但松键后停车指令也会更晚发出。
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

为减轻直线 S 形摆动，当前采用较宽的回中死区（横向归一化0.06、方向0.05 rad）和较低纠偏增益（位置0.65、方向0.35）；最大转速0.25 rad/s，角加速度0.35 rad/s²。正常跟线的线速度变化率0.08 m/s²，偏差减速系数0.4。未确认区域仍保留置信度减速；已通过两端几何验证的缺口默认系数1.0，并清除该续接路径上的重复疑似遮挡减速；丢线、故障和数据失效仍立即停车。树莓派实车效果需要沿现有轨迹再验证，弯道跟不上时应结合画面状态调整。

## 弯道优先跟随走向

当前可信线段的实际走向偏离车身前方约7°，或近端切线与前瞻走向相差约7°后，逐渐启用转弯优先控制；约14°时达到完整强度。近端拟合使用实测点的稳健斜率，不把整条直线仅向左/右平移误当成弯道；出弯后车身尚未对准倾斜直线时，仍优先跟随其走向。`geometry.bend_angle_rad`、`curve_strength`、`path_heading_rad` 可在 `/line_follow/status` 查看，预览标题显示 CURVE。图像角度不是已标定的地面转角。

弯道时中心容差由半图宽6%逐渐放到18%，中心纠偏力度降到原来的35%；方向增益从0.35逐渐提高至0.55，并优先使用近端到前瞻点的路径走向。转速上限由0.25过渡到0.28 rad/s（约16°/秒），弯道角加速度限制0.30 rad/s²。这样允许沿弯道有一定偏离，避免往中线拉的动作抵消转弯；直线保持原来的柔和参数。中心容差内的弯道偏差不再额外引起同等程度减速，正常前进设定仍为0.075 m/s。

相关参数在 `config/line_follow.yaml` 的 `curve_*` 项。模式按0～1强度连续过渡，并以0.35秒时间常数渐进进入，减少第一次转向过大，仍经过原来的转向滤波；单端/不合格缺口、丢线、过期、底盘故障与紧急停止规则不放宽。

## 90°停止横线

启用 `stop_marker_enabled` 后同时订阅 `/camera/line/stop_marker_mask`，用实测蓝线掩膜检测与局部主线垂直、两侧均有横向延伸的停止条，普通 L 形拐弯不算站点。横线后沿 `image_forward_direction` 仍有主线为 intermediate，没有继续线且视野足够为 final；证据含糊或标记仍在画面边缘时为 unknown。连续3张新图像确认，不用重复旧帧累计。

接近目标时从图高25%的距离范围内开始减速，最低接近速度0.008 m/s。接近和停车都使用同一横线最近5张新图像的算术平均位置；超过1秒的样本剔除，缺失帧不按零误差计入。误差按前进方向定义：正值表示尚未到达，负值表示已经越过。

平均误差大于图高0.8%时继续低速前进，不被之前的临时停车锁住；平均误差位于图高−2%～+0.8%时直接零速，连续3次新平均值在此范围内才确认到达。轻微越线默认算到达，不输出倒车命令；越过超过图高2%则停车报告 overshoot。480高图像对应允许越过9.6像素、尚未到达容差3.84像素，属于图像位置要求，不能换算为固定厘米，也不能保证底盘零像素误差。

停车目标由 `stop_marker_target_ratio` 设置，基础配置为图高50%，单独循迹任务默认46%。`stop_marker.error_ratio` 为平均误差，`raw_error_ratio` 为当前帧误差，`sample_count` 为有效平均样本数；状态日志同时显示平均/原始像素误差和不对称容差。

途中站到位后等待新的开始消息，提前发送不会留存；最终站不允许这个消息继续。

```bash
ros2 topic pub --once /line_follow/resume std_msgs/msg/Empty '{}'
```

已恢复通过的同一根横线会去重。整体任务只在最终站完成循迹，途中站等待时不启动后续机械臂。`/line_follow/status` 提供 `stop_marker` 对象，以及 stop_marker_approaching/averaging/settling/intermediate/final/unknown/missing/overshoot 状态。标记缺失、图像过期、底盘故障仍停车；不能用旧横线位置确认到达。停车确认、缺失、越线和类别未知共用5秒确认期限，期间状态切换不重置；无法确认最终站不当作正常完成。

图像中 STOP TARGET 画出停车目标，STOP intermediate/final/unknown 标出横线交点。标记长度、垂直角度容差、两侧最小长度、停车位置和接近限速均可在 YAML 的 `stop_marker_*` 调整。若明确不使用站点标记，可关闭此功能，随后不再要求掩膜流。

弯道连续性余量：近端缺线容差调整为图高40%；识别层逐行中心跳变允许65像素、连续漏检容差6行、中线形状跳变系数2.5。两侧实测线在缺口内相交时，接合转角上限统一为0.85 rad；缺口总长和两侧支撑长度限制仍保留。完全丢线不沿历史方向盲走，停止横线仍以配置的目标比例为停车位置。以上是图像参数，需实车验证。

长停止横线在图像层与循迹主线分离：`line/mask` 显示去除侧翼后的主线，`line/stop_marker_mask` 保留完整真实蓝线供停车判断。开关为相机 `separate_stop_bars`；状态 `separated_stop_bars` 报告本帧分离数量。横线两侧缺少可靠主线支撑时不会盲目切割。转弯前瞻点在相邻实测行间连续插值，减少逐行选点跳变；仅已验证缺口可插值，不向可见范围以外外推。当前不增加横线之后的路径规划。

停止横线定位使用两侧实体像素按列取中心并稳健拟合，与主线参考轴求交，不直接采用某一条 Canny 边缘。发出到位零速后固定主线参考轴，横线位置仍从每张新图像重测，因此主线提取抖动不会直接改变停车原点，真实横线位移仍可报告。停车后已确认的途中/终点类别锁存，避免继续线短暂反光改变类别。
