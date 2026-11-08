> 当前控制流程和实车测试入口见[底盘循迹逻辑与实车测试](CHASSIS_NAVIGATION_LOGIC.md)。以下包含早期调试记录，参数以当前配置及该流程说明为准。

# 标准路线：低速录制、检查、重放

当前普通循迹和自动充电入口保留，默认 `navigation_mode:=vision`。
新增模式仍使用 `line_follow_only.launch.py`，不启动机械臂、MoveIt、RViz、TCP或其他相机。
当前已验证的基线另有完整备份：项目根目录 `backups/line_follow_charge_baseline_20261106_163529/`。

## 采用的开源思路

- [Autoware Record/Replay](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/recordreplay-planner-howto.html)：先记录实际行驶点列，之后读取并跟踪，录制与重放分开。
- [Nav2 Regulated Pure Pursuit](https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller)：在路径附近投影进度，选择前瞻点，用几何曲率跟踪，并限制速度。
- [DonkeyCar Path Follow](https://docs.donkeycar.com/guide/path_follow/path_follow/)：记录、保存、加载命名路线；定位可以使用编码器。
- [ViSP visual servoing](https://github.com/lagadic/visp)：把图像中的稳定特征误差转成有界运动修正；这里用横线位置、交点横向误差和线方向做小步起步对齐。
- [Nav2 Docking](https://github.com/ros-navigation/navigation2/tree/main/nav2_docking)：分阶段接近、对齐和确认到位；这里借鉴阶段分离与停稳确认，不启动完整 docking server。
- [Nav2 DriveOnHeading](https://github.com/ros-navigation/navigation2/blob/main/nav2_behaviors/include/nav2_behaviors/plugins/drive_on_heading.hpp)：用实测位移判断完成、按剩余距离限制停车速度并设置最低有效速度。这里只适配速度规划思路，未直接引入其仅支持X轴的行为；横移用车体Y轴里程计投影。

本项目复用已有的路径几何控制与底盘接口，增加轻量前进重放和版本管理。
碰撞、急停、障碍、倾斜及连接异常仍由现有底盘状态检查处理。
没有引入地图定位；轮式里程计会漂移。相机保留对线、横线停车和末段充电对齐职责。

## 一次录制的标准步骤

1. 把车大致放到起步横线附近，保证横线和相连主线在相机画面内。启动后先停车，里程计确认停稳后等0.6秒，再收集5张停车后拍摄的新图；纵向位置、交点横向位置和主线朝向连续达标后，才建立路线起点。图像未稳定时不移动。
2. 确认相机安装、分辨率、朝向、底盘轮径/里程计配置固定，路线可通行。
3. 退出其他底盘控制任务，使用一个新的路线名称录制：

```bash
source /opt/ros/jazzy/setup.bash
source ~/project/robot_and_chassis_control/ros_ws/install/setup.bash
ros2 launch robot_task_control line_follow_only.launch.py \
  navigation_mode:=teach route_name:=route_01 line_show_window:=true
```

4. 如果起步仍接触充电桩，先以最高0.02m/s直线前进离桩，禁止横移和旋转；连续3帧新底盘反馈确认充电信号消失后立即发零速，再确认停稳、等待新画面。仍充电时最多允许累计行驶0.30m、等待25s，超限锁停；输入失效立即停车。这一阶段不记录路线，也不消耗后续视觉对齐的45s时限。`charging_mode:=false` 不关闭这个起步离桩流程。若始终没有实际前进，应检查底盘联锁/抱闸，不自动加速或忽略充电反馈。
   离桩后起步采用“停车观察 → 摆正朝向 → 横移回中 → 前后对横线 → 停车复核”。每次从稳定画面中按此优先级选择一个超差方向，只激活一个运动轴；不把横向和朝向误差相加成旋转命令。朝向已经达标时不再发微小转向。每步转角至少0.01rad、最多0.04rad，均大于0.003rad的里程计步进到位容差。
   前后与横移分别估计剩余距离，再决定本步距离。有地面标定时直接使用地面距离；无标定时先试探约5mm，只有前后偏差略超容差时试探2mm，横移不再使用2mm首次试探。用两次停车稳定图像的误差变化除以实际里程计位移估计局部响应，包含停车滑行量。小于半像素或画面波动的变化、明显侧滑/转动、错误响应方向和比例突变不参与估算，并清除旧比例重新试探。保留最近3次一致响应，取中位数报告比例、最大响应保守计算距离；转向修正后重新学习比例。
   目标设在允许误差的一半处，走估计修正距离的80%，每步最多1.5cm；前后步长至少2mm。横移的最小计划距离取 `v_min²/(2a)+1mm`，默认3.5mm，避免给最低横移速度安排更小的计划距离。它只是用配置减速度计算的规划下限，不是实测停车精度保证；配置的单步上限不能小于此下限。
   单步速度按 `sqrt(2a × (剩余距离−到位容差))` 计算，再受各轴最低/最高速度约束。前后最低目标速度0.012m/s、最高0.03m/s；横移最低目标速度0.02m/s、最高0.03m/s，转向最低0.02rad/s。加速仍由已有加速度限制完成，起动斜坡中的发布速度可能暂时低于最低目标速度；没有额外积分累积。里程计到位立即归零，不继续保持最低速度。底盘的实际滑行仍要停车后用新图复核。
   移动期间不追逐图像、不反向修正；到位立即发零速，重新停稳后再等待、收集新图，旧图按拍摄时间剔除。停车期间视觉短暂断续不重置已经确认的停稳计时，保留最多1.5秒内的有效采样；每次恢复都要求失效之后拍摄的新图，再决定运动或完成，超过500ms的观测仍不用于决策。里程计失效或检测到实际运动时重新停稳；运动中输入失效则立即中止该小步，重新停车观察。
   起步对齐最多45秒、累计位移上限30cm；修正速度不超过0.03m/s，倒退修正最多10cm，单步4秒仍未到位则锁停。输入过期立即停止当前小步，恢复后重新停车观察。无地面标定的局部响应只是下一小步的估计，不代表厘米级定位精度。离桩最高0.02m/s；脱离充电后，录制巡航设定为0.03m/s。
   使用原有视觉循迹，每约1cm或朝向变化约0.05rad记录一个实测位姿，不按运动指令猜位置。

5. 起步横线按原有离站规则忽略；第一条有效横线停稳后封存路线、检查并保存标准版本。
6. 等待5秒后沿本次实际行驶路线倒车，末段视觉摆正、回中、找横线，持续充电反馈确认后结束。
   到站后即保存前进标准；后续回桩失败也不会删除已保存记录，回桩效果仍需单独检查。

起步日志每2秒输出一次“起步对齐诊断”：三方向误差及单位、有效帧数量、到位确认帧数量、实际三轴速度、停稳判定、图像年龄和具体输入阻塞原因。每步日志分别打印估计剩余距离、估计来源、比例检查结果，以及本步前后位移、横移位移和转角。`probe_no_reliable_scale` 表示尚无可信米/像素比例，不报告虚构距离；`local_response_tentative` 表示仅一次有效响应，`local_response_confirmed` 表示已有至少两次一致响应，这些均不是地面标定精度承诺。实车复测时观察临近目标步长是否缩短、画面异常时是否回到试探，并保留完整日志。`start_alignment_complete` 才表示起步成功；`waiting_for_stop`、`settling` 或单步到位都不代表整体对准。录制首点和重放锚点都在全部方向确认、实际停稳后建立。

诊断还包含 `last_command`（上次发布的v/vy/w）、`step_target_m`（车体轴有符号目标）、
`step_actual_m`（该轴里程计实测位移，含停车后的滑行）、`step_remaining_m`、
`actual_velocity`、`marker_candidates`、`marker_rejected`、`detector_processing_ms` 和 `marker_result_age_ms`。
横移超时后，可据此区分速度目标已发出但没有位移、轮子反馈有速度但位移不足、以及等待有效视觉输入。

## 起步横移最低速度实车确认

0.02m/s目前是待实车确认的初始值，不代表已经测得本车起动阈值。前后、向左、向右的起动能力可能不同。
先结束旧任务，放回起步横线附近，让横线接近画面高度一半，并让主线交点距图像中心约25～35像素（640宽图）；
这样会优先测到一次横移，而不是先做较多前后修正。横线和主线须保持可见。

先按低巡航速度做一轮实际往返，使用独立文件保留之前路线：

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_task_control line_follow_only.launch.py \
  navigation_mode:=vision round_trip:=true charging_mode:=false \
  workflow_file:="$(ros2 pkg prefix robot_task_control)/share/robot_task_control/config/line_follow_charge_test.yaml" \
  start_alignment:=true line_follow_speed:=0.03 line_reverse_speed:=0.05 \
  start_alignment_min_lateral_speed:=0.02 start_alignment_lateral_speed:=0.03 \
  line_stop_marker_target_ratio:=0.5 line_show_window:=true \
  route_file:="$HOME/.ros/line_follow/alignment_base_020_left_01.json" \
  2>&1 | tee /tmp/alignment_base_020_left_01.log
```

这不是仅对齐模式：起步对齐完成后会继续前进，到下一条有效横线等5秒，再倒车返回。
首次横移应计划约5mm；加速后横移指令应约0.02～0.03m/s，并出现对应方向的实际位移。
`step_actual_m` 接近 `step_target_m`（单步到位容差1mm）后应先归零，再等待新画面复核；
只有 `start_alignment_complete` 才算全部对齐。4秒没有完成单步仍停车，不自动加大速度冲过去。

将车重新放回起点并换到另一侧重复，左右各测3次，文件名改成 `right` 及 `_02`/`_03` 避免覆盖。
若需要测最低可用值，保持横移上限0.03，分别试最低目标0.015、0.02、0.025m/s，每轮只改此参数、换新日志名；
选择两方向均能重复完成单步、停车后无明显反复越过的值，再留适当余量。
单步初始速度可能高于最低目标值，不能凭一次成功就认定最低值足够；还要看减速段 `last_command.vy` 与实际反馈。
若0.025～0.03仍基本无位移，先检查实际 `/cmd_vel`、轮子转向状态和里程计，不将原因直接归为静摩擦。
若再出现速度指令超时，需单独检查调度/计算负载；延长对齐总时限不能修复指令链超时。
横移确认后，再把正常前进巡航恢复到0.1m/s；专门录制 `teach` 的速度仍为0.03m/s。

默认录制的是“起点到第一条有效横线”的一段。后续其他形状路线换名称重新走；
需要不同站点流程时，可通过已有的 `workflow_file` 选择包含 `save_line_route` 的任务。
`teach` 模式必须保留保存步骤；不要用 `round_trip:=false` 把它跳过。

## 保存的数据

默认位置：`~/.ros/line_follow/standards/route_01/`。

```text
route_01/
  standard_route.json                 # 当前选用标准；只有合格新录制才原子更新
  versions/<时间_版本号>/
    raw_route.json                    # 未平滑的约1cm采样路线及时间/速度，保留原始坐标系
    standard_route.json               # 本版本标准，包含源记录校验值和录制参数
    points.csv                        # x/y米、yaw弧度、弧长米、ROS时间秒、车体vx/vy和角速度
    quality_report.json               # 点数、长度、最大/中位间距、相邻角度变化、站点事件数
    preview.svg                       # 里程计坐标系中的轨迹预览
```

`point_samples` 中的时间、速度与所选采样点对应；不是所有CAN原始报文。
`recording_config` 保存控制参数、路线抽样参数、图像尺寸及起步视觉对线信息。路线首点是在横线对齐、停稳确认之后取到的 `/wheel_odom` 位姿；对齐期间的里程计样本不会混入路线。重放同样延迟建立 Pure Pursuit 锚点，避免每次开跑位置不同导致路线整体偏移。
站点事件保留检测/忽略/到站等记录；事件中的里程计位置为接收时观测，不是横线地面坐标标定。

结构检查要求：记录完成且有效，至少10点、长度至少10cm、相邻位置间距不超过8cm、
相邻朝向跳变不超过约20度。检查失败会保留原始记录和报告，上一条标准不被替换。
质量检查失败会停止本轮任务，不自动继续返程。
这些条件只检查记录结构和连续性，不代表毫米级物理精度。录制出现明显蛇形、打滑或偏线时应重录。

## 检查后重放

已把实车完整记录 `~/.ros/line_follow/shuttle_test_01.json` 归档为本工作空间的
`routes/standards/shuttle_test_01/standard_route.json`：107点、约2.071m，包含终点横线到站事件。
该目录下 `versions/` 同时保存原始记录、质量报告、CSV和预览。质量检查只检查数据结构和连续性，不代表实车重复精度已验证。
未使用仅0.992m且没有到站事件的 `undock_test_01.json`。

使用这条完整路线进行实车往返：

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_task_control line_follow_only.launch.py \
  navigation_mode:=replay \
  standard_route_file:="$PWD/routes/standards/shuttle_test_01/standard_route.json" \
  round_trip:=true charging_mode:=false check_devices_on_startup:=false \
  workflow_file:="$(ros2 pkg prefix robot_task_control)/share/robot_task_control/config/line_follow_charge_test.yaml" \
  start_alignment:=true line_follow_speed:=0.075 line_reverse_speed:=0.05 \
  line_stop_marker_target_ratio:=0.5 line_show_window:=true \
  route_file:="$HOME/.ros/line_follow/replay_$(date +%Y%m%d_%H%M%S).json" \
  2>&1 | tee /tmp/chassis_replay_test.log
```

上述launch会实际启动运动：离桩、视觉对齐后按标准路线前进，下一条有效横线停车等5秒，再沿本次实际轨迹倒车。
初始速度前进0.075m/s、倒车0.05m/s；`charging_mode:=false`，返程按横线结束，不追加充电搜索。
标准文件只读，每次实际路线写入独立时间戳文件；日志在 `/tmp/chassis_replay_test.log`。
可附加 `line_show_window:=false` 等launch参数。观察状态中的 `standard_route.visual_loss` 与 `diagnostics` 可判断是否处于丢线续行。

先查看 `preview.svg` 和质量报告，再把车放回同一个实物起点，保持与录制时相同的朝向。
只让里程计数值归零，不能保证车处在同一个实物位置。

```bash
ros2 launch robot_task_control line_follow_only.launch.py \
  navigation_mode:=replay route_name:=route_01 line_show_window:=false
```

普通重放巡航参数默认0.1m/s，路线控制器上限同为0.10m/s；首次复测建议显式设0.075m/s，离桩仍受0.02m/s上限约束。
主控制按当前里程计位姿投影到标准路线并跟踪前瞻点；录制时间和速度用于检查，不决定重放进度。
途中视觉角速度辅助增益0.5，限制在±0.06rad/s，横向或朝向偏差扩大时会降低巡航速度。中段转向直接使用标准轨迹的米制曲率，不经过依赖摄像头新帧的视觉转向死区/换向确认。
可信弯道的前进最低巡航目标为0.05m/s，低巡航设定、较大偏差、终点及物理限制优先。
正常路线变速使用0.04m/s²加速、0.06m/s²减速斜坡；末段视觉接管前会按剩余距离提前减速。
相机继续正常出新帧但主线暂时检测不到时，允许沿已有路线短时续行：最高0.05m/s，累计实际路程最多0.30m、时间最多5秒，任一到限锁停并终止任务；不自动恢复。
连续3帧主线恢复有效后退出续行，恢复有限视觉辅助。相机停帧、掩膜过期、处理超预算、里程计失效或底盘状态异常仍停车，不使用旧图作为视觉恢复依据。
同样到首条有效横线停5秒后返程；仅charging_mode=true时额外执行充电确认。返程跟踪本次实际前进记录，不覆盖标准文件。

- 前进最后约15cm锁定切换到视觉低速接近横线，不再重复调用路径前瞻；允许的额外找线行程最多5cm，末段最多30秒，超限锁停。
- 倒车保留原有记录路线返回、末段视觉摆正/横移回中、横线定位及充电确认流程。
- 相机、里程计、底盘反馈失效仍会停车；路径偏差超过当前路线控制上限0.15m也会停车。
- 起步横线按停车后的多帧视觉误差对齐；无地面标定时以图像比例复核到位，小步局部响应估计不作为米制视觉测量。配置 `reverse_ground_mapping_file` 指向实测相机到 `base_link` 地面的单应性标定后，纵向/横向误差才以米报告。
- 起步对线与录制差异过大、相机方向/参考参数变化、坐标系名称不一致或标准文件缺失，拒绝重放。
- 里程计路线仍不是全局定位：对齐只能把重复起步落在同一条可见横线附近，不能识别场地绝对位置；相机安装、横线形状和胶带关系需保持稳定。

当前减负方式是重放时限制图像识别上限为8Hz，速度控制仍50Hz，USB采集配置为15Hz。
录制保留原有识别频率；普通 `vision` 模式不启用此限频。降低识别调用次数预计能减少计算量，
实际CPU占用、顺滑程度、最大可用速度和定位误差需在树莓派实测，不作未经验证的承诺。

## 弯道返程修正后的实车复测

2026-11-07日志中的返程在剩余0.847m触发 `reverse_target_not_behind`。
保存路径的弯道曲率约5/m，而原返程限幅仅2/m；使用原记录和原限幅进行数值核对，
在剩余0.846m复现同一保护。途中倒车现在单独使用
`reverse_route_max_curvature:=5.0`（1/m，允许范围0.2～5），
同时按角速度上限同比降低线/角速度。末段充电仍用 `route_max_curvature: 2.0`，
偏离路线15cm及目标方向保护保持有效。前瞻参数0.26m会按速度缩放，
在0.03m/s时实际前瞻约0.08m，并非固定26cm。

前进将“斜直线的航向误差”与“路径弯曲”分开：直线倾斜仍进行航向纠正，
不会因此放宽回中死区。持续同侧偏移超过直线死区1.25倍时，
仍需5张不同新图确认才放行小幅回中。记录器继续保存原始实际里程计轨迹，
不会自动将蛇形轨迹改为胶带中心线。

1. 结束旧任务，将车重新放回起步横线附近，保证横线和主线可见。
   **不能在中途卡住的位置直接重启返程**：新返程会将当前位置当作记录终点建立锚点。
2. 先重新走一次完整低速往返，保存到独立测试文件：

```bash
source /opt/ros/jazzy/setup.bash
source ~/project/robot_and_chassis_control/ros_ws/install/setup.bash
ros2 launch robot_task_control line_follow_only.launch.py \
  navigation_mode:=vision round_trip:=true charging_mode:=true \
  start_alignment:=true line_follow_speed:=0.03 \
  reverse_route_max_curvature:=5.0 line_show_window:=true \
  line_stop_marker_target_ratio:=0.5 \
  route_file:=~/.ros/line_follow/curve_fix_test.json \
  2>&1 | tee /tmp/chassis_curve_fix_test.log
```

3. 起步确认 `start_alignment_complete` 后才开始录制；观察去程直线偏移是否持续扩大、
   出弯是否仍频繁左右摆动，有明显异常应结束本次测试并保留日志。
4. 终点保存后等5秒自动返程，关注原卡住弯道能否通过；最后应由充电新反馈连续确认完成。
   `reverse_route_complete` 才表示返程完成，途中停车不等于到位。
5. 若再次触发保护，日志会打印目标在车体坐标下的位置、目标相对后方角度和路线偏差；
   状态话题还记录请求曲率、限幅值及是否饱和。结合日志和路线文件检查，勿直接放宽保护。

数值与节点回归检查只能验证控制逻辑，不能证明轮胎无打滑、里程计准确或实车能达到相同轨迹误差。

### 末段低速驻车修正与重新录制

20:23启动的实车测试已通过原故障弯道，记录长2.070m、共107点；
末段识别横线后速度降为0.008m/s，低于驱动0.01m/s零速判定，
自动驻车后剩余约0.093m不再推进，最终触发90秒充电超时。
该轮原路线、日志、摘要和等比例预览已另存：
`backups/line_follow_route_20261107_2025/`（项目根目录）。
这份前进记录有效，整轮充电往返未通过。

当前横线接近、充电搜索和越线前移修正统一使用 `stop_marker_creep_speed: 0.012` m/s；
配置必须大于0.01且不超过0.03m/s。充电阶段仍服从设定行驶/倒车速度上限，
上限低于有效运动门槛时明确停车报错；速度设为0时保持停车。
驱动自动驻车策略、到位立即归零、图像过期停车、90秒超时和额外搜索10cm限制保留。
横线漏检接近仍限制总指令行程5mm，下一周期会超限则提前停止，不延长漏检行程。

放回实物起点后重新录制并往返验证，使用新文件名保留对比：

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_task_control line_follow_only.launch.py \
  navigation_mode:=vision round_trip:=true charging_mode:=true \
  start_alignment:=true line_follow_speed:=0.03 \
  line_stop_marker_target_ratio:=0.5 line_show_window:=true \
  route_file:=~/.ros/line_follow/creep_fix_01.json \
  2>&1 | tee /tmp/chassis_creep_fix_01.log
```

启动后先起步对齐，前进录制，第一条有效横线停稳后封存，等待5秒自动倒车。
检查原弯道、末段0.012m/s能否持续推进、横线到位和连续充电确认；
只有工作流完成才算本轮通过。再测一轮时将路线和日志文件名都改成 `_02`，避免覆盖。

## 其他路线及恢复原流程

换名称即可独立保存：`navigation_mode:=teach route_name:=route_02`；
选择该路线：`navigation_mode:=replay route_name:=route_02`。
再次录制同名路线会新增版本目录，合格后更新当前标准，不删除历史版本。
也可用 `standard_route_file:=/绝对路径/standard_route.json` 指定文件。
标准文件必须与每次运行的 `last_route.json` 分开。

原有入口保持可用：

```bash
# 原有普通视觉循迹
ros2 launch robot_task_control line_follow_only.launch.py \
  drive_direction:=forward line_show_window:=true

# 原有纯底盘自动充电测试
ros2 launch robot_task_control line_follow_only.launch.py \
  round_trip:=true charging_mode:=true line_show_window:=true line_stop_marker_target_ratio:=0.5
```

## 倒车转弯过头时的参数

纯底盘往返提速测试可设置 `line_follow_speed:=0.1 line_reverse_speed:=0.05`：
前进巡航设定10cm/s，记录轨迹倒车最高5cm/s。倒车速度还受前进速度设定与路线速度上限约束，
弯道、末段视觉接管和横线接近仍按原有规则减速；起步对齐速度仍最高0.03m/s。
`line_reverse_speed` 传给控制器的 `reverse_max_speed`，摄像头倒车和记录轨迹倒车均使用此上限。
启动日志同时输出 `forward_speed` 和 `reverse_max_speed`，用于确认参数生效。

专门录制标准路线（`navigation_mode:=teach`）使用独立的 `line_record_speed`，默认0.03m/s。
此模式忽略正常运行参数 `line_follow_speed`，不会因巡航提速而加快录制。
本入口正常运行（`vision`/`replay`）前进巡航参数默认0.1m/s、倒车最高0.05m/s；
实际输出仍受弯道、横线及路线跟踪器上限约束（replay路线跟踪上限目前0.10m/s）。
普通往返为返程缓存当次去程轨迹仍属于正常运行，不会切到teach录制速度。
teach的返程还受其0.03m/s运行设定约束。这里的两种“录制”需区分。

前进和录制的视觉转向参数保留。倒车记录路径使用独立前瞻 `reverse_route_lookahead`，
默认从20cm调整为26cm，用更远的后方路径点减轻短前瞻造成的急纠偏；
末段摆正增益 `reverse_finish_heading_gain` 从0.80调整为0.60，减少对齐时的过强转动。
前进重放前瞻仍为20cm。倒车速度、横线停车和充电确认规则保留。

```bash
# 在已完成前进录制、原地停稳后，单独测试返回
ros2 launch robot_task_control line_follow_only.launch.py \
  drive_direction:=reverse line_show_window:=true \
  reverse_route_lookahead:=0.26 reverse_finish_heading_gain:=0.60
```

这两个参数也可以附加到teach/replay往返命令。每次只调整一个：
途中倒车仍急转，可尝试前瞻0.28m；如果开始切弯、跟不上弯，退到0.24m。
末段摆正仍过冲，可将摆正增益改为0.50。不要同时放大停车容差或改变转向符号。
这些是试跑起始参数，无法仅凭代码确认实车转向延迟及过冲已消除。
录制本身若偏线、切弯或蛇形，须重录；重放不会自动修复坏路线。

末段视觉接管的横向范围与最终停车容差分开：`route_finish_entry_lateral=0.65`
允许半图宽65%内的可信近端线进入摆正/横移回中；最终仍要求半图宽8%内、朝向6度内且连续确认。
原来的35%接管限制会使较大偏差一直无法进入纠偏，现已修正。640宽时65%约208像素，
这不是厘米映射；充电的里程计行程围栏与最多额外10cm搜索限制保留。
相机结果超过500ms仍停车，不用过期图像纠偏。若反复出现camera_stale/timeout，
应结合日志中的源图到结果延迟与detector_processing_ms检查处理负载；放宽居中范围不能解决调度超时。
