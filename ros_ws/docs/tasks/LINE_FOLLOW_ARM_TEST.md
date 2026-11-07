# 循迹各站点与机械臂姿态联动测试

入口：`ros2 launch robot_task_control line_follow_arm_test.launch.py`。

当前“行车位→循迹到站→过渡位等待5秒→行车位→沿记录路线返程”的完整实车方案见
[机械臂与小车整体往返测试](ARM_CHASSIS_ROUND_TRIP_TEST.md)。
该方案用 `test_profile:=return_charge start_tcp:=true enable_status_monitor:=false`，
保持9010 TCP通信，关闭额外状态汇总；首次默认自动执行。

该入口包含真机机械臂驱动、MoveIt、8cm地面碰撞场景、关节姿态任务、底盘驱动、速度选择器和USB循迹。沿用机械臂控制栈中的力传感器节点；本测试用MoveIt位置规划，不依赖单位标定和有效力数据。默认不启动HK、深度相机或拍照发送；默认加载9010状态桥，`remote_control:=true`时等待主机首次开始。

## 启动

先退出重复的机器人、底盘和循迹launch。在`ros_ws`执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_task_control
source install/setup.bash
sudo bash scripts/setup_can.sh auto
ros2 launch robot_task_control line_follow_arm_test.launch.py launch_rviz:=true
```

CAN脚本仍需手动运行，launch不执行sudo。机器人默认IP为169.254.199.150，可用`robot_ip`修改。已有机器人驱动时加`start_arm_driver:=false`；已有MoveIt时再加`start_moveit:=false`，安全监控和地面节点也必须已运行。

## 自动执行顺序

1. 等待驱动、外控、轨迹控制器、MoveIt和循迹设备就绪，确认底盘连续停稳1秒。
2. 按关节目标回到行驶姿态，核查实际关节到位，再开始循迹。关节轨迹控制器保持该姿态。
3. 每条途中停止横线到位后锁住底盘速度输出，保留循迹节点、驱动和路线记录。再次确认轮速连续停稳1秒。
4. 机械臂到中转姿态，再执行下表四个笛卡尔测试点，最后按关节目标回中转姿态。
5. 途中站等待下一次开始消息。收到后先经中转回行驶姿态，确认实际关节到位，再放行底盘并继续原来的循迹。
6. 重复处理每个途中站。最终横线同样执行一次机械臂动作，结束后保持中转姿态，底盘输出保持锁住。

中转姿态仅用于过渡；四点目标保留各自方向，不把中转姿态当作拍照或触碰姿态。

| 姿态 | 基座、肩部、肘部、手腕1、手腕2、手腕3（度） |
|---|---|
| 行驶 | -45, 15, -71, 4.4, 148, 75 |
| 中转 | -45, 0, 0, 0, 90, 0 |

两组关节角统一修改：`src/eli_cs_robot_moveit_config/config/arm_joint_postures.yaml`。

| 点 | tool0 XYZ，base_link下，米 | 固定轴RPY，度 |
|---|---|---|
| P1 | -0.05, 0.2, 0.475 | 0, 0, 90 |
| P2 | -0.13, 0.35, 0.8 | 0, 0, 90 |
| P3 | 0.3, 0.165, 0.8 | 0, 0, 0 |
| P4 | 0.3, 0.2, 0.6 | -160, 0, -77 |

本站动作集中在`config/line_follow_arm_test.yaml`的`workflow.station_steps`；最终站用YAML锚点复用同一组动作。更换拍照/检测动作只需替换这一处，中转和行驶保护保留。这里是四点位置测试，不执行20N触碰；所有路径运行时仍须通过MoveIt碰撞检查，当前四点整段路径尚未完成真机验证，规划失败就停止，不跳过该站继续开车。

## 途中站继续

首次启动默认自动执行。途中站动作完成后，发一次新的开始：

```bash
ros2 service call /task_controller/start std_srvs/srv/Trigger '{}'
```

也兼容原来的`/line_follow/resume` Empty消息。本站动作进行中收到继续请求会排队，必须等机械臂回行驶位才放行。未进入站点的提前消息不保留。工控机`start_task.py`在途中站的含义同上；首次等待主机开始时使用：

```bash
ros2 launch robot_task_control line_follow_arm_test.launch.py remote_control:=true
```

最终站结束后的start是新的一轮，不是继续穿过终点。

## 状态与停止

```bash
ros2 topic echo /task_controller/status
ros2 topic echo /arm_joint_postures/status
ros2 topic echo /line_follow/status
ros2 service call /task_controller/stop std_srvs/srv/Trigger '{}'
```

关节任务返回排队成功不代表到位；工作流只接受本次request_id对应的DONE，关节误差要求小于0.01rad。关节状态、外控、安全状态、地面场景失效会取消本节点关节运动，工作流不放行底盘。关节任务失败后保持中止锁定，需要查明原因并重启该任务节点。STOP会取消关节姿态任务；已有笛卡尔MoveIt执行的停止行为沿用执行器，紧急情况使用设备停止/急停。

路线记录不中途清空，不重启底盘、循迹或里程计节点。只有首次出发和最终站结束启停循迹控制。单独的`line_follow_only.launch.py`不增加机械臂姿态动作。

软件验证覆盖两个途中站、继续前实际到位门槛、最后一站以及路线连续性；模拟服务验证不代表四个笛卡尔点在现场可达。地面碰撞约束保留base_link下Z≤0.08m。实体空间中的桌面等障碍仍需建模。

## 自动测试：首条横线到站、中转等待5秒、倒车返回充电

复用同一个联合launch，选择`test_profile:=return_charge`，不新增重复入口：

允许从充电桩出发：机械臂先到行驶位，再以不超过0.02m/s循迹离桩；连续3帧确认停止充电后恢复正常速度。仍充电时累计前进上限30cm、总等待25s，超限锁停。手动充电、急停、故障或传感数据失效仍禁止运动。

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_task_control line_follow_arm_test.launch.py \
  test_profile:=return_charge line_show_window:=true launch_rviz:=true
```

此命令包含真机驱动，设备就绪后自动执行：

1. 确认底盘停稳，机械臂按关节角到行驶位`[-45, 15, -71, 4.4, 148, 75]°`。
2. 前进循迹；出发处已经存在、被离站逻辑忽略的横线不算目标。第一条有效横线到位即停车，即使它被识别成途中站，也不等待额外开始消息。
3. 确认轮速停稳，封存刚才的前进路线。随后机械臂到中转位`[-45, 0, 0, 0, 90, 0]°`，服务核查实际关节到位后，工作流再等待完整5秒。
4. 经中转按关节目标回行驶位，确认实际到位后才放行底盘。原进程内切换为倒车，不重启驱动、不清除路线；沿封存路线回到出发区域。
5. 末段用倒车充电模式先摆正、横移回中，再直退；横线辅助定位，充电首帧停车、连续反馈确认后完成。横线后额外搜索最多10cm，偏离/超距/充电信号异常时锁停报错。原有充电模式的图像、里程计和安全保护仍有效。
6. 等待底盘停稳，任务结束；机械臂保持行驶位，不循环、不自动再次出发。

启动前仍须配置CAN（需要时执行`sudo bash scripts/setup_can.sh auto`），并退出重复驱动。该profile不执行原来的四个笛卡尔点，也不启动海康、深度相机、力传感器、笛卡尔点位执行器；默认加载9010状态监视/TCP桥，MoveIt与关节姿态任务仍运行以执行碰撞检查和关节轨迹。中转是过渡姿态，没有触碰/20N检测。

当前默认充电位使用`charging_status_layout:=byte_flags`；若已实测确认固件使用整个0x204 uint32 bit3，改为`charging_status_layout:=bitfield32`。需要主机发首次开始时可加`remote_control:=true`，此时会加载既有TCP桥并等待主机start；后续中转、等待和返程全自动。

流程配置：`src/robot_task_control/config/line_follow_arm_charge_test.yaml`；两组关节角共用`src/eli_cs_robot_moveit_config/config/arm_joint_postures.yaml`。`delay.seconds`控制到中转后的等待时间。`finish_at_first_marker: true`只用于显式forward步骤，要求停止横线确实到位，普通丢线不能代替到站。`arm_postures_before_chassis`在每个前进/倒车步骤前自动插入并确认行驶姿态，配置不重复写关节角。

查看流程与充电阶段：

```bash
ros2 topic echo /task_controller/status
ros2 topic echo /arm_joint_postures/status
ros2 topic echo /line_follow/status
```

本测试已经用本机模拟服务验证首条途中横线可完成前进、封存不丢失、中转实际到位后至少等待5秒、回行驶位后才倒车及充电完成后停止。姿态往返和充电控制沿用各自已通过的仿真/模拟检查；完整流程尚未驱动实车验证。


树莓派联合往返测试优先关闭RViz，保留循迹画面：

```bash
ros2 launch robot_task_control line_follow_arm_test.launch.py \
  test_profile:=return_charge line_show_window:=true launch_rviz:=false
```

该配置提前准备USB相机和识别，机械臂连接稳定5秒、设备检查及行驶姿态到位的条件仍保留；
相机预热不会提前使能小车。`prepare_line_camera_early:=false` 可恢复相机原来的串行启动顺序。
图像识别及后台横线检测各默认最多2个OpenCV计算线程；开启RViz时默认显示刷新率降为10Hz，降低竞争。
速度选择器继续执行100ms输入超时停车，新增“输入超时/恢复”和“调度延迟”日志，
用于区分识别主动停车与速度发送断流。速度话题使用只保留最新一条的队列，不积压旧命令。


联合往返 `return_charge` 使用关节姿态专用模式：不加载未使用的柔顺控制节点、
笛卡尔执行器及力传感器（显式启用传感器时除外），也不等待柔顺使能服务。
MoveIt、模型/碰撞规划及安全监视保留，机械臂关节动作只在对应步骤请求后执行。
普通 `stations` 和实际柔顺/触碰任务仍保留原来的控制接口。
机械臂控制器分为“激活组”和“仅配置组”，由两个spawner顺序加载；任一批失败取消启动。
驱动、里程计与已封存返程路线在步骤切换间持续保留。

关节姿态运动和站点等待期间，USB相机采集按需暂停，下游图像识别不再持续计算；
恢复前进/倒车前先开启采集并等待有效视觉反馈，15秒仍未就绪则报错停车。
底盘和机械臂驱动、安全监视、里程计保持常驻；相机暂停不清空返程路线。
横线斜率计算改由NumPy执行，后台检测使用受限OpenCV并行，控制状态机仍保持单线程。


## 单独循迹：离桩、首条横线等待5秒、返程充电

先退出其他底盘/联合任务，避免同时向速度话题发送指令。启动后自动执行，无需调用开始服务：

```bash
source /opt/ros/jazzy/setup.bash
source ~/project/robot_and_chassis_control/ros_ws/install/setup.bash
ros2 launch robot_task_control line_follow_only.launch.py \
  round_trip:=true charging_mode:=true \
  line_show_window:=true line_stop_marker_target_ratio:=0.5
```

复用已有launch，选择 `config/line_follow_charge_test.yaml`：
底盘停稳→前进循迹离桩→第一条有效横线停稳→封存路线→等待5秒→沿记录路线倒车充电→停稳结束。
起步出发线按原规则忽略；到站横线既可以是途中横线，也可以是末端横线。
前进离桩仍为最高0.02m/s、连续3帧不充电后恢复正常循迹；超过30cm或25秒仍充电则锁停。
倒车保留视觉回中、横线定位、10cm额外搜索限制，连续充电反馈确认才完成。
不会启动机械臂、MoveIt、RViz、TCP或其他相机；整个往返期间USB循迹相机和里程计保持工作。
`round_trip:=true charging_mode:=false` 继续使用原来的“最终横线往返”流程。

纯MoveIt站点测试现在默认 `launch_admittance:=false`，不加载导纳节点；关节姿态管理器及任务控制器
也不再等待或调用导纳开关服务。旧独立阻抗入口保留；自定义任务确实含导纳步骤时须显式开启该参数。
相机点触碰由camera_detection_real入口的MoveIt限力流程负责，本四点位联动测试不自动新增触碰步骤。
