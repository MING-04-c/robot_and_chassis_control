# 底盘与机械臂 YAML 联合流程控制器

这里把底盘任务视为“远程调用”。任务表只登记对方实现了哪些调用；流程 YAML 决定调用顺序、等待时间、机械臂末端目标和控制模式。

通信中只保留两个标识：

- `task`：调用名称，例如 `forward_10cm`。
- `event_id`：每条控制或状态消息的唯一编号，只用于 ACK 与去重，由程序自动生成。

## 可调用任务表

`config/chassis_tasks.yaml`：

```yaml
tasks:
  forward_10cm:
    description: Move forward 0.10 m
  backward_10cm:
    description: Move backward 0.10 m
  left_10cm:
    description: Move left 0.10 m
  right_10cm:
    description: Move right 0.10 m
  rotate_left_90deg:
    description: Rotate left 90 degrees
  rotate_right_90deg:
    description: Rotate right 90 degrees
```

任务表的键，例如 `forward_10cm`，就是发给对方的调用名称。只有对方已经实现的任务才能加入表中。

## 默认流程

`config/workflow.yaml`：

```yaml
workflow:
  name: agv_arm_compliance_test
  auto_start: true
  task_table: chassis_tasks.yaml

  steps:
    # 此处省略左移、后退等底盘步骤，完整顺序见配置文件。
    - action: arm_move
      control_mode: admittance
      frame_id: base_link
      end_effector_frame: tool0
      max_position_distance: 0.06
      arrival_tolerance: 0.005
      after_arrival_delay_sec: 2.0
      motion_timeout_sec: 60.0
      progress_timeout_sec: 10.0
      target_pose:
        position_mode: relative
        position: [0.0, 0.05, 0.0]
        orientation: current
```

执行顺序：

联合启动时导纳输出默认禁用，只有工作流执行到 `admittance` 步骤才开启。此流程不启动独立的 `moveit_pose_executor` 和 `staged_pose_coordinator`；力方向调试日志默认关闭，测量话题仍持续发布。

1. 依次调用底盘前进、左移、后退，各步等待 `running` 和 `completed`。
2. 等待 2 秒，核对末端起点和力传感器数据。
3. 选择 `admittance` 模式，读取本步骤开始时的末端位姿，将沿 `base_link +Y` 偏移 5 cm 的目标交给导纳控制器，保留当前末端姿态；允许的单次目标距离为 6 cm。控制器使用 MoveIt IK，并向 `scaled_joint_trajectory_controller` 持续发送关节轨迹，不调用独立的 MoveIt 规划执行服务。
4. 进入 5 mm 到位范围后保持导纳 2 秒，再进入后续步骤。当前默认配置的机械臂点位是最后一步，因此等待结束后流程完成，导纳继续保持。该步骤总超时 60 秒，连续无进展超时 10 秒。

## 支持的流程动作

| action | 参数 | 含义 |
|---|---|---|
| `chassis_execute` | `task` | 直接调用一个底盘任务 |
| `chassis_continue` | 无 | 继续暂停的底盘任务 |
| `chassis_stop` | 无 | 停止底盘任务 |
| `wait_chassis` | `task`, `require_running` | 等待指定任务完成 |
| `delay` | `seconds` | 等待指定秒数 |
| `arm_poses` | `frame_id`, `angles_in_degrees`, `poses` | 执行机械臂末端点位 |
| `arm_move` | `control_mode`, `target_pose` | 按指定控制模式执行一个末端目标 |

`arm_move.control_mode` 可选：

- `moveit`：经 MoveIt 规划并执行到目标点，服务返回后流程继续。
- `admittance`：把目标点交给 100 Hz 导纳控制器，持续依据六维力数据修正位置。流程通过 TF 等待末端进入 `arrival_tolerance` 指定的目标区域；完成后导纳控制器保持启用，继续依据外力柔顺控制。本示例的到位范围为 5 mm。连续 10 秒没有进展或总计 60 秒仍未进入目标区域时，流程会停止导纳并报错。

`target_pose.position_mode` 默认 `absolute`，此时 `position` 是 `frame_id` 下的绝对位置（米）；设置为 `relative` 时，`position` 是该坐标系中的偏移量，在每一步开始时加到当前末端位置上，随后固定该目标。`target_pose.orientation` 可写 `current`，保留执行时测得的末端姿态；也可写 `[roll, pitch, yaw]`，角度单位为度。`expected_start_position` 和 `max_position_distance` 用于限制起点误差和单次运动距离。已有的 `arm_poses` 动作继续支持多个 MoveIt 点位。

`arm_move.after_arrival_delay_sec` 设置到位后等待多少秒，默认 `0.0`。到位判定成功后开始计时，等待期间不会因导纳让位再次启动到位超时；计时结束后按 YAML 顺序执行下一步。后续底盘动作和 `delay` 步骤会保留导纳；后续 `arm_move` 选择 `admittance` 时更新目标点，选择 `moveit` 时先关闭导纳，再规划执行新目标。要继续运动，请在该机械臂步骤下面追加所需的底盘任务或机械臂点位。

## 对方消息

收到主机调用：

```json
{"version":1,"type":"control_command","sequence":10,"timestamp_ns":1790593000000000000,"source":"supervisor","event_id":"control-abc123","command":"execute_task","task":"forward_10cm"}
```

对方 ACK 后执行 `forward_10cm`，并依次上报：

```json
{"version":1,"type":"chassis_status","sequence":20,"timestamp_ns":1790593010000000000,"source":"robot","event_id":"forward-10cm-running-001","task":"forward_10cm","state":1,"state_name":"running","message":"task_started"}
```

```json
{"version":1,"type":"chassis_status","sequence":21,"timestamp_ns":1790593020000000000,"source":"robot","event_id":"forward-10cm-completed-001","task":"forward_10cm","state":2,"state_name":"completed","progress":1.0,"message":"target_reached"}
```

默认要求先收到 `running` 才接受 `completed`，防止旧终态误触发。

## 启动

```bash
cd ~/project/ls65_ros/ls65/eli_ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch eli_to_chassis_control joint_motion_test.launch.py
```

使用其他流程文件：

```bash
ros2 launch eli_to_chassis_control joint_motion_test.launch.py \
  workflow_file:=/absolute/path/to/workflow.yaml
```

禁止自动开始，改为服务启动：

```bash
ros2 launch eli_to_chassis_control joint_motion_test.launch.py auto_start:=false
ros2 service call /workflow_controller/start std_srvs/srv/Trigger "{}"
```

联合 launch 默认不会在启动时移动机械臂。底盘完成并等待结束后，工作流按 YAML 中的 `control_mode` 启动机械臂运动。

## 位移任务和顺序坐标任务

配置目录只保留 `workflow.yaml`（动作顺序）与 `chassis_tasks.yaml`（底盘调用名称）。在同一份流程里配置两种机械臂目标：

| position_mode | position 的含义 |
|---|---|
| `relative` | 每一步开始时读取当前末端 TF，加上 `frame_id` 下的位移，并固定目标 |
| `absolute` | `frame_id` 下的绝对坐标；多个 `arm_move` 步骤按列表顺序执行 |

默认机械臂步骤沿基座 +Y 移动 5 cm，保持当前姿态：

```yaml
target_pose:
  position_mode: relative
  position: [0.0, 0.05, 0.0]
  orientation: current
```

改成绝对坐标时，将 `position_mode` 改为 `absolute`，并填写现场确定的 `[x, y, z]`（米）。多个点就按顺序添加多个 `arm_move` 步骤；每个点到位、等待结束后才发布下一点。`max_position_distance` 应与点位距离相符。

位移重复启动会再次相对当前位姿移动，绝对坐标重复启动的目标不变。当前到位误差为 5 mm、到位延迟为 2 秒；任何步骤失败都会停止流程，最后一个机械臂步骤完成后保持导纳。
