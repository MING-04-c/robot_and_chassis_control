# `eli_cs_controllers` 代码导读

本文面向刚开始阅读 C++ 和 ROS 2 的开发者，解释这个包中的控制器文件、配置和执行过程。

## 1. 这个包在系统中的位置

`eli_cs_controllers` 是一个 `ros2_control` **控制器插件包**。它本身不打开 socket，也不直接给机器人发送 TCP 数据。硬件通信由 `eli_cs_robot_driver` 中的硬件驱动完成；本包在 `controller_manager` 调度下读写“硬件接口”。

一次控制周期可以概括为：

```text
硬件驱动 read()
    -> 将机器人最新状态写入 state interface
controller_manager
    -> 调用已激活控制器的 update(time, period)
控制器
    -> 读取 state_interfaces_，写入 command_interfaces_
硬件驱动 write()
    -> 取走 command interface 中的命令并发送给机器人
```

这里有两个容易混淆的词：

| 名称 | 含义 | 例子 |
| --- | --- | --- |
| 状态接口（state interface） | 硬件提供，控制器只读 | 当前关节位置、数字输入、速度缩放系数 |
| 命令接口（command interface） | 控制器写入，硬件驱动消费 | 关节目标位置、设置 IO、开始 Freedrive |

所有插件注册在 [controller_plugins.xml](../controller_plugins.xml)，`pluginlib` 根据其中的 `type` 在运行时创建 C++ 类。

## 2. 如何从配置找到代码

[elite_cs_controllers.yaml](../../eli_cs_robot_driver/config/elite_cs_controllers.yaml) 中的三层含义如下：

```yaml
controller_manager:
  ros__parameters:
    io_and_status_controller:
      type: eli_cs_controllers/GPIOController
```

1. `io_and_status_controller` 是控制器实例名，也是其私有话题/服务名称的一部分。
2. `type` 是插件名，对应 `controller_plugins.xml` 中的 `eli_cs_controllers/GPIOController`。
3. 下方同名的 `io_and_status_controller: ros__parameters:` 是传给此实例的参数。

`$(var tf_prefix)` 是 launch/xacro 展开时替换的前缀。单臂通常为空字符串；多臂时可为 `left_`。例如速度缩放接口最终会是：

```text
<tf_prefix>speed_scaling/speed_scaling_factor
```

该接口的声明在 [cs.ros2_control.xacro](../../eli_cs_robot_description/urdf/cs.ros2_control.xacro) 中。**配置的接口名、控制器申请的接口名、Xacro 声明的接口名必须完全一致。**

## 3. 所有控制器一览

| 配置实例 | C++ 类/文件 | 作用 |
| --- | --- | --- |
| `speed_scaling_state_broadcaster` | `SpeedScalingStateBroadcaster` / `src/speed_scaling_state_broadcaster.cpp` | 将硬件的速度缩放系数发布为话题 |
| `scaled_joint_trajectory_controller` | `ScaledJointTrajectoryController` / `src/scaled_joint_trajectory_controller.cpp` | 执行关节轨迹，并按实际速度缩放推进轨迹时间 |
| `io_and_status_controller` | `GPIOController` / `src/gpio_controller.cpp` | 发布 IO/工具/模式状态，提供设置 IO、速度、负载等服务 |
| `freedrive_controller` | `FreedriveController` / `src/freedrive_controller.cpp` | 通过 Bool 话题进入/退出拖动示教（Freedrive）模式 |

所有类都继承 `controller_interface::ControllerInterface`。最重要的生命周期函数是：

| 函数 | controller_manager 何时调用 | 在本包中的用途 |
| --- | --- | --- |
| `on_init()` | 创建控制器对象后 | 创建参数监听器、读取初始参数 |
| `*_interface_configuration()` | 配置/激活前 | 声明希望独占的状态/命令接口 |
| `on_configure()` | `unconfigured -> inactive` | 刷新参数，创建订阅者或发布者 |
| `on_activate()` | `inactive -> active` | 检查硬件就绪、缓存接口、开始对外工作 |
| `update()` | 每个控制周期 | 读取状态、计算并写命令，或发布消息 |
| `on_deactivate()` | `active -> inactive` | 停止对外服务、清理运行期资源 |

## 4. 速度缩放广播器

文件：`include/eli_cs_controllers/speed_scaling_state_broadcaster.hpp` 和 `src/speed_scaling_state_broadcaster.cpp`。

它只申请一个状态接口：`speed_scaling/speed_scaling_factor`，不申请命令接口。因此它是“广播器”，不会改变机器人状态。

`update()` 中读取 `state_interfaces_[0]`。这里下标 0 安全的前提是：`state_interface_configuration()` 只放入了这一个接口。读取的硬件值通常为 `0.0 ~ 1.0`，代码再乘以 `100.0` 后发布 `std_msgs/msg/Float64` 到私有话题 `~/speed_scaling`，常见完整名称为：

```text
/speed_scaling_state_broadcaster/speed_scaling
```

配置项：

```yaml
speed_scaling_state_broadcaster:
  ros__parameters:
    state_publish_rate: 100.0  # 目标发布频率，单位 Hz
    tf_prefix: "$(var tf_prefix)"
```

注意：现有 README 写话题值为 `0~1`，但当前实现乘了 `100.0`，实际发布值按百分比解释更准确。例如硬件值 `0.5` 会发布 `50.0`。

## 5. 缩放关节轨迹控制器

文件：`include/eli_cs_controllers/scaled_joint_trajectory_controller.hpp` 和 `src/scaled_joint_trajectory_controller.cpp`。

它继承 ROS 2 官方的 `joint_trajectory_controller::JointTrajectoryController`。父类已经实现了接收 `FollowJointTrajectory` Action、轨迹插值、PID、容差判断和关节命令写入；本类主要改变的是“轨迹时间如何推进”。

普通轨迹控制器按真实经过的时间采样。假设轨迹要求 2 秒完成，即使示教器速度滑块只有 50%，它仍会在第 2 秒采样到终点，造成期望位置远超实际位置。本控制器在 `update()` 中读取 `scaling_factor_` 并将本周期时间增量乘以该系数：

```text
真实周期：10 ms
缩放系数：0.5
轨迹虚拟时钟前进：5 ms
```

当系数为 0 时，虚拟时钟不前进，轨迹自然暂停；恢复为正数后继续。`TimeData` 保存真实时刻、缩放后的周期与累计虚拟时间。`RealtimeBuffer<TimeData>` 用于实时控制循环安全地持有这些数据，避免在 `update()` 中使用常规锁。

主要配置沿用官方轨迹控制器：

```yaml
scaled_joint_trajectory_controller:
  ros__parameters:
    joints: [六个关节名]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    constraints:             # 轨迹/终点误差阈值
      goal_time: 0.0
    speed_scaling_interface_name: $(var tf_prefix)speed_scaling/speed_scaling_factor
```

`joints` 的顺序很重要：轨迹消息中的数组、父类内部状态数组和命令接口数组都以此顺序对应。`constraints.<joint>.trajectory` 是运动中允许误差，`goal` 是到终点时允许误差。

## 6. GPIO 与状态控制器

文件：`include/eli_cs_controllers/gpio_controller.hpp` 和 `src/gpio_controller.cpp`。

这是功能最多的控制器，承担两条方向相反的数据流：

```text
机器人状态接口 -> update() -> ROS 话题
ROS 服务请求     -> command_interfaces_ -> 硬件驱动
```

### 状态发布

每次 `update()` 调用以下函数：

| 函数 | 发布内容 | 私有话题 |
| --- | --- | --- |
| `publishIO()` | 标准/配置/工具数字 IO，标准模拟 IO | `~/io_states` |
| `publishToolData()` | 工具模式、电压、电流、温度、模拟 IO | `~/tool_data` |
| `publishRobotMode()` | 机器人模式，只有变化时发布 | `~/robot_mode` |
| `publishSafetyMode()` | 安全模式，只有变化时发布 | `~/safety_mode` |
| `publishTaskRunning()` | 程序是否运行，只有变化时发布 | `~/robot_task_running` |

`StateOffset` 是状态接口数组的下标表。例如它声明标准数字输出 16 个、配置数字输出 8 个、工具数字输出 4 个，随后才声明数字输入。因此 `StateOffset::STANDARD_DIG_IN` 就是输入分组的起始位置。这种写法性能直接，但**绝不能随意改变 `state_interface_configuration()` 中 `emplace_back()` 的顺序**。

### 服务命令与异步确认

激活后，该控制器提供：

| 服务 | 功能 |
| --- | --- |
| `~/set_io` | 设置数字/模拟输出或工具电压 |
| `~/set_speed_slider` | 设置速度滑块比例，合法范围为 `0.01~1.0` |
| `~/resend_external_script` | 让驱动重新发送 ExternalControl 脚本 |
| `~/hand_back_control` | 请求交还控制权给机器人程序 |
| `~/set_payload` | 设置负载质量与重心 |
| `~/zero_ftsensor` | 力/力矩传感器置零 |

服务不会直接等待网络响应，而是统一采用：

```text
1. 将 xxx_async_success 命令接口写为 2.0（ASYNC_WAITING）
2. 将实际命令值写入相应命令接口
3. 硬件驱动处理命令后改写 xxx_async_success
4. waitForAsyncCommand() 每 50 ms 轮询，超过 check_io_successfull_retries 后失败
```

GPIO 参数：

```yaml
io_and_status_controller:
  ros__parameters:
    tf_prefix: "$(var tf_prefix)"
    # 可选；未填写时使用参数定义文件中的默认值 10
    check_io_successfull_retries: 10
```

## 7. Freedrive 控制器

文件：`include/eli_cs_controllers/freedrive_controller.hpp` 和 `src/freedrive_controller.cpp`。

它申请三个命令接口：

```text
freedrive_mode/freedrive_async_success
freedrive_mode/freedrive_start_cmd
freedrive_mode/freedrive_end_cmd
```

它订阅私有话题 `~/enable_freedrive`（`std_msgs/msg/Bool`）。完整名称通常为 `/freedrive_controller/enable_freedrive`。收到 `true` 时请求进入 Freedrive；收到 `false` 时请求退出。

订阅回调并不直接执行 `set_value()`。回调只修改两个 `std::atomic<bool>`：`is_freedrive_active_` 表示目标状态，`is_new_request_` 表示有待处理请求。随后实时 `update()` 消费该请求并写 `freedrive_start_cmd` 或 `freedrive_end_cmd`。这种分工避免 ROS 回调线程与控制循环并发写硬件接口。

`inactive_timeout` 是看门狗秒数，默认值为 1。开始接收消息后，每条消息都会重置定时器；超时未收到任何消息会自动请求退出。连续发布 `true` 才能持续保持 Freedrive。

```yaml
freedrive_controller:
  ros__parameters:
    tf_prefix: "$(var tf_prefix)"
    inactive_timeout: 1  # 秒；可选，默认 1
```

Freedrive 前需要停用会占用关节命令接口的位置/速度控制器，否则 controller_manager 会因接口资源冲突拒绝激活。

## 8. C++ 阅读顺序建议

1. 先看 `controller_plugins.xml`，理解插件名和 C++ 类的映射。
2. 再看 `elite_cs_controllers.yaml`，确认运行时创建了哪些实例、传了哪些参数。
3. 对照 `cs.ros2_control.xacro`，确认每个接口由硬件暴露。
4. 阅读每个头文件的成员变量和接口下标枚举。
5. 最后读 `.cpp`：先生命周期函数，再 `update()`，最后服务/订阅回调。

这个顺序比直接从 `update()` 开始读更容易建立完整数据流。

## 9. 当前实现的注意点

以下是阅读源码时发现的实现特征。它们在本文档中记录以方便维护；本文档和源码注释没有改变原有行为。

1. `GPIOController::setIO()` 的数字 IO 引脚检查使用 `pin <= 数量`，但标准数字 IO 的有效下标应为 `0~15`。例如标准 IO 的 `pin == 16` 会越过标准输出分组并写入后续接口。模拟输出分支也使用了标准数字 IO 数量作为上限。调用服务时应只传入硬件实际定义的合法引脚编号，后续维护建议将边界条件复核为 `< 数量`。
2. `SpeedScalingStateBroadcaster::update()` 用单次控制周期 `period > 1 / publish_rate` 决定是否发布，而不是累计已过时间。若控制周期短于发布间隔，例如控制周期为 2 ms、发布频率为 100 Hz（10 ms），该条件可能始终不成立。应在实际控制频率下通过 `ros2 topic echo` 验证话题是否发布。
3. `ScaledJointTrajectoryController` 通过 `state_interfaces_.back()` 获取速度缩放接口。这依赖父类申请的接口之后恰好追加缩放接口；新增接口时要保持这一顺序，或改成按接口名称查找。
4. `FreedriveController::on_deactivate()` 只清除标志和定时器，不发送 `freedrive_end_cmd`；退出命令在 `on_cleanup()` 或收到 `false` 话题后由 `update()` 写入。使用生命周期切换停止 Freedrive 时应结合实际硬件行为进行验证。
