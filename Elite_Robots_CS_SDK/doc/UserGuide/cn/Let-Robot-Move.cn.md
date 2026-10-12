[Home](./UserGuide.cn.md)

# 让机器人动起来

## 目标
调用SDK的相关接口，让机器人以当前位姿为起点，末端运行一个三角形轨迹。

## 背景说明
SDK 控制机器人的方式是：通过插件或30001发送一个控制脚本给机器人运行，这个控制脚本会通过TCP/IP协议与SDK连接，并接收控制指令。

在继续阅读此章节之前，请确保已经了解了 [`DashboardClient`](./Power-on-Robot.cn.md) 和 [`RtsiIOInterface`](./Get-Robot-State.cn.md) 两个类的使用。

## 任务

### 1. 封装轨迹运动的接口

创建一个代码文件：
```bash
touch trajectory_example.cpp
```

复制下面代码，使用你习惯的文本编辑器，粘贴到`trajectory_example.cpp`中：

```cpp
#include <Elite/DataType.hpp>
#include <Elite/EliteDriver.hpp>
#include <Elite/Log.hpp>
#include <Elite/DashboardClient.hpp>

#include <future>
#include <iostream>
#include <memory>
#include <thread>

using namespace ELITE;

class TrajectoryControl {
   private:
    std::unique_ptr<EliteDriver> driver_;
    
    std::unique_ptr<DashboardClient> dashboard_;
    EliteDriverConfig config_;

   public:
    TrajectoryControl(const EliteDriverConfig& config) {
        config_ = config;
        driver_ = std::make_unique<EliteDriver>(config);
        dashboard_ = std::make_unique<DashboardClient>();

        ELITE_LOG_INFO("Connecting to the dashboard");
        if (!dashboard_->connect(config.robot_ip)) {
            ELITE_LOG_FATAL("Failed to connect to the dashboard.");
            throw std::runtime_error("Failed to connect to the dashboard.");
        }
        ELITE_LOG_INFO("Successfully connected to the dashboard");
    }

    ~TrajectoryControl() {
        if (dashboard_) {
            dashboard_->disconnect();
        }
        driver_->stopControl();
    }

    bool startControl() {
        ELITE_LOG_INFO("Start powering on...");
        if (!dashboard_->powerOn()) {
            ELITE_LOG_FATAL("Power-on failed");
            return false;
        }
        ELITE_LOG_INFO("Power-on succeeded");

        ELITE_LOG_INFO("Start releasing brake...");
        if (!dashboard_->brakeRelease()) {
            ELITE_LOG_FATAL("Brake release failed");
            return false;
        }
        ELITE_LOG_INFO("Brake released");

        if (config_.headless_mode) {
            if (!driver_->isRobotConnected()) {
                if (!driver_->sendExternalControlScript()) {
                    ELITE_LOG_FATAL("Fail to send external control script");
                    return false;
                }
            }
        } else {
            if (!dashboard_->playProgram()) {
                ELITE_LOG_FATAL("Fail to play program");
                return false;
            }
        }

        ELITE_LOG_INFO("Wait external control script run...");
        while (!driver_->isRobotConnected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ELITE_LOG_INFO("External control script is running");
        return true;
    }

    bool moveTrajectory(const std::vector<vector6d_t>& target_points, float point_time, float blend_radius, bool is_cartesian) {
        std::promise<TrajectoryMotionResult> move_done_promise;
        driver_->setTrajectoryResultCallback([&](TrajectoryMotionResult result) { move_done_promise.set_value(result); });

        ELITE_LOG_INFO("Trajectory motion start");
        if(!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::START, target_points.size(), 200)) {
            ELITE_LOG_ERROR("Failed to start trajectory motion");
            return false;
        }

        for (const auto& joints : target_points) {
            if (!driver_->writeTrajectoryPoint(joints, point_time, blend_radius, is_cartesian)) {
                ELITE_LOG_ERROR("Failed to write trajectory point");
                return false;
            }
            // Send NOOP command to avoid timeout.
            if(!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200)) {
                ELITE_LOG_ERROR("Failed to send NOOP command");
                return false;
            }
        }

        std::future<TrajectoryMotionResult> move_done_future = move_done_promise.get_future();
        while (move_done_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
            // Wait for the trajectory motion to complete, and send NOOP command to avoid timeout.
            if(!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200)) {
                ELITE_LOG_ERROR("Failed to send NOOP command");
                return false;
            }
        }
        auto result = move_done_future.get();
        ELITE_LOG_INFO("Trajectory motion completed with result: %d", result);

        if(!driver_->writeIdle(0)) {
            ELITE_LOG_ERROR("Failed to write idle command");
            return false;
        }

        return result == TrajectoryMotionResult::SUCCESS;
    }

    bool moveTo(const vector6d_t& point, float time, bool is_cartesian) {
        return moveTrajectory({point}, time, 0, is_cartesian);
    }
};
```

### 2. 代码解析
`TrajectoryControl` 是对 `EliteDriver` 和 `DashboardClient` 的进一步封装，把 SDK 的 API 组织成易用的流程。

`EliteDriver`这个类主要负责与控制脚本通讯，并且集成了一些常用接口进去。

本章节主要用到`EliteDriver`中 `setTrajectoryResultCallback()`、`writeTrajectoryPoint()`、`writeTrajectoryControlAction()`三个接口来让机器人执行“movej”和“movel”运动，调用流程如下：
- 调用`setTrajectoryResultCallback()`注册运动结果的回调函数，在机器人结束了这段轨迹的运动后，会调用这个回调，并告知运动结果。
- 调用`writeTrajectoryControlAction()`写入start命令
- 调用`writeTrajectoryPoint()`将点位与参数发送过去，与此同时调用`writeTrajectoryControlAction()`发送noop指令来防止超时。
- 持续调用`writeTrajectoryControlAction()`发送noop指令并等待运动结果。
- 期间如果想要取消运动，可以调用`writeTrajectoryControlAction()`发送cancel指令

---

#### 2.1 成员变量 & 构造 / 析构

##### ***2.1.1 成员变量***

```cpp
std::unique_ptr<EliteDriver> driver_;
std::unique_ptr<DashboardClient> dashboard_;
EliteDriverConfig config_;
```

* **`driver_`**
  用于发送轨迹控制命令、设置回调、管理外部控制脚本等
* **`dashboard_`**
  用于做机器人开机、松刹车、启动程序等“面板”功能
* **`config_`**
  保存配置（机器人 IP、模式、端口等）

---

##### ***2.1.2 构造函数***

```cpp
TrajectoryControl(const EliteDriverConfig& config) {
    config_ = config;
    driver_ = std::make_unique<EliteDriver>(config);
    dashboard_ = std::make_unique<DashboardClient>();

    ELITE_LOG_INFO("Connecting to the dashboard");
    if (!dashboard_->connect(config.robot_ip)) {
        ELITE_LOG_FATAL("Failed to connect to the dashboard.");
        throw std::runtime_error("Failed to connect to the dashboard.");
    }
    ELITE_LOG_INFO("Successfully connected to the dashboard");
}
```

* 保存 `config`
* 创建 `driver_`（使用 config 进行构造）
* 创建 `dashboard_`（默认构造）
* 立即连接 Dashboard，连接失败就直接抛异常中断程序

---

##### ***2.1.3 析构函数***

```cpp
~TrajectoryControl() {
    if (dashboard_) {
        dashboard_->disconnect();
    }
    driver_->stopControl();
}
```

* 如果 Dashboard 已连接 → 断开
* 停止轨迹控制（保证机器人退出控制模式）

---

#### 2.2 核心方法的逻辑

##### 2.2.1 ***`startControl()` — 启动机器人控制流程***

```cpp
bool startControl() {
    // 1. 上电
    if (!dashboard_->powerOn()) return false;

    // 2. 松刹车
    if (!dashboard_->brakeRelease()) return false;

    // 3. 启动外部控制脚本（分 headless / 非 headless）
    if (config_.headless_mode) {
        if (!driver_->isRobotConnected()) {
            if (!driver_->sendExternalControlScript()) return false;
        }
    } else {
        if (!dashboard_->playProgram()) return false;
    }

    // 4. 等待机器人进入外部控制模式
    while (!driver_->isRobotConnected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
}
```

流程：

1. **上电**
2. **松刹车**
3. **根据模式启动外部控制**
   * headless → 直接发送外部控制脚本
   * 非 headless → 通过 Dashboard 启动已加载的控制程序
4. 等待机器人连接到外部控制接口（轮询检测）

---

##### 2.2.3 ***`moveTrajectory()` — 执行多点轨迹运动***

```cpp
bool moveTrajectory(const std::vector<vector6d_t>& target_points, float point_time, float blend_radius, bool is_cartesian) {
        std::promise<TrajectoryMotionResult> move_done_promise;
        driver_->setTrajectoryResultCallback([&](TrajectoryMotionResult result) { move_done_promise.set_value(result); });

        ELITE_LOG_INFO("Trajectory motion start");
        if(!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::START, target_points.size(), 200)) {
            ELITE_LOG_ERROR("Failed to start trajectory motion");
            return false;
        }

        for (const auto& points : target_points) {
            ELITE_LOG_INFO("Moving joints to target: [%lf, %lf, %lf, %lf, %lf, %lf]", points[0], points[1], points[2], points[3], points[4], points[5]);

            if (!driver_->writeTrajectoryPoint(points, point_time, blend_radius, is_cartesian)) {
                ELITE_LOG_ERROR("Failed to write trajectory point");
                return false;
            }
            // Send NOOP command to avoid timeout.
            if(!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200)) {
                ELITE_LOG_ERROR("Failed to send NOOP command");
                return false;
            }
        }

        std::future<TrajectoryMotionResult> move_done_future = move_done_promise.get_future();
        while (move_done_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
            // Wait for the trajectory motion to complete, and send NOOP command to avoid timeout.
            if(!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200)) {
                ELITE_LOG_ERROR("Failed to send NOOP command");
                return false;
            }
        }

        auto result = move_done_future.get();
        ELITE_LOG_INFO("Trajectory motion completed with result: %d", result);

        if(!driver_->writeIdle(0)) {
            ELITE_LOG_ERROR("Failed to write idle command");
            return false;
        }

        auto cancel_done_future = cancel_done_promise.get_future();
        if (cancel_done_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            ELITE_LOG_ERROR("Failed to wait for cancel done");
            return false;
        }

        return result == TrajectoryMotionResult::SUCCESS;
    }
```

这个函数的功能是：发送多个轨迹点，机器人按顺序运动。

执行逻辑：

1. 调用`setTrajectoryResultCallback()`注册运动结果的回调函数，在回调函数中，用 `std::promise` / `std::future` 等待异步回调的运动完成信号。
   
   ```cpp
    std::promise<TrajectoryMotionResult> move_done_promise;
    driver_->setTrajectoryResultCallback([&](TrajectoryMotionResult result) { move_done_promise.set_value(result); });
   ```

2. 发送轨迹开始命令，以及轨迹的点位数量和指令超时时间（200ms）。

    ```cpp
    driver_->writeTrajectoryControlAction(START, target_points.size(), 200);
    ```
> 指令超时时间：如果超时时间内没有发送下一条指令，则会触发超时。例如这里的200ms，如果200ms内没有再次调用`writeTrajectoryControlAction()`发送指令，则会触发超时。

3. 发送轨迹点位(缩减了日志打印的代码)

    ```cpp
    for (const auto& points : target_points) {
        if (!driver_->writeTrajectoryPoint(points, point_time, blend_radius, is_cartesian)) {
            return false;
        }

        if(!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200)) {
            return false;
        }
    }
    ```

   * 调用 `writeTrajectoryPoint()` 发送关节角 / 笛卡尔位姿
   * 每发一个点，立即发一个 `NOOP`（防止通信超时）

4. 等待轨迹完成
    ```cpp
    std::future<TrajectoryMotionResult> move_done_future = move_done_promise.get_future();
    while (move_done_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
        if(!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200)) {
            return false;
        }
    }
    ```

   * 循环检查 `future` 是否被赋值
   * 每次等待期间继续发 `NOOP` 防止超时

5. 运动结束，让机器人回归空闲

    ```cpp
    if(!driver_->writeIdle(0)) {
        ELITE_LOG_ERROR("Failed to write idle command");
        return false;
    }
    ```
    如果直接调用`writeIdle()`接口，则还会触发一次回调（退出了轨迹运动模式会触发一次轨迹运动的CANCEL信号），由于`move_done_promise`在先前轨迹运动到位后已经被赋值了，因此这里需要再注册一个回调来接收CANCEL信号。
    使用`writeIdle()`让机器人进入空闲（即不执行任何控制指令），这里的超时时间设置为0代表了无限时间。
    最后等待10秒`future`被赋值。

6. **返回运动是否成功**

   * 判断结果是否为 `TrajectoryMotionResult::SUCCESS`

---

##### 2.2.4 ***`moveTo()` — 单点运动***

```cpp
bool moveTo(const vector6d_t& point, float time, bool is_cartesian) {
    return moveTrajectory({point}, time, 0, is_cartesian);
}
```

就是 `moveTrajectory` 的单点版本，`blend_radius=0`，轨迹只有一个点。

---

### 3. 使用TrajectoryControl
在`trajectory_example.cpp`的头文件声明处加入头文件`#include <Elite/RtsiIOInterface.hpp>`

在`trajectory_example.cpp`文件的最后加入下面的内容：
```cpp
int main(int argc, const char** argv) {
    EliteDriverConfig config;
    if (argc == 2) {
        config.robot_ip = argv[1];
    } else if (argc == 3) {
        config.robot_ip = argv[1];
        config.local_ip = argv[2];
    } else {
        std::cout << "Must provide robot IP. Example: ./trajectory_example aaa.bbb.ccc.ddd <eee.fff.ggg.hhh>" << std::endl;
        return 1;
    }
    config.headless_mode = true;
    config.script_file_path = "external_control.script";
    std::unique_ptr<TrajectoryControl> trajectory_control = std::make_unique<TrajectoryControl>(config);
    std::unique_ptr<RtsiIOInterface> rtsi_client = std::make_unique<RtsiIOInterface>("output_recipe.txt", "input_recipe.txt", 250);

    ELITE_LOG_INFO("Connecting to the RTSI");
    if (!rtsi_client->connect(config.robot_ip)) {
        ELITE_LOG_FATAL("Fail to connect or config to the RTSI.");
        throw std::runtime_error("Fail to connect or config to the RTSI");
    }
    ELITE_LOG_INFO("Successfully connected to the RTSI");

    ELITE_LOG_INFO("Starting trajectory control...");
    if(!trajectory_control->startControl()) {
        ELITE_LOG_FATAL("Failed to start trajectory control.");
        return 1;
    }
    ELITE_LOG_INFO("Trajectory control started");

    vector6d_t actual_joints = rtsi_client->getActualJointPositions();
    actual_joints[3] = -1.57;

    ELITE_LOG_INFO("Moving joints to target: [%lf, %lf, %lf, %lf, %lf, %lf]",
                   actual_joints[0], actual_joints[1], actual_joints[2], actual_joints[3], actual_joints[4], actual_joints[5]);
    if(!trajectory_control->moveTo(actual_joints, 3, false)) {
        ELITE_LOG_FATAL("Failed to move joints to target.");
        return 1;
    }
    ELITE_LOG_INFO("Joints moved to target");


    vector6d_t actual_pose = rtsi_client->getActualTCPPose();
    std::vector<vector6d_t> trajectory;

    actual_pose[2] -= 0.2;
    trajectory.push_back(actual_pose);


    actual_pose[1] -= 0.2;
    trajectory.push_back(actual_pose);

    actual_pose[1] += 0.2;
    actual_pose[2] += 0.2;
    trajectory.push_back(actual_pose);

    ELITE_LOG_INFO("Moving joints to target");
    if(!trajectory_control->moveTrajectory(trajectory, 3, 0, true)) {
        ELITE_LOG_FATAL("Failed to move trajectory.");
        return 1;
    }
    ELITE_LOG_INFO("Joints moved to target");

    return 0;
}
```

创建RTSI配方文件：
```bash
touch output_recipe.txt input_recipe.txt
```

复制下面的文本内容，使用你习惯的文本编辑器，粘贴到`output_recipe.txt`中：
```
timestamp
payload_mass
payload_cog
script_control_line
target_joint_positions
target_joint_speeds
actual_joint_torques
actual_joint_positions
actual_joint_speeds
actual_joint_current
actual_TCP_pose
actual_TCP_speed
actual_TCP_force
target_TCP_pose
target_TCP_speed
actual_digital_input_bits
actual_digital_output_bits
joint_temperatures
robot_mode
joint_mode
safety_status
speed_scaling
target_speed_fraction
actual_robot_voltage
actual_robot_current
runtime_state
elbow_position
robot_status_bits
safety_status_bits
analog_io_types
standard_analog_input0
standard_analog_input1
standard_analog_output0
standard_analog_output1
io_current
tool_mode
tool_analog_input_types
tool_analog_output_types
tool_analog_input
tool_analog_output
tool_output_voltage
tool_output_current
tool_temperature
tool_digital_mode
tool_digital0_mode
tool_digital1_mode
tool_digital2_mode
tool_digital3_mode
output_bit_registers0_to_31
output_bit_registers32_to_63
input_bit_registers0_to_31
input_bit_registers32_to_63
```


复制下面的文本内容，使用你习惯的文本编辑器，粘贴到`input_recipe.txt`中：
```
speed_slider_mask
speed_slider_fraction
standard_digital_output_mask
standard_digital_output
configurable_digital_output_mask
configurable_digital_output
tool_digital_output_mask
tool_digital_output
standard_analog_output_mask
standard_analog_output_type
standard_analog_output_0
standard_analog_output_1
external_force_torque
input_bit_registers0_to_31
input_bit_registers32_to_63
```

拷贝`external_control.script`控制脚本文件到当前目录下（注意替换/your/path/）：
```bath
cp /your/path/external_control.script ./
```

> tips： `external_control.script`的位置：
> - 如果是编译安装的，可以在源码项目中的`Elite_Robots_CS_SDK/source/resources/`路径下找到，亦或者，没有设置过安装路径的前提下，通常在`/usr/local/share/Elite/`路径下。
> - 如果使用apt安装通常会在`/usr/share/Elite/`路径下。


### 4. 解析

#### 4.1 新增代码

新增代码的功能是，使用`RtsiIOInterface`获取当前机器人的关节角度、位姿，并以当前的位姿为起始点进行运动。

1. 从命令行获取机器人IP与本机IP（如果需要的话）。
    ```cpp
    EliteDriverConfig config;
    if (argc == 2) {
        config.robot_ip = argv[1];
    } else if (argc == 3) {
        config.robot_ip = argv[1];
        config.local_ip = argv[2];
    } else {
        std::cout << "Must provide robot IP. Example: ./trajectory_example aaa.bbb.ccc.ddd <eee.fff.ggg.hhh>" << std::endl;
        return 1;
    }
    config.headless_mode = true;
    config.script_file_path = "external_control.script";
    ```

如前文中[“背景说明”](#背景说明)所述，控制脚本会与SDK建立TCP连接，因此本质上SDK是需要本机IP地址的（用于替换控制脚本中`socket_open()`的IP地址），然而 `EliteDriver` 在构造时会尝试获取本机IP，所以不需要手动去设置`config`的`local_ip`。但是，某些情况下可能需要单独设置本机IP，例如，你使用了NAT配置的虚拟机，并且配置好了虚拟机与宿主机50001~50004的端口转发（SDK与控制脚本通讯的默认端口），那么此时就需要特别指定`local_ip`为你的宿主机IP。

`config.headless_mode = true;`设置启用了无界面模式，此模式下会发送控制脚本给机器人的主端口（30001），机器人便会运行这个脚本。
> note：在`EliteDriver`的构造函数中会自动发送一次，如果此时机器人还未释放抱闸，则需要在释放抱闸后，调用`sendExternalControlScript()`方法再次发送控制脚本。

`config.script_file_path`指定了外部控制脚本的路径。


2. 获取当前关节角度，并让第四关节旋转90度。
    ```cpp
    vector6d_t actual_joints = rtsi_client->getActualJointPositions();
    actual_joints[3] = -1.57;

    if(!trajectory_control->moveTo(actual_joints, 3, false)) {
        return 1;
    }
    ```

3. 获取当前TCP位姿，规划一个三角形轨迹，并运行
    ```cpp
    vector6d_t actual_pose = rtsi_client->getActualTCPPose();
    std::vector<vector6d_t> trajectory;

    actual_pose[2] -= 0.2;
    trajectory.push_back(actual_pose);


    actual_pose[1] -= 0.2;
    trajectory.push_back(actual_pose);

    actual_pose[1] += 0.2;
    actual_pose[2] += 0.2;
    trajectory.push_back(actual_pose);

    if(!trajectory_control->moveTrajectory(trajectory, 3, 0, true)) {
        return 1;
    }
    ```

#### 4.2 RTSI配方文件与控制脚本

通过[获取机器人状态](./Get-Robot-State.cn.md)章节可得知，`RtsiIOInterface`需要配方文件来决定读写机器人的那些状态或数据。

通过[“背景说明”](#背景说明)可得知SDK的控制逻辑是，通过控制脚本进行TCP/IP协议与SDK连接，并发送控制指令。

### 5. 编译与运行
使用gcc编译器，编译`trajectory_example.cpp`：
```bash
g++ trajectory_example.cpp -o trajectory_example -lelite-cs-series-sdk
```

运行：
```bash
./trajectory_example <your robot ip> <your local ip(option)>
```
运行后机器人应当会上电释放抱闸，四关节旋转90度后，运行一个三角形轨迹（可留意示教器界面，因为可能会出现奇异位姿的情况）。

---

[>>>下一章：透传:servoj](./Servoj-Move.cn.md)

