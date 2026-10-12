[Home](./UserGuide.cn.md)

# 透传:servoj

## 目标
调用SDK的相关接口，让机器人以当前位姿为起点，末端正反转。

## 背景说明
SDK 控制机器人的方式是：通过插件或30001发送一个控制脚本给机器人运行，这个控制脚本会通过TCP/IP协议与SDK连接，并接收控制指令。

Elite 机器人的servoj指令用于实时控制机器人的关节位置，在前瞻时间内利用时间间隔处理接收到的关节角度，并进行均值滤波，再将滤波的数据进行样条拟合，从而得到实时控制所需的关节位置。SDK提供了此指令的控制接口：`writeServoj()`.

在继续阅读此章节之前，请确保已经了解了 [`DashboardClient`](./Power-on-Robot.cn.md) 和 [`RtsiIOInterface`](./Get-Robot-State.cn.md) 两个类的使用。

## 任务

### 1. 写一个让机器人末端关节正反转的程序

创建一个代码文件：
```bash
touch servoj_example.cpp
```

复制下面代码，使用你习惯的文本编辑器，粘贴到`servoj_example.cpp`中：
```cpp
#include <Elite/DashboardClient.hpp>
#include <Elite/DataType.hpp>
#include <Elite/EliteDriver.hpp>
#include <Elite/Log.hpp>
#include <Elite/RtsiIOInterface.hpp>
#include <Elite/RtUtils.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>

#include <sys/mman.h>

using namespace ELITE;
using namespace std::chrono;

static std::unique_ptr<EliteDriver> s_driver;
static std::unique_ptr<RtsiIOInterface> s_rtsi_client;
static std::unique_ptr<DashboardClient> s_dashboard;

int main(int argc, char** argv) {
    mlockall(MCL_CURRENT | MCL_FUTURE);
    pthread_t handle = pthread_self();
    RT_UTILS::setThreadFiFoScheduling(handle, RT_UTILS::getThreadFiFoMaxPriority());
    RT_UTILS::bindThreadToCpus(handle, 2);

    EliteDriverConfig config;
    if (argc == 2) {
        config.robot_ip = argv[1];
    } else if (argc == 3) {
        config.robot_ip = argv[1];
        config.local_ip = argv[2];
    } else {
        std::cout << "Must provide robot IP. Example: ./servoj_example aaa.bbb.ccc.ddd <eee.fff.ggg.hhh>" << std::endl;
        return 1;
    }
    config.headless_mode = true;
    config.script_file_path = "external_control.script";
    config.servoj_time = 0.004;
    s_driver = std::make_unique<EliteDriver>(config);
    s_rtsi_client = std::make_unique<RtsiIOInterface>("output_recipe.txt", "input_recipe.txt", 250);
    s_dashboard = std::make_unique<DashboardClient>();
    ELITE_LOG_INFO("Connecting to the dashboard");
    if (!s_dashboard->connect(config.robot_ip)) {
        ELITE_LOG_FATAL("Failed to connect to the dashboard.");
        return 1;
    }
    ELITE_LOG_INFO("Successfully connected to the dashboard");

    ELITE_LOG_INFO("Connecting to the RTSI");
    if (!s_rtsi_client->connect(config.robot_ip)) {
        ELITE_LOG_FATAL("Failed to connect to the RTSI.");
        return 1;
    }
    ELITE_LOG_INFO("Successfully connected to the RTSI");

    VersionInfo version = s_rtsi_client->getControllerVersion();
    ELITE_LOG_INFO("Controller version is %s", version.toString().c_str());

    ELITE_LOG_INFO("Start powering on...");
    if (!s_dashboard->powerOn()) {
        ELITE_LOG_FATAL("Power-on failed");
        return 1;
    }
    ELITE_LOG_INFO("Power-on succeeded");

    ELITE_LOG_INFO("Start releasing brake...");
    if (!s_dashboard->brakeRelease()) {
        ELITE_LOG_FATAL("Release brake fail");
        return 1;
    }
    ELITE_LOG_INFO("Brake released");

    if (!s_driver->isRobotConnected()) {
        if (!s_driver->sendExternalControlScript()) {
            ELITE_LOG_FATAL("Fail to send external control script");
            return 1;
        }
    }

    ELITE_LOG_INFO("Wait external control script run...");
    while (!s_driver->isRobotConnected()) {
        std::this_thread::sleep_for(10ms);
    }
    ELITE_LOG_INFO("External control script is running");

    bool positive_rotation = false;
    bool negative_rotation = false;
    vector6d_t actual_joint;
    vector6d_t target_joint;
    double increment = 0;
    bool first_point = true;
    auto next = steady_clock::now();
    while (!(positive_rotation && negative_rotation)) {
        actual_joint = s_rtsi_client->getActualJointPositions();
        // If first point init target_joint
        if (first_point) {
            target_joint = actual_joint;
            first_point = false;
        }

        // Set the increment of positive rotation and negative rotation
        if (positive_rotation == false) {
            increment = 0.0005;
            if (actual_joint[5] >= 3) {
                positive_rotation = true;
            }
        } else if (negative_rotation == false) {
            increment = -0.0005;
            if (actual_joint[5] <= -3) {
                negative_rotation = true;
            }
        }
        target_joint[5] += increment;

        if (!s_driver->writeServoj(target_joint, 100)) {
            ELITE_LOG_FATAL("Send servoj command to robot fail");
            return 1;
        }
        next += 4ms;
        std::this_thread::sleep_until(next);
    }
    ELITE_LOG_INFO("Motion finish");
    s_driver->stopControl();

    return 0;
}

```

### 2. 代码解析
这段代码演示了如何连接机器人、启动外部控制脚本，并通过 RTSI 接口实时驱动关节运动，直到满足一定的旋转条件。


#### 2.1 主函数流程

```cpp
mlockall(MCL_CURRENT | MCL_FUTURE);
pthread_t handle = pthread_self();
RT_UTILS::setThreadFiFoScheduling(handle, RT_UTILS::getThreadFiFoMaxPriority());
RT_UTILS::bindThreadToCpus(handle, 2);
```

`mlockall(MCL_CURRENT | MCL_FUTURE);` 函数的调用是为了锁定内存，避免实时任务被换出到交换区。

SDK 中提供了一些实时线程的工具，这里调用`RT_UTILS::setThreadFiFoScheduling()`方法设置了线程的优先级（此处使用`RT_UTILS::getThreadFiFoMaxPriority()`方法将线程设置到了最高优先级）。使用`RT_UTILS::bindThreadToCpus()`方法设置了线程与CPU的亲和性。

实时线程的设置有利于更加平滑地控制机器人。

---

#### 2.2 解析命令行参数**

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
config.servoj_time = 0.004;
```

如前文中[“背景说明”](#背景说明)所述，控制脚本会与SDK建立TCP连接，因此本质上SDK是需要本机IP地址的（用于替换控制脚本中`socket_open()`的IP地址），然而 `EliteDriver` 在构造时会尝试获取本机IP，所以不需要手动去设置`config`的`local_ip`。但是，某些情况下可能需要单独设置本机IP，例如，你使用了NAT配置的虚拟机，并且配置好了虚拟机与宿主机50001~50004的端口转发（SDK与控制脚本通讯的默认端口），那么此时就需要特别指定`local_ip`为你的宿主机IP。

`config.headless_mode = true;`设置启用了无界面模式，此模式下会发送控制脚本给机器人的主端口（30001），机器人便会运行这个脚本。
> note：在`EliteDriver`的构造函数中会自动发送一次，如果此时机器人还未释放抱闸，则需要在释放抱闸后，调用`sendExternalControlScript()`方法再次发送控制脚本。

`config.script_file_path`指定了外部控制脚本的路径。

`config.servoj_time`设置了servoj的周期为0.004秒。


#### 2.3 加载控制脚本

```cpp
if (!s_driver->isRobotConnected()) {
    s_driver->sendExternalControlScript();
}

while (!s_driver->isRobotConnected()) {
    std::this_thread::sleep_for(10ms);
}

```

在构造`EliteDriver`时机器人可能还未释放抱闸，因此调用 `EliteDriver::isRobotConnected()` 判断SDK是否连接机器人，如果没有的话，就发送一次控制脚本。

发送完成后继续使用`EliteDriver::isRobotConnected()`等待机器人连接。

---

#### 2.3 控制逻辑：关节 6 正反转运动

```cpp
bool positive_rotation = false;
bool negative_rotation = false;
vector6d_t actual_joint;
vector6d_t target_joint;
double increment = 0;
bool first_point = true;
auto next = steady_clock::now();

while (!(positive_rotation && negative_rotation)) {
    actual_joint = s_rtsi_client->getActualJointPositions();

    if (first_point) {
        target_joint = actual_joint; // 初始化目标位置
        first_point = false;
    }

    // 正向旋转到关节 6 >= 3 rad
    if (!positive_rotation) {
        increment = 0.0005;
        if (actual_joint[5] >= 3) {
            positive_rotation = true;
        }
    }
    // 反向旋转到关节 6 <= -3 rad
    else if (!negative_rotation) {
        increment = -0.0005;
        if (actual_joint[5] <= -3) {
            negative_rotation = true;
        }
    }

    target_joint[5] += increment;

    // 发送 servoj 控制命令
    if (!s_driver->writeServoj(target_joint, 100)) {
        ELITE_LOG_FATAL("Send servoj command to robot fail");
        return 1;
    }

    next += 4ms;
    std::this_thread::sleep_until(next);
}
```

此循环：
* 从RTSI实时读取当前关节角度
* 每次增量 `0.0005 rad`（≈0.028°），让第六轴先正向旋转到 `+3 rad`（≈172°）再反向旋转到 `-3 rad`（≈-172°）
* 控制周期是 4ms（`servoj_time`），通过 `std::this_thread::sleep_until` 精确保持节拍

---

#### 2.4 停止控制**

```cpp
s_driver->stopControl();
```

释放控制权，程序退出。

---

> tips: 如果发送的目标点位是笛卡尔空间的坐标，那么`writeServoj()`提供了“cartesian”参数，此参数为true时，发送的点位将在控制脚本中做一次逆运动学变换的处理。

[>>>下一章：自定义日志](./Log.cn.md)
