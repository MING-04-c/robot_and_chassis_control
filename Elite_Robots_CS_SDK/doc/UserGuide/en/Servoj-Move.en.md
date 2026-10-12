[Home](./UserGuide.en.md)  

# Passthrough: servoj  

## Objective  
Call the relevant SDK interfaces to make the robot's end-effector oscillate (forward and reverse rotation) starting from its current pose.  

## Background  
The SDK controls the robot by sending a control script to the robot via a plugin or port 30001. This control script establishes a TCP/IP connection with the SDK and receives control commands.  

Elite Robots' `servoj` command is used for real-time joint position control. It processes received joint angles within a lookahead time window, applies moving average filtering, and then performs spline fitting to generate the final joint positions for real-time control. The SDK provides the `writeServoj()` interface for this command.  

Before proceeding, ensure you are familiar with the usage of the [`DashboardClient`](./Power-on-Robot.en.md) and [`RtsiIOInterface`](./Get-Robot-State.en.md) classes.  

## Task  

### 1. Write a Program to Oscillate the Robot's End-Effector  

Create a code file:  
```bash  
touch servoj_example.cpp  
```  

Copy the following code into `servoj_example.cpp` using your preferred text editor:  
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

### 2. Code Explanation  
This code demonstrates how to connect to the robot, start an external control script, and drive joint motion in real-time via the RTSI interface until specific rotation conditions are met.  

#### 2.1 Main Function Flow  

```cpp  
mlockall(MCL_CURRENT | MCL_FUTURE);  
pthread_t handle = pthread_self();  
RT_UTILS::setThreadFiFoScheduling(handle, RT_UTILS::getThreadFiFoMaxPriority());  
RT_UTILS::bindThreadToCpus(handle, 2);  
```  

- `mlockall(MCL_CURRENT | MCL_FUTURE);` locks memory to prevent real-time tasks from being swapped out.  
- The SDK provides real-time thread utilities. Here, `RT_UTILS::setThreadFiFoScheduling()` sets the thread priority (using `RT_UTILS::getThreadFiFoMaxPriority()` for the highest priority).  
- `RT_UTILS::bindThreadToCpus()` binds the thread to specific CPU cores.  
- These settings ensure smoother robot control.  

---  

#### 2.2 Parse Command-Line Arguments  

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

- As mentioned in the [Background](#background), the control script establishes a TCP connection, so the SDK needs the local IP (to replace the `socket_open()` IP in the script).  
- `EliteDriver` automatically retrieves the local IP, but manual configuration may be needed (e.g., in NAT-configured VMs).  
- `headless_mode = true` enables script execution on port 30001.  
- `script_file_path` specifies the external control script.  
- `servoj_time` sets the servoj cycle to 0.004 seconds.  

---  

#### 2.3 Load Control Script  

```cpp  
if (!s_driver->isRobotConnected()) {  
    s_driver->sendExternalControlScript();  
}  

while (!s_driver->isRobotConnected()) {  
    std::this_thread::sleep_for(10ms);  
}  
```  

- If the robot is not connected, the script is resent.  
- The loop waits until the robot is connected.  

---  

#### 2.4 Control Logic: Joint 6 Oscillation  

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
        target_joint = actual_joint; // Initialize target position  
        first_point = false;  
    }  

    // Rotate forward to Joint 6 ≥ 3 rad  
    if (!positive_rotation) {  
        increment = 0.0005;  
        if (actual_joint[5] >= 3) {  
            positive_rotation = true;  
        }  
    }  
    // Rotate backward to Joint 6 ≤ -3 rad  
    else if (!negative_rotation) {  
        increment = -0.0005;  
        if (actual_joint[5] <= -3) {  
            negative_rotation = true;  
        }  
    }  

    target_joint[5] += increment;  

    // Send servoj command  
    if (!s_driver->writeServoj(target_joint, 100)) {  
        ELITE_LOG_FATAL("Send servoj command to robot fail");  
        return 1;  
    }  

    next += 4ms;  
    std::this_thread::sleep_until(next);  
}  
```  

- Reads current joint angles via RTSI.  
- Increments Joint 6 by `0.0005 rad` (~0.028°) until reaching `+3 rad` (~172°), then reverses to `-3 rad` (~-172°).  
- Maintains a 4ms control cycle (`servoj_time`) using `std::this_thread::sleep_until`.  

---  

#### 2.5 Stop Control  

```cpp  
s_driver->stopControl();  
```  

Releases control before program exit.  

---  

> **Tip**: If sending Cartesian coordinates, set `writeServoj()`'s `cartesian` parameter to `true` for inverse kinematics processing.  

[>>> Next: Custom Logging](./Log.en.md)  