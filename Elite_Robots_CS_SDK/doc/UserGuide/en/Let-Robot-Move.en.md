[Home](./UserGuide.en.md)

# Get the Robot Moving

## Objective
Call the relevant interfaces of the SDK to make the robot's end effector run a triangular trajectory starting from its current pose.

## Background
The SDK controls the robot by sending a control script to the robot via a plug-in or port 30001. This control script connects to the SDK through the TCP/IP protocol and receives control commands.

Before proceeding with this chapter, ensure that you have understood the usage of the [`DashboardClient`](./Power-on-Robot.en.md) and [`RtsiIOInterface`](./Get-Robot-State.en.md) classes.

## Tasks

### 1. Encapsulate the Trajectory Motion Interface

Create a code file:
```bash
touch trajectory_example.cpp
```

Copy the following code and paste it into `trajectory_example.cpp` using your preferred text editor:

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

### 2. Code Analysis
`TrajectoryControl` is a further encapsulation of `EliteDriver` and `DashboardClient`, organizing the SDK's APIs into easy-to-use processes.

The `EliteDriver` class is mainly responsible for communicating with the control script and integrating some commonly used interfaces.

This chapter mainly uses three interfaces from `EliteDriver`: `setTrajectoryResultCallback()`, `writeTrajectoryPoint()`, and `writeTrajectoryControlAction()` to make the robot perform "movej" and "movel" movements. The calling process is as follows:
- Call `setTrajectoryResultCallback()` to register a callback function for motion results. After the robot finishes moving along this trajectory, this callback will be invoked to inform the motion result.
- Call `writeTrajectoryControlAction()` to write the start command.
- Call `writeTrajectoryPoint()` to send the points and parameters, and at the same time call `writeTrajectoryControlAction()` to send a noop command to prevent timeout.
- Continuously call `writeTrajectoryControlAction()` to send noop commands and wait for the motion result.
- If you want to cancel the motion during the process, you can call `writeTrajectoryControlAction()` to send a cancel command.

---

#### 2.1 Member Variables & Constructor / Destructor

##### ***2.1.1 Member Variables***

```cpp
std::unique_ptr<EliteDriver> driver_;
std::unique_ptr<DashboardClient> dashboard_;
EliteDriverConfig config_;
```

* **`driver_`**
  Used to send trajectory control commands, set callbacks, manage external control scripts, etc.
* **`dashboard_`**
  Used for "panel" functions such as powering on the robot, releasing the brake, starting programs, etc.
* **`config_`**
  Stores configurations (robot IP, mode, port, etc.)

---

##### ***2.1.2 Constructor***

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

* Saves `config`
* Creates `driver_` (constructed using config)
* Creates `dashboard_` (default construction)
* Immediately connects to the Dashboard; if the connection fails, it directly throws an exception to interrupt the program.

---

##### ***2.1.3 Destructor***

```cpp
~TrajectoryControl() {
    if (dashboard_) {
        dashboard_->disconnect();
    }
    driver_->stopControl();
}
```

* If the Dashboard is connected → disconnect
* Stops trajectory control (ensures the robot exits the control mode)

---

#### 2.2 Logic of Core Methods

##### 2.2.1 ***`startControl()` — Start the Robot Control Process***

```cpp
bool startControl() {
    // 1. Power on
    if (!dashboard_->powerOn()) return false;

    // 2. Release brake
    if (!dashboard_->brakeRelease()) return false;

    // 3. Start external control according to mode
    if (config_.headless_mode) {
        if (!driver_->isRobotConnected()) {
            if (!driver_->sendExternalControlScript()) return false;
        }
    } else {
        if (!dashboard_->playProgram()) return false;
    }

    // 4. Wait for the robot to enter external control mode
    while (!driver_->isRobotConnected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
}
```

Process:

1. **Power on**
2. **Release the brake**
3. **Start external control according to mode**
   * headless → directly send the external control script
   * non-headless → start the loaded control program through the Dashboard
4. Wait for the robot to connect to the external control interface (polling detection)

---

##### 2.2.3 ***`moveTrajectory()` — Execute Multi-point Trajectory Motion***

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

        return result == TrajectoryMotionResult::SUCCESS;
    }
```

The function of this function is to send multiple trajectory points, and the robot moves in sequence.

Execution logic:

1. Call `setTrajectoryResultCallback()` to register a callback function for motion results. In the callback function, use `std::promise` / `std::future` to wait for the asynchronous callback of the motion completion signal.
   
   ```cpp
    std::promise<TrajectoryMotionResult> move_done_promise;
    driver_->setTrajectoryResultCallback([&](TrajectoryMotionResult result) { move_done_promise.set_value(result); });
   ```

2. Send the trajectory start command, as well as the number of trajectory points and the command timeout time (200ms).

    ```cpp
    driver_->writeTrajectoryControlAction(START, target_points.size(), 200);
    ```
> Command timeout time: If no next command is sent within the timeout time, a timeout will be triggered. For example, 200ms here. If the `writeTrajectoryControlAction()` is not called again to send a command within 200ms, a timeout will be triggered.

3. Send trajectory points (code for log printing is abbreviated)

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

   * Call `writeTrajectoryPoint()` to send joint angles / Cartesian poses
   * Immediately send a `NOOP` after sending each point (to prevent communication timeout)

4. Wait for trajectory completion
    ```cpp
    std::future<TrajectoryMotionResult> move_done_future = move_done_promise.get_future();
    while (move_done_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
        if(!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200)) {
            return false;
        }
    }
    ```

   * Cyclically check if the `future` has been assigned a value
   * Continue to send `NOOP` during each waiting period to prevent timeout

5. After the motion ends, make the robot return to idle

    ```cpp
    std::promise<TrajectoryMotionResult> cancel_done_promise;
    driver_->setTrajectoryResultCallback([&](TrajectoryMotionResult result) { cancel_done_promise.set_value(result); });
    if(!driver_->writeIdle(0)) {
        ELITE_LOG_ERROR("Failed to write idle command");
        return false;
    }
    ```
    If the `writeIdle()` interface is called directly, it will trigger another callback (exiting the trajectory motion mode will trigger a CANCEL signal for trajectory motion). Since `move_done_promise` has already been assigned a value after the previous trajectory motion reaches the target, another callback needs to be registered here to receive the CANCEL signal.
    Use `writeIdle()` to make the robot enter an idle state (i.e., not execute any control commands). The timeout time here is set to 0, which means infinite time.
    Finally, wait for 10 seconds for the `future` to be assigned a value.

6. **Return whether the motion was successful**

   * Determine if the result is `TrajectoryMotionResult::SUCCESS`

---

##### 2.2.4 ***`moveTo()` — Single-point Motion***

```cpp
bool moveTo(const vector6d_t& point, float time, bool is_cartesian) {
    return moveTrajectory({point}, time, 0, is_cartesian);
}
```

It is a single-point version of `moveTrajectory` with `blend_radius=0` and only one point in the trajectory.

---

### 3. Using TrajectoryControl
Add the header file `#include <Elite/RtsiIOInterface.hpp>` to the header declaration section of `trajectory_example.cpp`.

Add the following content to the end of the `trajectory_example.cpp` file:
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

Create RTSI recipe files:
```bash
touch output_recipe.txt input_recipe.txt
```

Copy the following text content and paste it into `output_recipe.txt` using your preferred text editor:
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


Copy the following text content and paste it into `input_recipe.txt` using your preferred text editor:
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

Copy the `external_control.script` control script file to the current directory (note to replace /your/path/):
```bash
cp /your/path/external_control.script ./
```

> tips: Location of `external_control.script`:
> - If installed by compilation, it can be found in the `Elite_Robots_CS_SDK/source/resources/` path of the source code project, or, if no installation path is set, it is usually in the `/usr/local/share/Elite/` path.
> - If installed using apt, it is usually in the `/usr/share/Elite/` path.


### 4. Analysis

#### 4.1 New Code

The function of the new code is to use `RtsiIOInterface` to obtain the current joint angles and pose of the robot, and perform movements starting from the current pose.

1. Obtain the robot IP and local IP from the command line (if needed).
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

As mentioned in the [“Background Description”](#background-description), the control script will establish a TCP connection with the SDK. Therefore, essentially, the SDK needs the local IP address (to replace the IP address in `socket_open()` in the control script). However, `EliteDriver` will try to obtain the local IP during construction, so there is no need to manually set the `local_ip` of `config`. However, in some cases, you may need to set the local IP separately. For example, if you use a virtual machine with NAT configuration and have configured port forwarding for ports 50001~50004 between the virtual machine and the host (the default ports for communication between the SDK and the control script), then you need to specifically specify `local_ip` as your host IP.

`config.headless_mode = true;` enables headless mode, in which the control script will be sent to the robot's main port (30001), and the robot will run this script.
> note: It will be sent automatically in the constructor of `EliteDriver`. If the robot has not released the brake at this time, you need to call the `sendExternalControlScript()` method to send the control script again after releasing the brake.

`config.script_file_path` specifies the path of the external control script.


2. Obtain the current joint angles and rotate the fourth joint by 90 degrees.
    ```cpp
    vector6d_t actual_joints = rtsi_client->getActualJointPositions();
    actual_joints[3] = -1.57;

    if(!trajectory_control->moveTo(actual_joints, 3, false)) {
        return 1;
    }
    ```

3. Obtain the current TCP pose, plan a triangular trajectory, and run it
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

#### 4.2 RTSI Recipe Files and Control Script

As known from the chapter [Obtaining Robot State](./Get-Robot-State.en.md), `RtsiIOInterface` requires recipe files to determine which states or data of the robot to read and write.

As known from [“Background Description”](#background-description), the control logic of the SDK is to connect with the SDK through the control script using the TCP/IP protocol and send control commands.

### 5. Compilation and Running
Use the gcc compiler to compile `trajectory_example.cpp`:
```bash
g++ trajectory_example.cpp -o trajectory_example -lelite-cs-series-sdk
```

Run:
```bash
./trajectory_example <your robot ip> <your local ip(option)>
```
After running, the robot should power on, release the brake, rotate the fourth joint by 90 degrees, and then run a triangular trajectory (you can pay attention to the teach pendant interface, as a singular pose may occur).

---

[>>>Next Chapter: Transparent Transmission: servoj](./Servoj-Move.en.md)