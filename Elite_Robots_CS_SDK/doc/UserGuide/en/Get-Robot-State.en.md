[Home](./UserGuide.en.md)

# Obtain Robot Status

## Objective
Call the relevant interfaces of the SDK to obtain the robot's joint angles and set the digital output to high level.

## Background
RTSI is a communication interface for Elite CS series robots, which can read and write robot I/O at a maximum frequency of 250Hz. The SDK provides two types of interfaces. The first is the basic RTSI interface (RtsiClientInterface), which can control every step of RTSI communication. The second interface encapsulates various robot data (RtsiIOInterface), such as joint angles, into functions that can be called directly.  

The specific description of the RTSI protocol can be found in the RTSI documentation downloaded from the official website. The following diagram briefly illustrates the workflow of the RTSI protocol.

```
               +------+
               | 连接 |
               +------+
                  |
                  |
                  v
            +-------------+
            | 验证协议版本 |
            +-------------+
                   |
         +---------+------------+
         |                      |
         |                      |
+-------------------+       +------------------+       
| 配置输入、输出订阅 |       | 获取控制器版本信息|
+-------------------+       +------------------+
        |
        |
+-------------+ 
| 发送启动信号 |
+-------------+ 
       |
       |
+-------------+ 
| 开始数据同步 |
+-------------+ 
```

## Tasks

### 1. Write a Simple RTSI Client Program

Create a code file:
```bash
touch rtsi_client_example.cpp
```

Copy the following code and paste it into `rtsi_client_example.cpp` using your preferred text editor:

```cpp
#include <iostream>
#include <memory>
#include <string>
#include <Elite/RtsiClientInterface.hpp>
#include <Elite/RtsiRecipe.hpp>

using namespace ELITE;


int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Must provide robot IP. Example: ./rtsi_client_example aaa.bbb.ccc.ddd" << std::endl;
        return 1;
    }
    std::string robot_ip = std::string(argv[1]);

    std::unique_ptr<RtsiClientInterface> rtsi = std::make_unique<RtsiClientInterface>();

    rtsi->connect(robot_ip);

    if(rtsi->negotiateProtocolVersion()) {
        std::cout << "Negotiate protocol version success" << std::endl;
    } else {
        std::cout << "Negotiate protocol version fail" << std::endl;
        return 1;
    }
    
    std::cout << "Controller version: " << rtsi->getControllerVersion().toString() << std::endl;

    auto out_recipe = rtsi->setupOutputRecipe({"timestamp", "actual_joint_positions"}, 250);

    auto in_recipe = rtsi->setupInputRecipe({"standard_digital_output_mask", "standard_digital_output"});

    if(rtsi->start()) {
        std::cout << "RTSI sync start successful" << std::endl;
    } else {
        std::cout << "RTSI sync start fail" << std::endl;
        return 1;
    }
    
    double timestamp;
    vector6d_t actual_joints;
    int count = 250;
    while(count--) {
        if (!rtsi->receiveData(out_recipe)) {
            std::cout << "Receive recipe fail" << std::endl;
            return 1;
        }
        out_recipe->getValue("timestamp", timestamp);
        out_recipe->getValue("actual_joint_positions", actual_joints);

        std::cout << "timestamp: " << timestamp << std::endl;
        std::cout << "actual_joint_positions: ";
        for(auto i : actual_joints) {
            std::cout << i << " ";
        }
        std::cout << std::endl;
    }

    in_recipe->setValue("standard_digital_output_mask", 1);
    in_recipe->setValue("standard_digital_output", 1);
    rtsi->send(in_recipe);

    rtsi->disconnect();

    return 0;
}

```


### 2. Code Analysis

This code is an example program based on the `RTSI (Real-Time Synchronization Interface)` in the **Elite Robot SDK**, demonstrating how to perform real-time data interaction with the robot.

---

#### 2.1 Program Function Overview

The program completes the following functions:

1. Establish a real-time communication connection (RTSI) with the robot
2. Negotiate the protocol version
3. Set up output/input data recipes
4. Start data synchronization
5. Continuously receive joint data and print it
6. Set digital output signals
7. Disconnect

---

#### 2.2 Include Header Files

```cpp
#include <iostream>                         // Standard input and output
#include <memory>                           // Use smart pointer std::unique_ptr
#include <string>                           // Use std::string type
#include <Elite/RtsiClientInterface.hpp>    // RTSI client interface
#include <Elite/RtsiRecipe.hpp>             // RTSI recipe-related interfaces
```

These are the main components for real-time communication in the Elite SDK.

---

#### 2.3 Use Namespace

```cpp
using namespace ELITE;
```

`RtsiClientInterface` and `RtsiRecipe` are classes under the `ELITE` namespace, and `using` is used to simplify calls.

---

#### 2.4 Main Function Entry and Parameter Check

```cpp
if (argc < 2) {
    std::cout << "Must provide robot IP..." << std::endl;
    return 1;
}
```

Requires the user to input the robot IP from the command line, such as:

```bash
./rtsi_client_example 192.168.0.100
```

---

#### 2.5 Create and Connect RTSI Client

```cpp
std::unique_ptr<RtsiClientInterface> rtsi = std::make_unique<RtsiClientInterface>();
rtsi->connect(robot_ip);
```

`RtsiClientInterface` is the core class for RTSI communication.
Establish a TCP connection with the robot through `connect()`.

---

#### 2.6 Negotiate RTSI Protocol Version

```cpp
if(rtsi->negotiateProtocolVersion()) {
    std::cout << "Negotiate protocol version success" << std::endl;
}
```

Before the robot and client communicate using RTSI, they need to negotiate the protocol version to ensure compatibility. Call `negotiateProtocolVersion()` to complete this step.

---

#### 2.7 Output Controller Version

```cpp
std::cout << "Controller version: " << rtsi->getControllerVersion().toString() << std::endl;
```

Call the `getControllerVersion()` method to obtain the software version information of the robot controller.
> tips: This interface will perform an RTSI communication. Therefore, if you need to use the controller version frequently, it is recommended to obtain it once after establishing the communication and record it.

---

#### 2.8 Set Up Output Recipe (Receive Data)

```cpp
auto out_recipe = rtsi->setupOutputRecipe({"timestamp", "actual_joint_positions"}, 250);
```

Call the `setupOutputRecipe()` interface to tell the robot which fields I need to receive:
- `timestamp`: Timestamp
- `actual_joint_positions`: Actual joint angles (vector6d_t type)
- 250: RTSI synchronization frequency

---

#### 2.9 Set Up Input Recipe (Send Data)

```cpp
auto in_recipe = rtsi->setupInputRecipe({"standard_digital_output_mask", "standard_digital_output"});
```

Call the `setupInputRecipe()` interface to define the input fields to be written to the robot: "standard_digital_output_mask" and "standard_digital_output":  
- The function of `standard_digital_output_mask` is to enable the RTSI standard digital IO output setting. Only when a certain bit of this value is set to 1, the corresponding IO can be set through `standard_digital_output`.
- The function of `standard_digital_output` is to set the standard digital IO output, where each bit represents an IO. The corresponding bit of `standard_digital_output_mask` needs to be set.
These fields can be used to control the robot's digital output ports.

---

#### 2.10 Start Synchronous Communication

```cpp
if(rtsi->start()) {
    std::cout << "RTSI sync start successful" << std::endl;
}
```

Call the `start()` method to formally start the RTSI data synchronization cycle. After calling this method, the CS controller will send the set data to the client at the set frequency (such as 250Hz mentioned earlier). And the CS controller will be able to receive data from the input recipe and set the robot.

---

#### 2.11 Cyclically Receive and Print Data (250 times in total)

```cpp
int count = 250;
while(count--) {
    if (!rtsi->receiveData(out_recipe)) {
        std::cout << "Receive recipe fail" << std::endl;
        return 1;
    }

    out_recipe->getValue("timestamp", timestamp);
    out_recipe->getValue("actual_joint_positions", actual_joints);
```

In each cycle:

* `receiveData()` receives a frame of data
* Use `getValue()` to obtain the value of the data field

Print joint angles:

```cpp
for(auto i : actual_joints) {
    std::cout << i << " ";
}
```

---

#### 2.12 Set Digital Output (DIO)

```cpp
in_recipe->setValue("standard_digital_output_mask", 1);
in_recipe->setValue("standard_digital_output", 1);
rtsi->send(in_recipe);
```

The function of this code is:

* Call `RtsiRecipe::setValue()` to set `mask` to specify which bit to modify (1 means modify bit 0)
* Call `RtsiRecipe::setValue()` to set digital output port 0 to 1 (high level)
* Call `RtsiClientInterface::send()` to send the settings to RTSI.

---

#### 2.13 Disconnect

```cpp
rtsi->disconnect();
```

Close the RTSI communication connection with the robot.

---

### 3. Compilation and Running
Use the gcc compiler to compile `rtsi_client_example.cpp`:
```bash
g++ rtsi_client_example.cpp -o rtsi_client_example -lelite-cs-series-sdk
```

Run:
```bash
./rtsi_client_example <your robot ip>
```
After running, it will receive and print the robot's timestamp and joint angles 250 times, and set digital output 0 to high level.

---

### 4. Use Encapsulated Interfaces to Obtain Robot Status

The `RtsiClientInterface` interface can control each step in RTSI communication, while the `RtsiIOInterface` interface further encapsulates `RtsiClientInterface`. Next, we will use the interface in `RtsiIOInterface` to realize the function of obtaining the robot's joint angles and setting digital output 0 to high level.

#### 4.1 Prepare Recipe Files

Create two text files:
```bash
touch joint_recipe.txt digital_recipe.txt
```

Copy the following text and paste it into `joint_recipe.txt` using your preferred text editor:
```text
timestamp
actual_joint_positions
```

Copy the following text and paste it into `digital_recipe.txt` using your preferred text editor:
```text
standard_digital_output_mask
standard_digital_output
```

> Note: There should be no any whitespace characters after each line

#### 4.2 Write a Simple RTSI Program

Create a code file:
```bash
touch rtsi_client_io_example.cpp
```

Copy the following code and paste it into `rtsi_client_io_example.cpp` using your preferred text editor:
```cpp
#include <iostream>
#include <memory>
#include <chrono>

#include <Elite/RtsiIOInterface.hpp>

using namespace ELITE;
using namespace std::chrono;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Must provide robot IP. Example: ./rtsi_client_io_example aaa.bbb.ccc.ddd" << std::endl;
        return 1;
    }
    std::string robot_ip = std::string(argv[1]);

    std::unique_ptr<RtsiIOInterface> io_interface = std::make_unique<RtsiIOInterface>("joint_recipe.txt", "digital_recipe.txt", 250);

    if (!io_interface->connect(robot_ip)) {
        std::cout << "Couldn't connect RTSI server" << std::endl;
        return 1;
    }

    VersionInfo version = io_interface->getControllerVersion();
    std::cout << "Controller is: " << version.toString() << std::endl;

    int count = 250;
    auto next = steady_clock::now();
    while(count--) {
        auto actual_joints = io_interface->getActualJointPositions();
        auto timestamp = io_interface->getTimestamp();

        std::cout << "timestamp: " << timestamp << std::endl;
        std::cout << "actual_joint_positions: ";
        for(auto i : actual_joints) {
            std::cout << i << " ";
        }
        std::cout << std::endl;

        next += 4ms;
        std::this_thread::sleep_until(next);
    }

    io_interface->setStandardDigital(0, 1);
    std::this_thread::sleep_for(100ms);

    io_interface->disconnect();

    return 0;
}
```

### 5. Code Analysis

This program is an example of using the `RtsiIOInterface` class in the RTSI (Real-Time Synchronization Interface) of the **Elite Robot SDK**. The example demonstrates how to use this interface to:

* Connect to the robot
* Real-time obtain joint angles and timestamps
* Control digital output (DIO)

---

#### 5.1 Function Overview

The main process of the program:

1. Create an `RtsiIOInterface` instance (using recipe files)
2. Connect to the robot's RTSI service
3. Print robot version information
4. Continuously read actual joint angle data and timestamps 250 times (with a period of 4ms)
5. Set a standard digital output
6. Disconnect

---

#### 5.2 Include Header Files

```cpp
#include <iostream>                     // Standard input and output
#include <memory>                       // Use smart pointer std::unique_ptr
#include <chrono>                       // Time control (period, sleep, etc.)
#include <Elite/RtsiIOInterface.hpp>    // Elite SDK's RTSI I/O interface class
```

---

#### 5.3 Use Namespaces

```cpp
using namespace ELITE;
using namespace std::chrono;
```

* Simplify class calls, such as `RtsiIOInterface`, `VersionInfo`, `4ms`, `steady_clock`

---

#### 5.4 Parse Command-Line Parameters

```cpp
if (argc < 2) {
    std::cout << "Must provide robot IP..." << std::endl;
    return 1;
}
std::string robot_ip = std::string(argv[1]);
```

* Requires inputting the robot IP address through the command line, example usage:
```bash
./rtsi_client_io_example 192.168.0.100
```

---

#### 5.5 Create RTSI IO Interface

```cpp
std::unique_ptr<RtsiIOInterface> io_interface = 
    std::make_unique<RtsiIOInterface>("joint_recipe.txt", "digital_recipe.txt", 250);
```

* Meanings of constructor parameters:

  * `"joint_recipe.txt"`: Output recipe file (defines which data the robot sends)
  * `"digital_recipe.txt`: Input recipe file (defines which data can be controlled)
  * `250`: RTSI synchronization frequency

> tips: About recipe files: Each line in the recipe file is a subscription item, and there should be no whitespace characters after the subscription item.

---

#### 5.6 Connect to the Robot

```cpp
if (!io_interface->connect(robot_ip)) {
    std::cout << "Couldn't connect RTSI server" << std::endl;
    return 1;
}
```

Call `connect()` to establish an RTSI communication connection with the robot, verify the protocol version, set input and output recipes, start the background synchronization thread, etc.

---

#### 5.7 Obtain Controller Version

```cpp
VersionInfo version = io_interface->getControllerVersion();
std::cout << "Controller is: " << version.toString() << std::endl;
```

Obtain the controller version of the robot and print the version information.

---

#### 5.8 Real-Time Obtain Data (Loop 250 times)

```cpp
int count = 250;
auto next = steady_clock::now();
while(count--) {
    auto actual_joints = io_interface->getActualJointPositions();
    auto timestamp = io_interface->getTimestamp();
```

* Call `getActualJointPositions()` to obtain the current angles of the six joints (returns `vector6d_t`)
* Call `getTimestamp()` to obtain the current timestamp of the robot controller (`double`)

Output:

```cpp
    std::cout << "timestamp: " << timestamp << std::endl;
    std::cout << "actual_joint_positions: ";
    for(auto i : actual_joints) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
```

Control period is 4ms:

```cpp
    next += 4ms;
    std::this_thread::sleep_until(next);
```

---

#### 5.9 Set Standard Digital Output

```cpp
io_interface->setStandardDigital(0, 1);
```

Set digital output 0 to 1 (high level)

Pause for 100ms to observe the effect:

```cpp
std::this_thread::sleep_for(100ms);
```

---

#### 5.10 Disconnect

```cpp
io_interface->disconnect();
```

Disconnect the RTSI real-time communication connection.

---

### 3. Compilation and Running
Use the gcc compiler to compile `rtsi_client_io_example.cpp`:
```bash
g++ rtsi_client_io_example.cpp -o rtsi_client_io_example -lelite-cs-series-sdk
```

Run:
```bash
./rtsi_client_io_example <your robot ip>
```
After running, it will receive and print the robot's timestamp and joint angles 250 times, and set digital output 0 to high level.

---

[>>>Next Chapter: Communicate with the Robot's Primary Port](./Primary-Port-Message.en.md)