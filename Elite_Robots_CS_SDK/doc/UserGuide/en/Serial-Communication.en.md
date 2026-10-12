[Home](./UserGuide.en.md)

# RS485 Serial Communication

## Objectives

1. Read from and write to serial ports using the SDK's interfaces.
2. (Linux) Map the serial port to the local machine.

## Background

Elite CS series robot control cabinets and end effectors are equipped with RS485 communication interfaces (availability depends on the specific model). The SDK provides a series of methods to operate these interfaces.

## Tasks

### 1. Write a Simple Read/Write Program
Create a code file:
```bash
touch serial_example.cpp
```

Copy the following code and paste it into `serial_example.cpp` using your preferred text editor:

```cpp
#include <Elite/EliteDriver.hpp>
#include <Elite/Log.hpp>
#include <chrono>
#include <iostream>
#include <thread>

using namespace ELITE;
using namespace std::chrono;

int main(int argc, char** argv) {
    EliteDriverConfig config;
    if (argc == 2) {
        config.robot_ip = argv[1];
    } else if (argc == 3) {
        config.robot_ip = argv[1];
        config.local_ip = argv[2];
    } else {
        std::cout << "Must provide robot IP. Example: ./serial_example aaa.bbb.ccc.ddd <eee.fff.ggg.hhh>" << std::endl;
        return 1;
    }
    config.headless_mode = true;
    config.script_file_path = "external_control.script";
    auto driver = std::make_unique<EliteDriver>(config);
    
    ELITE_LOG_INFO("Wait external control script run...");
    while (!driver->isRobotConnected()) {
        std::this_thread::sleep_for(10ms);
    }
    ELITE_LOG_INFO("External control script is running");

    SerialConfig serial_config;
    serial_config.baud_rate = SerialConfig::BaudRate::BR_115200;
    serial_config.parity = SerialConfig::Parity::NONE;
    serial_config.stop_bits = SerialConfig::StopBits::ONE;
    auto serial = driver->startToolRs485(serial_config);
    
    ELITE_LOG_INFO("Connecting to serial socat server...");
    if(!serial->connect(1000)) {
        ELITE_LOG_FATAL("Can't connect socat server");
        return 1;
    }
    ELITE_LOG_INFO("Connected to serial socat server.");

    ELITE_LOG_INFO("Send data to serial...");
    std::string hello_str = "hello world";
    if (serial->write((const uint8_t*)hello_str.c_str(), hello_str.size()) <= 0) {
        ELITE_LOG_FATAL("Send data to serial fail.");
        return 1;
    }
    ELITE_LOG_INFO("Data sent.");

    ELITE_LOG_INFO("Read data from serial...");
    std::string receive_str(hello_str);
    if(serial->read((uint8_t*)receive_str.data(), receive_str.size(), 5000) <= 0) {
        ELITE_LOG_FATAL("Read data to serial fail.");
        return 1;
    }
    ELITE_LOG_INFO("Receive:%s", receive_str.c_str());

    ELITE_LOG_INFO("Ending serial communication...");
    driver->endToolRs485(serial);
    driver->stopControl();
    ELITE_LOG_INFO("Serial communication ended.");

    return 0;
}
```

### 2. Code Analysis

This code demonstrates how to use the **Elite Robot SDK's** tool RS485 communication functionality.

The code begins by declaring the required C++ header files. The `Elite/EliteDriver.hpp` header, included before the C++ standard library headers, primarily contains methods for controlling the robot. The `Elite/Log.hpp` header is for the Elite CS SDK's logging functionality.
```cpp
#include <Elite/EliteDriver.hpp>
#include <Elite/Log.hpp>
#include <chrono>
#include <iostream>
#include <thread>

using namespace ELITE;
using namespace std::chrono;
```

The next few lines of code parse the command-line arguments and assign the "robot IP" and "local IP" to the `config` variable. The `config` variable is of type `EliteDriverConfig`, which is the configuration for the `EliteDriver` class constructor, including communication port settings, motion parameters, etc.
```cpp
int main(int argc, char** argv) {
    EliteDriverConfig config;
    if (argc == 2) {
        config.robot_ip = argv[1];
    } else if (argc == 3) {
        config.robot_ip = argv[1];
        config.local_ip = argv[2];
    } else {
        std::cout << "Must provide robot IP. Example: ./serial_example aaa.bbb.ccc.ddd <eee.fff.ggg.hhh>" << std::endl;
        return 1;
    }
```

`headless_mode` is enabled, allowing `EliteDriver` to control the robot directly without a teach pendant. The control script path is set to the `external_control.script` file in the program's running directory. Then, an instance of `EliteDriver` is constructed. This class contains methods for controlling the robot and some communication interfaces.
```cpp
    config.headless_mode = true;
    config.script_file_path = "external_control.script";
    auto driver = std::make_unique<EliteDriver>(config);
```

Use `EliteDriver::isRobotConnected()` in a loop to wait for the robot to connect to the SDK.
```cpp
    ELITE_LOG_INFO("Wait external control script run...");
    while (!driver->isRobotConnected()) {
        std::this_thread::sleep_for(10ms);
    }
    ELITE_LOG_INFO("External control script is running");
```

Configure RS485 with a baud rate of 115200, no parity, and one stop bit. Call `EliteDriver::startToolRs485()` to enable tool RS485 communication. Upon successful enabling, obtain an instance pointer of type `SerialCommunicationSharedPtr`. This instance can be used for operations like connecting, reading, and writing to the serial port.
```cpp
    SerialConfig serial_config;
    serial_config.baud_rate = SerialConfig::BaudRate::BR_115200;
    serial_config.parity = SerialConfig::Parity::NONE;
    serial_config.stop_bits = SerialConfig::StopBits::ONE;
    auto serial = driver->startToolRs485(serial_config);
```

Call `SerialCommunication::connect()` to connect to the forwarding service port (explained later).
```cpp
    ELITE_LOG_INFO("Connecting to serial socat server...");
    if(!serial->connect(1000)) {
        ELITE_LOG_FATAL("Can't connect socat server");
        return 1;
    }
    ELITE_LOG_INFO("Connected to serial socat server.");
```

Call `SerialCommunication::write()` to send the "hello world" string via the tool RS485.
```cpp
    ELITE_LOG_INFO("Send data to serial...");
    std::string hello_str = "hello world";
    if (serial->write((const uint8_t*)hello_str.c_str(), hello_str.size()) <= 0) {
        ELITE_LOG_FATAL("Send data to serial fail.");
        return 1;
    }
    ELITE_LOG_INFO("Data sent.");
```

Call `SerialCommunication::read()` to receive a string of the same length as the sent string and print it.
```cpp
    ELITE_LOG_INFO("Read data from serial...");
    std::string receive_str(hello_str);
    if(serial->read((uint8_t*)receive_str.data(), receive_str.size(), 5000) <= 0) {
        ELITE_LOG_FATAL("Read data to serial fail.");
        return 1;
    }
    ELITE_LOG_INFO("Receive:%s", receive_str.c_str());
```

Stop the tool RS485 communication, stop controlling the robot, and exit the program normally.
```cpp
    ELITE_LOG_INFO("Ending serial communication...");
    driver->endToolRs485(serial);
    driver->stopControl();
    ELITE_LOG_INFO("Serial communication ended.");

    return 0;
}
```

### 3. SDK RS485 Communication Principle

After the SDK sends a command to the robot to start serial communication, the robot uses socat to forward the RS485 serial port (tool or control cabinet) to a specified TCP/IP port. For example, in the code above, after calling `driver->startToolRs485(serial_config)`, data from the tool RS485 serial port is forwarded to TCP/IP port 54321. Therefore, simply connecting to port 54321 on the robot's IP allows reading and writing of tool RS485 serial port data. This is why the `SerialCommunicationSharedPtr` type returned by `driver->startToolRs485(serial_config)` in the code above is essentially a TCP/IP client, necessitating the subsequent call to `SerialCommunication::connect()` to connect to the robot's serial port forwarding server.

Of course, if other protocols are used, such as MODBUS-RTU, which require other libraries to open the serial port and parse data, tools can be used to map the robot's TCP/IP port to an operating system serial port. For example, in Linux, socat can also be used, as shown in the following command:
```bash
# Set your robot IP here
export ROBOT_IP=192.168.56.101
# Set the robot's forwarding port here, corresponding to the port parameter of EliteDriver::startToolRs485()
export TCI_PORT=54321
# Set the name for the device you wish to create. Make sure that your user can write to that location.
export LOCAL_DEVICE_NAME=/tmp/ttyElite
socat pty,link=${LOCAL_DEVICE_NAME},raw,ignoreeof,waitslave tcp:${ROBOT_IP}:${TCI_PORT}
```
After this, you can use `${LOCAL_DEVICE_NAME}` (in this example, `/tmp/ttyElite`) just like any other serial port device.

### 4. Compilation and Execution

Use the gcc compiler to compile `serial_example.cpp`:
```bash
g++ serial_example.cpp -o serial_example -lelite-cs-series-sdk
```

Before running, connect the robot's end effector RS485 interface to your computer. Then, open a serial port host computer software on your computer, open the connected device, and configure it with: baud rate 115200, no parity, one stop bit, and 8 data bits.

Since the code above does not include operations like powering on the robot arm, you need to manually power on and release the brake using the teach pendant.

Run:
```bash
./serial_example <your robot ip>
```

After running, your computer's serial port host software should receive the "hello world" string. Within 5 seconds, you can also send "hello world", and the program will receive this string and print it.