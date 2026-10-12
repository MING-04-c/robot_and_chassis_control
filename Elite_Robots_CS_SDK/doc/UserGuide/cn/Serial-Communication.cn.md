[Home](./UserGuide.cn.md)

# RS485 串口通讯

## 目标

1. 调用SDK的接口读写串口。

2. （Linux）将串口映射到本地。

## 背景说明

Elite CS 系列机器人的控制柜、末端有RS485的通讯接口（依据具体型号为准，部分型号没有此接口），SDK 提供了一些列操作这些接口的方法。

## 任务

### 1. 写一个简单的读写程序
创建一个代码文件：
```bash
touch serial_example.cpp
```

复制下面代码，使用你习惯的文本编辑器，粘贴到`serial_example.cpp`中：

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

### 2. 代码解析

这段代码展示了如何使用**Elite 机器人 SDK**的工具RS485通讯功能。

代码最开始声明了使用到的C++头文件，在C++标准库之前的`Elite/EliteDriver.hpp`头文件主要包含了控制机器人的方法。`Elite/Log.hpp`头文件是Elite CS SDK的日志功能的头文件。
```cpp
#include <Elite/EliteDriver.hpp>
#include <Elite/Log.hpp>
#include <chrono>
#include <iostream>
#include <thread>

using namespace ELITE;
using namespace std::chrono;
```

接下来的几行代码功能是，解析命令行输入的参数，并赋值给`config`中的“机器人IP”和“本地IP”，`config`变量是`EliteDriverConfig`类型，此类型是`EliteDriver`类构造函数的配置，包括了通讯端口的配置、运动参数的配置等等。
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

启用了`headless_mode`，此模式下`EliteDriver`可无需示教器直接控制机器人。设置了控制脚本的路径，即程序运行目录下的`external_control.script`文件。接着构造了`EliteDriver`的实例，这个类里包含了控制机器人的方法，以及部分通讯接口。
```cpp
    config.headless_mode = true;
    config.script_file_path = "external_control.script";
    auto driver = std::make_unique<EliteDriver>(config);
```

使用`EliteDriver::isRobotConnected()`循环判断，等待机器人和SDK连接。
```cpp
    ELITE_LOG_INFO("Wait external control script run...");
    while (!driver->isRobotConnected()) {
        std::this_thread::sleep_for(10ms);
    }
    ELITE_LOG_INFO("External control script is running");
```

配置RS485波特率为115200，无奇偶校验，停止位为一，并调用`EliteDriver::startToolRs485()`启用工具RS485通讯，启用成功后，获取到类型为`SerialCommunicationSharedPtr`的实例指针，此实例可用于连接、读写串口等操作。
```cpp
    SerialConfig serial_config;
    serial_config.baud_rate = SerialConfig::BaudRate::BR_115200;
    serial_config.parity = SerialConfig::Parity::NONE;
    serial_config.stop_bits = SerialConfig::StopBits::ONE;
    auto serial = driver->startToolRs485(serial_config);
```

调用`SerialCommunication::connect()`连接到转发服务端口，此操作在后文进行说明。
```cpp
    ELITE_LOG_INFO("Connecting to serial socat server...");
    if(!serial->connect(1000)) {
        ELITE_LOG_FATAL("Can't connect socat server");
        return 1;
    }
    ELITE_LOG_INFO("Connected to serial socat server.");
```

调用`SerialCommunication::write()`让工具RS485发送`hello wrold`字符串。
```cpp
    ELITE_LOG_INFO("Send data to serial...");
    std::string hello_str = "hello world";
    if (serial->write((const uint8_t*)hello_str.c_str(), hello_str.size()) <= 0) {
        ELITE_LOG_FATAL("Send data to serial fail.");
        return 1;
    }
    ELITE_LOG_INFO("Data sent.");
```

调用`SerialCommunication::read()`接收与发送字符串长度相同的字符串，并打印。
```cpp
    ELITE_LOG_INFO("Read data from serial...");
    std::string receive_str(hello_str);
    if(serial->read((uint8_t*)receive_str.data(), receive_str.size(), 5000) <= 0) {
        ELITE_LOG_FATAL("Read data to serial fail.");
        return 1;
    }
    ELITE_LOG_INFO("Receive:%s", receive_str.c_str());
```

停止工具RS485通讯，并停止控制机器人，正常退出程序。
```cpp
    ELITE_LOG_INFO("Ending serial communication...");
    driver->endToolRs485(serial);
    driver->stopControl();
    ELITE_LOG_INFO("Serial communication ended.");

    return 0;
}
```

### 3. SDK RS485通讯原理

SDK 向机器人发送启动串口通讯的指令后，机器人使用 socat 将RS485串口（工具或控制柜）转发到指定的 TCP/IP 端口，例如，在上面的代码中，调用了`driver->startToolRs485(serial_config)`之后，就会把工具RS485串口的数据，转发到TCP/IP端口54321，所以只需要去连接机器人IP的54321端口，就能读写工具RS485串口的数据了。因此在上面的代码中`driver->startToolRs485(serial_config)`返回的`SerialCommunicationSharedPtr`类型，其本质上是一个TCP/IP的客户端，所以才需要再调用`SerialCommunication::connect()`连接到机器人串口转发的服务端。

当然，如果用到其他的一些协议，例如MODBUS-RTU，需要用其他库来打开串口、并解析数据，这种情况下，可以再用一些工具将机器人的TCP/IP端口映射为操作系统的串口。例如，在Linux中，同样可以用socat，参考下面指令：
```bash
# 在这里设置你的机器人IP
export ROBOT_IP=192.168.56.101
# 在这里设置机器人的转发端口，对应 EliteDriver::startToolRs485() 的 port 参数
export TCI_PORT=54321
# 请设置要创建的设备的名称，并确保对该位置具有写入权限。
export LOCAL_DEVICE_NAME=/tmp/ttyElite
socat pty,link=${LOCAL_DEVICE_NAME},raw,ignoreeof,waitslave tcp:${ROBOT_IP}:${TCI_PORT}
```
这之后就能像使用其他串口设备一样去使用`${LOCAL_DEVICE_NAME}`了（在这里的例子中就是`/tmp/ttyElite`）。

### 4. 编译与运行

使用gcc编译器，编译`serial_example.cpp`：
```bash
g++ serial_example.cpp -o serial_example -lelite-cs-series-sdk
```

开始运行前，需要连接机器人末端RS485接口到你的电脑上，然后在你的电脑上打开一个串口上位机软件，并打开连接的设备，配置为：波特率 115200，无奇偶校验，停止位设为一，数据位数为8。

因为上面的代码中没有机械臂上电等操作，因此这里需要使用示教器手动上电并释放抱闸。

运行：
```bash
./serial_example <your robot ip>
```

运行后，你电脑的串口上位机软件中应当会接收到“hello world”的字符串，5秒内你可以同样发送“hello world”，那么程序会接收到这个字符串并打印。