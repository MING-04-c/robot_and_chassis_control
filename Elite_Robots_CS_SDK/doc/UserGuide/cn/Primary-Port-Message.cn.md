[Home](./UserGuide.cn.md)

# 与机器人的主端口通讯

## 目标
调用SDK的接口，与机器人的主端口（30001）通讯。实现解析报文，获取异常，发送脚本功能。

## 背景说明
Primary port是 Elite CS 系列机器人的一种通讯接口，以10HZ的频率向外发送机器人的状态数据，同时此接口能接收脚本并运行。协议的具体说明可到官网下载说明文档。


## 任务

### 1. 写一个简单的客户端程序

创建一个代码文件：
```bash
touch primary_example.cpp
```

复制下面代码，使用你习惯的文本编辑器，粘贴到`primary_example.cpp`中：

```cpp
#include <Elite/Log.hpp>
#include <Elite/PrimaryPortInterface.hpp>
#include <Elite/RobotConfPackage.hpp>
#include <Elite/RobotException.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono;

// When the robot encounters an exception, this callback will be called
void robotExceptionCb(ELITE::RobotExceptionSharedPtr ex) {
    if (ex->getType() == ELITE::RobotException::Type::RUNTIME) {
        auto r_ex = std::static_pointer_cast<ELITE::RobotRuntimeException>(ex);
        ELITE_LOG_INFO("Robot throw exception: %s", r_ex->getMessage().c_str());
    }
}

int main(int argc, const char** argv) {
    if (argc < 2) {
        std::cout << "Must provide robot IP. Example: ./primary_example aaa.bbb.ccc.ddd" << std::endl;
        return 1;
    }
    std::string robot_ip = argv[1];

    auto primary = std::make_unique<ELITE::PrimaryPortInterface>();

    auto kin = std::make_shared<ELITE::KinematicsInfo>();

    primary->connect(robot_ip, 30001);

    primary->registerRobotExceptionCallback(robotExceptionCb);

    primary->getPackage(kin, 200);

    std::string dh_param = "\n\tDH parameter a: ";
    for (auto i : kin->dh_a_) {
        dh_param += std::to_string(i);
        dh_param += '\t';
    }
    dh_param += '\n';

    dh_param += "\n\tDH parameter d: ";
    for (auto i : kin->dh_d_) {
        dh_param += std::to_string(i);
        dh_param += '\t';
    }
    dh_param += '\n';

    dh_param += "\n\tDH parameter alpha: ";
    for (auto i : kin->dh_alpha_) {
        dh_param += std::to_string(i);
        dh_param += '\t';
    }
    dh_param += '\n';

    ELITE_LOG_INFO("%s", dh_param.c_str());

    std::string script = "def hello():\n\ttextmsg(\"hello world\")\nend\n";

    primary->sendScript(script);

    script = "def exFunc():\n\t1abcd\nend\n";
    primary->sendScript(script);

    // Wait robot exception
    std::this_thread::sleep_for(1s);

    primary->disconnect();

    return 0;
}
```

### 2. 代码解析
这段代码演示了如何使用 **Elite SDK** 通过 **PrimaryPortInterface** 与机器人建立主通信连接，获取机器人运动学参数（DH 参数），发送自定义脚本，并处理运行时异常。主要流程包括：

1. 连接机器人主端口（30001）。
2. 注册异常回调函数，处理机器人运行时异常。
3. 获取并打印机器人 DH 参数。
4. 发送正常脚本与错误脚本，触发异常回调。
5. 等待一段时间以接收异常信息，然后断开连接。

#### 2.1 头文件与命名空间
```cpp
#include <Elite/Log.hpp>
#include <Elite/PrimaryPortInterface.hpp>
#include <Elite/RobotConfPackage.hpp>
#include <Elite/RobotException.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
```

* **Elite SDK 相关**

  * `Elite/Log.hpp`：日志接口，用于打印信息。
  * `Elite/PrimaryPortInterface.hpp`：机器人主通信端口接口类，负责连接、收发数据。
  * `Elite/RobotConfPackage.hpp`：机器人配置信息结构体，包括 DH 参数等。
  * `Elite/RobotException.hpp`：机器人异常类定义，包含运行时异常类型。

* **标准库**

  * `<chrono>`、`<thread>`：用于延时等待。
  * `<memory>`：智能指针。
  * `<iostream>`、`<string>`：基本输入输出和字符串处理。

* **命名空间简化**

  ```cpp
  using namespace std::chrono;
  namespace po = boost::program_options;
  ```

---

#### 2.2 异常回调函数

```cpp
void robotExceptionCb(ELITE::RobotExceptionSharedPtr ex) {
    if (ex->getType() == ELITE::RobotException::Type::RUNTIME) {
        auto r_ex = std::static_pointer_cast<ELITE::RobotRuntimeException>(ex);
        ELITE_LOG_INFO("Robot throw exception: %s", r_ex->getMessage().c_str());
    }
}
```
这是一个异常回调函数，在后面的代码中会被注册到SDK中。当机器人出现异常时，SDK 会调用此回调函数。

这个函数判断异常类型是否为 **运行时异常**，即机器人运行脚本抛出的异常（还有一种异常是`ROBOT_ERROR` 这种异常是如急停、碰撞等机器人硬件上的错误）。

如果是运行时异常，使用C++标准库函数`std::static_pointer_cast`转换为 `RobotRuntimeException` 类型，因为`RobotException`是`RobotRuntimeException`的父类，而`RobotRuntimeException`对应着`ROBOT_RUNTIME`类型的异常（`ROBOT_ERROR`异常对应着`RobotError`类）。通过`ELITE_LOG_INFO`打印错误信息。

---

#### 3.3 主程序入口

```cpp
int main(int argc, const char** argv) {
    if (argc < 2) {
        std::cout << "Must provide robot IP. Example: ./primary_example aaa.bbb.ccc.ddd" << std::endl;
        return 1;
    }
    std::string robot_ip = argv[1];
```

这段代码表示必须传入机器人 IP 地址，否则退出。例如：
```bash
./primary_example 192.168.1.10
```

---

#### 3.4 创建通信接口与数据对象

```cpp
auto primary = std::make_unique<ELITE::PrimaryPortInterface>();
auto kin = std::make_shared<ELITE::KinematicsInfo>();
```

* `PrimaryPortInterface`：与机器人主端口（30001）建立通信的核心对象。
* `KinematicsInfo`：用于存储机器人运动学信息（DH 参数）。

---

#### 3.5 连接机器人与注册回调

```cpp
primary->connect(robot_ip, 30001);
primary->registerRobotExceptionCallback(robotExceptionCb);
```

* 调用 `connect()`，使用 IP + 端口（30001）连接主通信端口。
* 调用`registerRobotExceptionCallback()`将前文提到的 `robotExceptionCb` 注册到接口对象，用于接收异常通知。

---

#### 3.6 获取并打印 DH 参数

```cpp
primary->getPackage(kin, 200);
```

从机器人获取配置信息（此处是运动学参数），超时时间 200ms。

```cpp
std::string dh_param = "\n\tDH parameter a: ";
for (auto i : kin->dh_a_) { ... }
...
ELITE_LOG_INFO("%s", dh_param.c_str());
```

* 循环读取 **DH 参数 a、d、alpha** 并拼接成字符串。
* 最终通过 `ELITE_LOG_INFO` 打印到日志。

> DH 参数（Denavit–Hartenberg parameters）是描述机械臂关节与连杆空间关系的标准化参数。

---

#### 3.7 发送脚本

```cpp
std::string script = "def hello():\n\ttextmsg(\"hello world\")\nend\n";
primary->sendScript(script);
```

* **正常脚本**：定义 `hello()` 函数，在机器人端输出 `hello world`。

```cpp
script = "def exFunc():\n\t1abcd\nend\n";
primary->sendScript(script);
```

* **错误脚本**：语法错误（`1abcd` 非法），会触发运行时异常，通过之前注册的回调输出异常信息。

---

#### 3.8 等待异常回调并断开连接

```cpp
std::this_thread::sleep_for(1s);
primary->disconnect();
```

* 延时 1 秒等待异常消息到达。
* 断开机器人连接，释放资源。

### 3. 编译与运行
使用gcc编译器，编译`primary_example.cpp`：
```bash
g++ primary_example.cpp -o primary_example -lelite-cs-series-sdk
```

运行：
> 注意：在运行之前先将机器人上电并释放抱闸。
```bash
./primary_example <your robot ip>
```

运行之后，应当打印机器人的DH参数以及一个运行时异常。机器人示教器界面会出现一个异常提醒，示教器的“运行 -> 日志”输出框里应该有"hello world"的输出。

> tips：`PrimaryPortInterface` 的接口同时也集成到了 `EliteDriver` 类里面，关于`EliteDriver`类会在后文中出现。

---
[>>>下一章：如何解析主端口报文](./How-to-Parser-30001.cn.md)

