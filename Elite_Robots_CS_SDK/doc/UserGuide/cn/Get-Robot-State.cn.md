[Home](./UserGuide.cn.md)

# 获取机器人状态

## 目标
调用SDK的相关接口，获取机器人的关节角度，并设置数字输出为高电平。

## 背景说明
RTSI 是 Elite CS 系列机器人的一种通讯接口，最高可以250HZ的频率读取、写入机器人IO。SDK中提供了两种接口，第一种接口是RTSI的基础接口（RtsiClientInterface），可以操控RTSI通讯的每一个步骤。第二种接口将机器人的各种数据（RtsiIOInterface），例如关节角等，封装成了函数，可以直接调用。  

RTSI 协议的具体说明可到官网下载RTSI说明文档，下图简单展示了RTSI协议的流程。

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

## 任务

### 1. 写一个简单的RTSI客户端程序

创建一个代码文件：
```bash
touch rtsi_client_example.cpp
```

复制下面代码，使用你习惯的文本编辑器，粘贴到`rtsi_client_example.cpp`中：

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
    vector6d_t actula_joints;
    int count = 250;
    while(count--) {
        if (!rtsi->receiveData(out_recipe)) {
            std::cout << "Receive recipe fail" << std::endl;
            return 1;
        }
        out_recipe->getValue("timestamp", timestamp);
        out_recipe->getValue("actual_joint_positions", actula_joints);

        std::cout << "timestamp: " << timestamp << std::endl;
        std::cout << "actual_joint_positions: ";
        for(auto i : actula_joints) {
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


### 2. 代码解析

这段代码是一个基于 **Elite 机器人 SDK** 中 `RTSI（Real-Time Synchronization Interface）` 实现的示例程序，展示了如何与机器人进行实时数据交互。

---

#### 2.1 程序功能概述

程序完成以下功能：

1. 与机器人建立实时通信连接（RTSI）
2. 协商协议版本
3. 设置输出/输入数据配方（recipe）
4. 启动数据同步
5. 连续接收关节数据并打印
6. 设置数字输出信号
7. 断开连接

---

#### 2.2 引入头文件

```cpp
#include <iostream>                         // 标准输入输出
#include <memory>                           // 使用智能指针 std::unique_ptr
#include <string>                           // 使用 std::string 类型
#include <Elite/RtsiClientInterface.hpp>    // RTSI 客户端接口
#include <Elite/RtsiRecipe.hpp>             // RTSI recipe（配方）相关接口
```

这些是 Elite SDK 中用于实时通信的主要组件。

---

#### 2.3 使用命名空间

```cpp
using namespace ELITE;
```

`RtsiClientInterface` 和 `RtsiRecipe` 是 `ELITE` 命名空间下的类，使用 `using` 简化调用。

---

#### 2.4 主函数入口和参数检查

```cpp
if (argc < 2) {
    std::cout << "Must provide robot IP..." << std::endl;
    return 1;
}
```

要求用户从命令行输入机器人 IP，如：

```bash
./rtsi_client_example 192.168.0.100
```

---

#### 2.5 创建并连接 RTSI 客户端

```cpp
std::unique_ptr<RtsiClientInterface> rtsi = std::make_unique<RtsiClientInterface>();
rtsi->connect(robot_ip);
```

`RtsiClientInterface` 是 RTSI 通信的核心类。
通过 `connect()` 与机器人建立 TCP 连接。

---

#### 2.6 协商 RTSI 协议版本

```cpp
if(rtsi->negotiateProtocolVersion()) {
    std::cout << "Negotiate protocol version success" << std::endl;
}
```

机器人与客户端使用 RTSI 进行通信前，需要协商协议版本，确保兼容性。调用`negotiateProtocolVersion()`来完成此步骤。

---

#### 2.7 输出控制器版本

```cpp
std::cout << "Controller version: " << rtsi->getControllerVersion().toString() << std::endl;
```

调用`getControllerVersion()`方法，可以获取机器人控制器的软件版本信息。
> tips: 此接口会发生一次RTSI的通讯，因此建议在如果要频繁使用控制器版本，在建立通讯后获取一次后记录下来。

---

#### 2.8 设置输出配方（接收数据）

```cpp
auto out_recipe = rtsi->setupOutputRecipe({"timestamp", "actual_joint_positions"}, 250);
```

调用`setupOutputRecipe()`接口，告诉机器人我需要接收哪些字段：
- `timestamp`: 时间戳
- `actual_joint_positions`: 实际关节角度（vector6d\_t 类型）
- 250：RTSI 的同步频率

---

#### 2.9 设置输入配方（发送数据）

```cpp
auto in_recipe = rtsi->setupInputRecipe({"standard_digital_output_mask", "standard_digital_output"});
```

调用`setupInputRecipe()`接口，定义要写入机器人的输入字段：“standard_digital_output_mask” 和 “standard_digital_output”：  
- `standard_digital_output_mask` 的功能是：使能RTSI标准数字IO输出设置。只有当该值的某个bit设置为1时，才能通过 standard_digital_output 设置对应的IO。
- `standard_digital_output`的功能是：设置标准数字IO输出，每一个 bit 代表一个 IO 。需要设置 standard_digital_output_mask 对应的 bit。
这些字段可以用来控制机器人的数字输出端口。

---

#### 2.10 启动同步通信

```cpp
if(rtsi->start()) {
    std::cout << "RTSI sync start successful" << std::endl;
}
```

调用`start()`方法，正式启动 RTSI 数据同步周期。调用此方法后，CS控制器将按设定的频率（如前文的250HZ）给客户端发送设定的数据。并且CS控制器将可以接收输入配方的数据，并设置机器人。

---

#### 2.11 循环接收并打印数据（共 250 次）

```cpp
int count = 250;
while(count--) {
    if (!rtsi->receiveData(out_recipe)) {
        std::cout << "Receive recipe fail" << std::endl;
        return 1;
    }

    out_recipe->getValue("timestamp", timestamp);
    out_recipe->getValue("actual_joint_positions", actula_joints);
```

每次循环中：

* `receiveData()` 接收一帧数据
* 使用 `getValue()` 获取数据字段的值

打印关节角度：

```cpp
for(auto i : actula_joints) {
    std::cout << i << " ";
}
```

---

#### 2.12 设置数字输出（DIO）

```cpp
in_recipe->setValue("standard_digital_output_mask", 1);
in_recipe->setValue("standard_digital_output", 1);
rtsi->send(in_recipe);
```

这段代码的作用是：

* 调用`RtsiRecipe::setValue()`设置 `mask` 指定修改哪个位（1 表示修改第 0 位）
* 调用`RtsiRecipe::setValue()`将第 0 号数字输出端口设为 1（高电平）
* 调用`RtsiClientInterface::send()`发送设置给RTSI。

---

#### 2.13 断开连接

```cpp
rtsi->disconnect();
```

关闭与机器人的 RTSI 通信连接。

---

### 3. 编译与运行
使用gcc编译器，编译`rtsi_client_example.cpp`：
```bash
g++ rtsi_client_example.cpp -o rtsi_client_example -lelite-cs-series-sdk
```

运行：
```bash
./rtsi_client_example <your robot ip>
```
运行后会接收并打印250次机器人的时间戳以及关节角度，并且设置数字输出 0 为高电平。

---

### 4. 使用封装好的接口来获取机器人状态

使用`RtsiClientInterface`接口可以控制RTSI通讯中的每个步骤，而`RtsiIOInterface`接口则对`RtsiClientInterface`进行了进一步的封装。接下来将使用`RtsiIOInterface`中的接口来实现：获取机器人关节角度并设置数字输出 0 为高电平的功能。

#### 4.1 准备配方文件

创建两个文本文件：
```bash
touch joint_recipe.txt digital_recipe.txt
```

复制下面文本，使用你习惯的文本编辑器，粘贴到`joint_recipe.txt`中：
```text
timestamp
actual_joint_positions
```

复制下面文本，使用你习惯的文本编辑器，粘贴到`digital_recipe.txt`中：
```text
standard_digital_output_mask
standard_digital_output
```

> 注意：每一行后面不要有任何的空白字符

#### 4.2 写一个简单的RTSI程序

创建代码文件：
```bash
touch rtsi_client_io_example.cpp
```

复制下面代码，使用你习惯的文本编辑器，粘贴到`rtsi_client_io_example.cpp`中：
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
        auto actula_joints = io_interface->getActualJointPositions();
        auto timestamp = io_interface->getTimestamp();

        std::cout << "timestamp: " << timestamp << std::endl;
        std::cout << "actual_joint_positions: ";
        for(auto i : actula_joints) {
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

### 5. 代码解析

这段程序是基于 **Elite 机器人 SDK** 的 RTSI（Real-Time Synchronization Interface）接口中，`RtsiIOInterface` 类的使用示例。该示例展示了如何使用该接口实现：

* 连接机器人
* 实时获取关节角度和时间戳
* 控制数字输出（DIO）

---

#### 5.1 功能概览

程序主要流程：

1. 创建 `RtsiIOInterface` 实例（使用 recipe 文件）
2. 连接机器人 RTSI 服务
3. 打印机器人版本信息
4. 连续读取 250 次实际关节角度数据和时间戳（周期 4ms）
5. 设置一个标准数字输出
6. 断开连接

---

#### 5.2 引用头文件

```cpp
#include <iostream>                     // 标准输入输出
#include <memory>                       // 使用智能指针 std::unique_ptr
#include <chrono>                       // 时间控制（周期、sleep 等）
#include <Elite/RtsiIOInterface.hpp>    //  Elite SDK 的 RTSI I/O 接口类
```

---

#### 5.3 使用命名空间

```cpp
using namespace ELITE;
using namespace std::chrono;
```

* 简化类调用，如 `RtsiIOInterface`, `VersionInfo`, `4ms`, `steady_clock`

---

#### 5.4 解析命令行参数

```cpp
if (argc < 2) {
    std::cout << "Must provide robot IP..." << std::endl;
    return 1;
}
std::string robot_ip = std::string(argv[1]);
```

* 要求通过命令行输入机器人 IP 地址，示例用法：
```bash
./rtsi_client_io_example 192.168.0.100
```

---

#### 5.5 创建 RTSI IO 接口

```cpp
std::unique_ptr<RtsiIOInterface> io_interface = 
    std::make_unique<RtsiIOInterface>("joint_recipe.txt", "digital_recipe.txt", 250);
```

* 构造函数参数含义：

  * `"joint_recipe.txt"`：输出 recipe 文件（定义机器人发送哪些数据）
  * `"digital_recipe.txt"`：输入 recipe 文件（定义可以控制哪些数据）
  * `250`：RTSI 的同步频率

> tips：关于配方文件：配方文件每一行是一个订阅项，订阅项后面不要有空白字符

---

#### 5.6 连接机器人

```cpp
if (!io_interface->connect(robot_ip)) {
    std::cout << "Couldn't connect RTSI server" << std::endl;
    return 1;
}
```

调用 `connect()` 与机器人建立 RTSI 通信连接，并校验协议版本，设置输入、输出配方，启动后台同步线程等。

---

#### 5.7 获取控制器版本

```cpp
VersionInfo version = io_interface->getControllerVersion();
std::cout << "Controller is: " << version.toString() << std::endl;
```

获取机器人的控制器版本，并打印版本信息。

---

#### 5.8 实时获取数据（循环 250 次）

```cpp
int count = 250;
auto next = steady_clock::now();
while(count--) {
    auto actula_joints = io_interface->getActualJointPositions();
    auto timestamp = io_interface->getTimestamp();
```

* 调用 `getActualJointPositions()` 获取六个关节当前角度（返回 `vector6d_t`）
* 调用 `getTimestamp()` 获取机器人控制器当前的时间戳（`double`）

输出：

```cpp
    std::cout << "timestamp: " << timestamp << std::endl;
    std::cout << "actual_joint_positions: ";
    for(auto i : actula_joints) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
```

控制周期为 4ms：

```cpp
    next += 4ms;
    std::this_thread::sleep_until(next);
```

---

#### 5.9 设置标准数字输出

```cpp
io_interface->setStandardDigital(0, 1);
```

设置第 0 路数字输出为 1（高电平）

暂停 100ms 以观察效果：

```cpp
std::this_thread::sleep_for(100ms);
```

---

#### 5.10 断开连接

```cpp
io_interface->disconnect();
```

断开 RTSI 实时通信连接。

---

### 3. 编译与运行
使用gcc编译器，编译`rtsi_client_io_example.cpp`：
```bash
g++ rtsi_client_io_example.cpp -o rtsi_client_io_example -lelite-cs-series-sdk
```

运行：
```bash
./rtsi_client_io_example <your robot ip>
```
运行后会接收并打印250次机器人的时间戳以及关节角度，并且设置数字输出 0 为高电平。

---

[>>>下一章： 与机器人的主端口通讯](./Primary-Port-Message.cn.md)