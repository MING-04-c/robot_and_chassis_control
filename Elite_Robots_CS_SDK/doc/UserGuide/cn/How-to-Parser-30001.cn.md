[Home](./UserGuide.cn.md)

# 如何解析主端口报文

## 目标
编写一个读取机器人TCP坐标的程序。

## 背景说明
Primary port报文是由多个子报文组成，但是SDK并没有提供所有的子报文解析，因此需要用户自行解析需要的数据。

## 任务

### 1. 写一个解析TCP坐标的类

创建一个头文件：
```bash
touch RobotTCPPackage.hpp
```

复制下面代码，使用你习惯的文本编辑器，粘贴到`RobotTCPPackage.hpp`中：

```cpp
#pragma once

#include <Elite/PrimaryPackage.hpp>
#include <algorithm>
#include <type_traits>
#include <cstring>
#include <iostream>

class RobotTCPPackage : public ELITE::PrimaryPackage {
private:
    constexpr static int ROBOT_TCP_PKG_TYPE = 4;
public:
    RobotTCPPackage() : PrimaryPackage(ROBOT_TCP_PKG_TYPE) { }

    ~RobotTCPPackage() = default;

    uint32_t unpackUInt32(const std::vector<uint8_t>::const_iterator& iter) {
        uint8_t bytes[sizeof(uint32_t)];
        std::copy(iter, iter + sizeof(uint32_t), bytes);

        std::reverse(std::begin(bytes), std::end(bytes));
        
        uint32_t result;
        std::memcpy(&result, bytes, sizeof(uint32_t));
        return result;
    }

    double unpackDouble(const std::vector<uint8_t>::const_iterator& iter) {
        uint8_t bytes[sizeof(double)];
        std::copy(iter, iter + sizeof(double), bytes);

        std::reverse(std::begin(bytes), std::end(bytes));
        
        double result;
        std::memcpy(&result, bytes, sizeof(double));
        return result;
    }

    virtual void parser(int len, const std::vector<uint8_t>::const_iterator& iter) override {
        int offset = 0;
        std::cout << "Package len: " << unpackUInt32(iter + offset) << std::endl;
        offset += sizeof(uint32_t);

        std::cout << "Package type: " << (int)*(iter + offset) << std::endl;
        offset += sizeof(uint8_t);

        std::cout << "TCP Position X:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Position Y:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Position Z:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Rotation X:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Rotation Y:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Rotation Z:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Offset Position X:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Offset Position Y:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Offset Position Z:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Offset Rotation X:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Offset Rotation Y:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Offset Rotation Z:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

    }
};
```

### 2. 代码解析

RobotTCPPackage 继承自 `ELITE::PrimaryPackage`，专门用于解析机器人 TCP（Tool Center Point）数据包。
TCP 数据包含机械臂末端在空间中的 位置（X、Y、Z）和 姿态（旋转角度），以及 位置/姿态偏移量。
该类实现了自定义的二进制解析逻辑，把接收到的字节流解码成实际的数值并输出。

正式开始解析代码之前，需要打开 “CS_用户手册_机器人状态报文.xlsx” 表格，并找到对应你机器人版本的 sheet ，找到“笛卡尔数据子报文”。

---

#### 2.1 头文件
```cpp
#include <Elite/PrimaryPackage.hpp>
#include <algorithm>
#include <type_traits>
#include <cstring>
#include <iostream>
```

`Elite/PrimaryPackage.hpp`：由于要继承`ELITE::PrimaryPackage`，因此必须包含此头文件。

---

#### 2.2 构造与析构

```cpp
RobotTCPPackage() : PrimaryPackage(ROBOT_TCP_PKG_TYPE) { }
~RobotTCPPackage() = default;
```
* 构造时调用父类构造函数，绑定包类型。通过前文提到的“CS_用户手册_机器人状态报文.xlsx” 表格中的“笛卡尔数据子报文”部分得知，这封子报文的类型值是 `4` ，因此代码的前文`ROBOT_TCP_PKG_TYPE` 就是一个值为4的常量。
* 析构函数使用 default，无特殊清理逻辑。

---

#### 2.3 数据解包工具函数
* 解包 32 位无符号整数
    ```cpp
    uint32_t unpackUInt32(const std::vector<uint8_t>::const_iterator& iter)
    ```
    1. 从 `iter` 位置复制 4 字节到本地数组 `bytes`。
    2. 反转字节顺序（因为网络字节序是大端）。
    3. 用 `memcpy` 把字节拷贝到 `uint32_t` 变量中并返回。

* 解包 double 浮点数
    ```cpp
    double unpackDouble(const std::vector<uint8_t>::const_iterator& iter)
    ```
    * 逻辑与 `unpackUInt32` 类似，只是处理的字节数变为 8 字节，并转为 `double` 类型。

---

#### 2.4 解析函数（核心）

```cpp
virtual void parser(int len, const std::vector<uint8_t>::const_iterator& iter) override
```

* 该函数重写了父类的 `parser()` 方法，用于按协议格式解析接收到的字节流。
* 查看“CS_用户手册_机器人状态报文.xlsx” 表格中的“笛卡尔数据子报文”部分，依据这部分的数据格式来进行解析。
* 参数`iter`是子报文第一个字节的迭代器。
* 解析顺序严格按照数据包定义的字段顺序来读取。


解析时维护一个 `offset` 表示当前解析位置，每解析一个字段，就把 `offset` 向前推进该字段的字节数。

1. **包长度**

   ```cpp
   unpackUInt32(iter + offset);
   offset += sizeof(uint32_t);
   ```

2. **包类型**（单字节）

   ```cpp
   (int)*(iter + offset);
   offset += sizeof(uint8_t);
   ```

3. **TCP 末端位置**（X、Y、Z，各 8 字节 double）

   * TCP Position X
   * TCP Position Y
   * TCP Position Z

4. **TCP 姿态**（旋转 X、Y、Z，各 8 字节 double）

   * TCP Rotation X
   * TCP Rotation Y
   * TCP Rotation Z

5. **TCP 位置偏移**（X、Y、Z，各 8 字节 double）

   * TCP Offset Position X
   * TCP Offset Position Y
   * TCP Offset Position Z

6. **TCP 姿态偏移**（旋转 X、Y、Z，各 8 字节 double）

   * TCP Offset Rotation X
   * TCP Offset Rotation Y
   * TCP Offset Rotation Z


### 3. 使用***RobotTCPPackage***

在同一目录下，创建一个代码文件：
```bash
touch primary_pkg_example.cpp
```

复制下面代码，使用你习惯的文本编辑器，粘贴到`primary_pkg_example.cpp`中：
```cpp
#include <Elite/PrimaryPortInterface.hpp>
#include "RobotTCPPackage.hpp"
#include <memory>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono;

int main(int argc, const char** argv) {
    if (argc < 2) {
        std::cout << "Must provide robot IP. Example: ./primary_pkg_example aaa.bbb.ccc.ddd" << std::endl;
        return 1;
    }
    std::string robot_ip = argv[1];

    auto primary = std::make_unique<ELITE::PrimaryPortInterface>();

    auto robotPackage = std::make_shared<RobotTCPPackage>();

    primary->connect(robot_ip, 30001);

    primary->getPackage(robotPackage, 200);

    primary->disconnect();

    return 0;
}
```

### 4. 代码解析
这段代码演示了如何通过 **ELITE SDK** 的 `PrimaryPortInterface` 从机器人主端口（Primary Interface）获取 **TCP（Tool Center Point）数据包**，并使用自定义的 `RobotTCPPackage` 类解析数据。

运行流程非常简洁：

1. 从命令行读取机器人 IP 地址。
2. 创建主通信接口对象和 TCP 数据包解析对象。
3. 连接机器人主端口（30001）。
4. 获取一帧 TCP 数据并解析。
5. 断开连接并退出。

---

#### 4.1 头文件

```cpp
#include <Elite/PrimaryPortInterface.hpp>
#include "RobotTCPPackage.hpp"
#include <memory>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
```
* `<Elite/PrimaryPortInterface.hpp>`：主端口通信接口类，用于连接、收发数据包。
* `"RobotTCPPackage.hpp"`：自定义的 TCP 数据包解析类，这是包含刚才`RobotTCPPackage`类定义的头文件。

---

#### 4.2 检查命令行参数

```cpp
if (argc < 2) {
    std::cout << "Must provide robot IP. Example: ./primary_pkg_example aaa.bbb.ccc.ddd" << std::endl;
    return 1;
}
std::string robot_ip = argv[1];
```

* 程序必须通过参数传入机器人 IP。
* 示例运行方式：

  ```bash
  ./primary_pkg_example 192.168.1.10
  ```

---

#### 4.3 创建通信对象与数据包解析对象

```cpp
auto primary = std::make_unique<ELITE::PrimaryPortInterface>();
auto robotPackage = std::make_shared<RobotTCPPackage>();
```

* `PrimaryPortInterface`：与机器人主端口通信的核心接口。
* `RobotTCPPackage`：自定义继承自 `PrimaryPackage`，负责解析 TCP 位姿数据（X、Y、Z 位置 + 姿态 + 偏移量）。

---

#### 4.4 连接机器人主端口

```cpp
primary->connect(robot_ip, 30001);
```

* 连接到指定 IP 的 **Primary Interface**，端口号 **30001**。
* 该端口是 Elite 机器人 SDK 用于传输实时控制和状态数据的主要通道。

---

#### 4.5 获取并解析数据包

```cpp
primary->getPackage(robotPackage, 200);
```

* 从机器人读取一个 **PrimaryPackage** 数据帧（这里传入的是 `RobotTCPPackage` 对象）。
* **200** 表示超时时间（毫秒），超过则返回失败。
* `PrimaryPortInterface` 会调用 `RobotTCPPackage::parser()` 解析字节流，并在其中输出 TCP 位姿信息。

---

#### 4.6 断开连接

```cpp
primary->disconnect();
```

* 释放网络连接，清理资源。

### 5. 编译与运行
使用gcc编译器，编译`primary_pkg_example.cpp`：
```bash
g++ primary_pkg_example.cpp -o primary_pkg_example -lelite-cs-series-sdk
```

```bash
./primary_pkg_example <your robot ip>
```

运行之后，应当打印包的类型、长度、以及TCP的位姿与偏移量。

---

[>>>下一章：让机器人动起来](./Let-Robot-Move.cn.md)