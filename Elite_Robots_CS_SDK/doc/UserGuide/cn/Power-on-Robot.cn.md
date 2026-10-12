[Home](./UserGuide.cn.md)
# Power On

## 目标
调用SDK的接口，打开机器人电源，并释放抱闸。

## 背景说明
Dashbaord Shell是与机器人交互的一种方式，通过TCP连接机器人的29999端口，可实现机器人上下电、释放抱闸、加载查询任务等操作。SDK中提供了`DashboardClient`类，里面包含了大部分的dashboard接口。
此章节中，将使用机器人的Dashbaord Shell功能来实现打开机器人电源与释放机器人抱闸。

## 任务

### 1. 写一个简单的dashboard客户端程序

创建一个代码文件：
```bash
touch dashboard_example.cpp
```

复制下面代码，使用你习惯的文本编辑器，粘贴到`dashboard_example.cpp`中：

```cpp
#include <iostream>
#include <memory>
#include <string>
#include <Elite/DashboardClient.hpp>

using namespace ELITE;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Must provide robot IP. Example: ./dashboard_example aaa.bbb.ccc.ddd" << std::endl;
        return 1;
    }
    std::string robot_ip = argv[1];

    std::unique_ptr<DashboardClient> my_dashboard;
    my_dashboard.reset(new DashboardClient());

    if (!my_dashboard->connect(robot_ip)) {
        std::cout << "Could not connect to robot" << std::endl;
        return 1;
    } else {
        std::cout << "Connect to robot" << std::endl;
    }

    // Power on
    if (!my_dashboard->powerOn()) {
        std::cout << "Could not send Power on command" << std::endl;
        return 1;
    } else {
        std::cout << "Power on" << std::endl;
    }

    // Brake release
    if (!my_dashboard->brakeRelease()) {
        std::cout << "Could not send BrakeRelease command" << std::endl;
        return 1;
    } else {
        std::cout << "Brake release" << std::endl;
    }
    
    my_dashboard->disconnect();

    return 0;
}
```

### 2. 代码解析

这段代码用于通过 Elite 机器人 SDK 中的 `DashboardClient` 控制接口，连接机器人并执行以下操作：

1. 连接机器人
2. 上电
3. 释放抱闸
4. 断开连接

下面是对每一部分的详细解释：

---

#### 2.1 引用的头文件

```cpp
#include <iostream>     // 标准输入输出
#include <memory>       // 使用智能指针 std::unique_ptr
#include <string>       // 使用 std::string 类型
#include <Elite/DashboardClient.hpp>  // Elite SDK 提供的 Dashboard 控制类
```

---

#### 2.2 命名空间

```cpp
using namespace ELITE;
```

`DashboardClient` 是 `ELITE` 命名空间下的类，使用 `using` 简化调用。

---

#### 2.3 主函数入口

```cpp
int main(int argc, char* argv[])
```

这是程序入口，支持通过命令行参数传入机器人 IP 地址。

---

#### 2.4 参数检查

```cpp
if (argc < 2) {
    std::cout << "Must provide robot IP. Example: ./dashboard_example aaa.bbb.ccc.ddd" << std::endl;
    return 1;
}
```

程序要求必须传入机器人 IP，否则会打印错误提示并退出。

---

#### 2.5 创建并连接 Dashboard 客户端

```cpp
std::string robot_ip = argv[1];

std::unique_ptr<DashboardClient> my_dashboard;
my_dashboard.reset(new DashboardClient());
```

使用 `std::unique_ptr` 管理 `DashboardClient` 实例，确保资源自动释放。

```cpp
if (!my_dashboard->connect(robot_ip)) {
    std::cout << "Could not connect to robot" << std::endl;
    return 1;
} else {
    std::cout << "Connect to robot" << std::endl;
}
```

调用 `connect()` 方法连接机器人，失败则退出。

---

#### 2.6 上电

```cpp
if (!my_dashboard->powerOn()) {
    std::cout << "Could not send Power on command" << std::endl;
    return 1;
} else {
    std::cout << "Power on" << std::endl;
}
```

发送上电指令，如果失败则退出。

---

#### 2.7 释放抱闸

```cpp
if (!my_dashboard->brakeRelease()) {
    std::cout << "Could not send BrakeRelease command" << std::endl;
    return 1;
} else {
    std::cout << "Brake release" << std::endl;
}
```

抱闸是机械臂的制动装置，调用该接口释放制动器。

---

#### 2.8 断开连接

```cpp
my_dashboard->disconnect();
```

完成操作后，调用 `disconnect()` 断开与机器人的连接。

---

#### 2.9 返回退出码

```cpp
return 0;
```

成功执行所有步骤后正常退出。

---


### 3. 编译与运行
使用gcc编译器，编译`dashboard_example.cpp`：
```bash
g++ dashboard_example.cpp -o dashboard_example -lelite-cs-series-sdk
```

运行：
```bash
./dashboard_example <your robot ip>
```
运行后机器人应当会上电并释放抱闸。

---
[>>>下一章：获取机器人状态](./Get-Robot-State.cn.md)
