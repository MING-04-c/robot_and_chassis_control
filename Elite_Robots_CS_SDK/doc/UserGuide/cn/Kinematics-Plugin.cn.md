[首页](./UserGuide.cn.md)

# 运动学插件

## 目标

使用SDK的插件系统与基于KDL的运动学插件，计算机器人的正运动学（FK）和逆运动学（IK）。

## 背景

SDK提供了一套插件机制（基于`ClassLoader`），允许在运行时以共享库的形式加载运动学求解器。这种设计将运动学实现与主SDK解耦，便于未来替换或扩展求解器。

内置运动学插件（`KdlKinematicsPlugin`）基于Orocos项目的[KDL（运动学与动力学库）](https://github.com/orocos/orocos_kinematics_dynamics)。它使用从机器人读取的Modified DH（MDH）参数构建运动学链，并提供以下功能：

- **正运动学（FK）**：根据一组关节角度计算末端执行器位姿。
- **逆运动学（IK）**：使用Levenberg-Marquardt（LMA）算法，根据期望的末端执行器位姿计算关节角度。

在开始之前，请确保已了解 [`PrimaryPortInterface`](./Primary-Port-Message.cn.md) 和 [`RtsiIOInterface`](./Get-Robot-State.cn.md) 的使用方法，示例中会用到它们来获取机器人参数和当前状态。

---

## 任务

### 1. 安装依赖

除SDK标准依赖外，运动学插件还需要安装以下库。

#### Ubuntu

```bash
sudo apt update

# 安装 KDL（orocos-kdl）和 Eigen3
sudo apt install liborocos-kdl-dev

sudo apt install libeigen3-dev
```

#### Windows（Visual Studio + vcpkg）

```shell
.\vcpkg install orocos-kdl

.\vcpkg install eigen3

.\vcpkg integrate install
```

---

### 2. 编译SDK并启用运动学插件

从源码编译SDK时，需要启用 `ELITE_COMPILE_KIN_PLUGIN` 选项：

```shell
cd <本仓库克隆路径>

mkdir build && cd build

cmake -DELITE_COMPILE_KIN_PLUGIN=TRUE ..

make -j

# 安装
sudo make install

sudo ldconfig
```

编译成功后，插件共享库（Linux下为 `libelite_kdl_kinematics.so`，Windows下为 `elite_kdl_kinematics.dll`）将生成到：

```
build/plugin/kinematics/
```

安装后位于：

```
<安装前缀>/lib/elite-cs-series-sdk-plugins/kinematics/
```

---

### 3. 编写正逆运动学计算程序

创建源文件：

```bash
touch kinematics_example.cpp
```

将以下代码复制到 `kinematics_example.cpp`：

```cpp
#include <Elite/Log.hpp>
#include <Elite/PrimaryPortInterface.hpp>
#include <Elite/RobotConfPackage.hpp>
#include <Elite/RobotException.hpp>
#include <Elite/ClassLoader.hpp>
#include <Elite/RtsiIOInterface.hpp>
#include <Elite/KinematicsBase.hpp>

#include <boost/program_options.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono;
using namespace ELITE;
namespace po = boost::program_options;

void logVector6d(const std::string& prefix, const vector6d_t& vec) {
    std::string vec_str = "[";
    for (const auto& val : vec) {
        vec_str += std::to_string(val) + ", ";
    }
    vec_str += "]";
    ELITE_LOG_INFO("%s %s", prefix.c_str(), vec_str.c_str());
}

int main(int argc, const char** argv) {
    std::string robot_ip;

    po::options_description desc("Parameters:");
    desc.add_options()
        ("help,h", "Print help message")
        ("robot-ip", po::value<std::string>(&robot_ip)->required(), "IP address of the robot.");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }
        po::notify(vm);
    } catch (const po::error& e) {
        std::cerr << "Argument error: " << e.what() << "\n\n";
        std::cerr << desc << "\n";
        return 1;
    }

    // 步骤1：从机器人获取MDH参数
    auto primary = std::make_unique<PrimaryPortInterface>();
    auto kin_info = std::make_shared<KinematicsInfo>();

    if (!primary->connect(robot_ip, 30001)) {
        ELITE_LOG_FATAL("Connect robot 30001 port fail.");
        return 1;
    }
    if (!primary->getPackage(kin_info, 200)) {
        ELITE_LOG_FATAL("Get robot kinematics info fail.");
        return 1;
    }
    primary->disconnect();
    ELITE_LOG_INFO("Got robot kinematics info.");

    // 步骤2：通过RTSI获取当前关节角度和TCP位姿
    auto io_interface = std::make_unique<RtsiIOInterface>(
        "output_recipe.txt", "input_recipe.txt", 250);
    if (!io_interface->connect(robot_ip)) {
        ELITE_LOG_FATAL("Connect robot RTSI port fail.");
        return 1;
    }

    auto current_joint = io_interface->getActualJointPositions();
    auto current_tcp   = io_interface->getActualTCPPose();
    ELITE_LOG_INFO("Got current joint positions and TCP pose.");

    // 步骤3：加载运动学插件
    ClassLoader loader("./libelite_kdl_kinematics.so");
    if (!loader.loadLib()) {
        ELITE_LOG_FATAL("Load plugin fail.");
        return 1;
    }

    auto kin_solver = loader.createUniqueInstance<KinematicsBase>(
        "ELITE::KdlKinematicsPlugin");
    if (!kin_solver) {
        ELITE_LOG_FATAL("Create KinematicsBase instance fail.");
        return 1;
    }

    // 步骤4：使用MDH参数配置求解器
    kin_solver->setMDH(kin_info->dh_alpha_, kin_info->dh_a_, kin_info->dh_d_);

    // 步骤5：正运动学
    vector6d_t fk_pose;
    if (!kin_solver->getPositionFK(current_joint, fk_pose)) {
        ELITE_LOG_FATAL("Get FK fail.");
        return 1;
    }

    // 步骤6：逆运动学
    vector6d_t ik_joints;
    KinematicsResult ik_result;
    if (!kin_solver->getPositionIK(current_tcp, current_joint, ik_joints, ik_result)) {
        ELITE_LOG_FATAL("Get IK fail.");
        return 1;
    }

    logVector6d("Current TCP Pose:", current_tcp);
    logVector6d("FK result Pose:", fk_pose);
    logVector6d("IK result Joints:", ik_joints);
    logVector6d("Current Joints:", current_joint);

    return 0;
}
```

---

### 4. 代码说明

#### 4.1 获取MDH参数

```cpp
auto primary = std::make_unique<PrimaryPortInterface>();
auto kin_info = std::make_shared<KinematicsInfo>();

primary->connect(robot_ip, 30001);
primary->getPackage(kin_info, 200);
primary->disconnect();
```

机器人内部存有自身的标定数据（Modified DH参数）。通过 `PrimaryPortInterface` 和 `KinematicsInfo` 数据包从30001端口读取这些参数。获取的三个MDH向量分别为：

- `dh_alpha_`：连杆扭转角（单位：弧度）
- `dh_a_`：连杆长度（单位：米）
- `dh_d_`：连杆偏置（单位：米）

---

#### 4.2 读取当前机器人状态

```cpp
auto io_interface = std::make_unique<RtsiIOInterface>(
    "output_recipe.txt", "input_recipe.txt", 250);
io_interface->connect(robot_ip);

auto current_joint = io_interface->getActualJointPositions();
auto current_tcp   = io_interface->getActualTCPPose();
```

- `getActualJointPositions()` 返回包含6个关节角度（单位：弧度）的 `vector6d_t`。
- `getActualTCPPose()` 返回TCP位姿 `[x, y, z, roll, pitch, yaw]`（米和弧度）的 `vector6d_t`。

这些数据作为FK和IK求解器的输入。

---

#### 4.3 加载插件

```cpp
ClassLoader loader("./libelite_kdl_kinematics.so");
loader.loadLib();
```

`ClassLoader` 从指定路径加载共享库。加载库时，其静态初始化器会执行，`ELITE_CLASS_LOADER_REGISTER_CLASS` 宏会将 `ELITE::KdlKinematicsPlugin` 注册到 `ClassRegistry` 中。

---

#### 4.4 实例化求解器

```cpp
auto kin_solver = loader.createUniqueInstance<KinematicsBase>(
    "ELITE::KdlKinematicsPlugin");
```

`createUniqueInstance<KinematicsBase>()` 通过类名在 `ClassRegistry` 中查找已注册的类，调用其工厂函数，并返回 `std::unique_ptr<KinematicsBase>`。后续所有操作均通过 `KinematicsBase` 接口进行，使代码与具体插件实现解耦。

---

#### 4.5 设置MDH参数

```cpp
kin_solver->setMDH(kin_info->dh_alpha_, kin_info->dh_a_, kin_info->dh_d_);
```

该方法使用机器人标定数据构建KDL运动学链，并初始化FK和IK求解器。**在进行任何FK/IK查询之前，必须先调用此方法。**

---

#### 4.6 正运动学

```cpp
vector6d_t fk_pose;
kin_solver->getPositionFK(current_joint, fk_pose);
```

给定一组关节角度，`getPositionFK()` 计算末端执行器位姿 `[x, y, z, roll, pitch, yaw]`。

---

#### 4.7 逆运动学

```cpp
vector6d_t ik_joints;
KinematicsResult ik_result;
kin_solver->getPositionIK(current_tcp, current_joint, ik_joints, ik_result);
```

给定期望的TCP位姿和初始猜测关节构型（seed），`getPositionIK()` 计算到达该位姿所需的关节角度。结果状态通过 `ik_result.kinematic_error` 返回。

---

### 5. 编译

```bash
g++ -std=c++17 kinematics_example.cpp \
    -lelite-cs-series-sdk \
    -lboost_program_options \
    -o kinematics_example
```

> **注意**：运动学插件库（`libelite_kdl_kinematics.so`）在**运行时**由 `ClassLoader` 动态加载，编译时无需指定。但程序运行时，该文件必须存在于传入 `ClassLoader` 构造函数的路径下。

---

### 6. 运行

```bash
./kinematics_example --robot-ip <robot_ip>
```

预期输出（数值取决于机器人当前位姿）：

```
[INFO] Got robot kinematics info.
[INFO] Got current joint positions and TCP pose.
[INFO] Current TCP Pose: [0.450000, -0.100000, 0.500000, 0.000000, 1.570796, 0.000000, ]
[INFO] FK result Pose:   [0.450000, -0.100000, 0.500000, 0.000000, 1.570796, 0.000000, ]
[INFO] IK result Joints: [0.000000, -1.570796, 1.570796, -1.570796, -1.570796, 0.000000, ]
[INFO] Current Joints:   [0.000000, -1.570796, 1.570796, -1.570796, -1.570796, 0.000000, ]
```

FK结果位姿应与当前TCP位姿一致，IK结果关节角度应与当前关节角度接近。

---

> **提示**：如需实现自定义运动学求解器，可继承 `KinematicsBase`，实现所有纯虚方法，并使用 `ELITE_CLASS_LOADER_REGISTER_CLASS` 进行注册。详情请参阅[API参考文档](../../API/cn/KinematicsBase.cn.md)。
