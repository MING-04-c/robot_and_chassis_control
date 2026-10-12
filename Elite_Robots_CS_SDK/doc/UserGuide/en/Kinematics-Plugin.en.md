[Home](./UserGuide.en.md)

# Kinematics Plugin

## Objective

Use the SDK's plugin system and KDL-based kinematics plugin to compute forward kinematics (FK) and inverse kinematics (IK) for the robot.

## Background

The SDK provides a plugin mechanism (based on `ClassLoader`) that allows kinematics solvers to be loaded as shared libraries at runtime. This design decouples the kinematics implementation from the main SDK and makes it easy to replace or extend solvers in the future.

The built-in kinematics plugin (`KdlKinematicsPlugin`) is based on the [KDL (Kinematics and Dynamics Library)](https://github.com/orocos/orocos_kinematics_dynamics) from the Orocos project. It uses the Modified DH (MDH) parameters read from the robot to build a kinematic chain and then:

- **Forward Kinematics (FK)**: Computes the end-effector pose from a set of joint angles.
- **Inverse Kinematics (IK)**: Computes joint angles from a desired end-effector pose using the Levenberg-Marquardt (LMA) algorithm.

Before proceeding, ensure you are familiar with the [`PrimaryPortInterface`](./Primary-Port-Message.en.md) and [`RtsiIOInterface`](./Get-Robot-State.en.md) classes, as they are used in the example to retrieve robot parameters and current state.

---

## Task

### 1. Install Dependencies

The kinematics plugin depends on the following libraries, in addition to the standard SDK dependencies.

#### Ubuntu

```bash
sudo apt update

# Install KDL (orocos-kdl) and Eigen3
sudo apt install liborocos-kdl-dev

sudo apt install libeigen3-dev
```

#### Windows (Visual Studio + vcpkg)

```shell
.\vcpkg install orocos-kdl

.\vcpkg install eigen3

.\vcpkg integrate install
```

---

### 2. Build the SDK with the Kinematics Plugin

When building the SDK from source, enable the `ELITE_COMPILE_KIN_PLUGIN` option:

```shell
cd <clone of this repository>

mkdir build && cd build

cmake -DELITE_COMPILE_KIN_PLUGIN=TRUE ..

make -j

# Installation
sudo make install

sudo ldconfig
```

After a successful build, the plugin shared library (`libelite_kdl_kinematics.so` on Linux, `elite_kdl_kinematics.dll` on Windows) will be placed in:

```
build/plugin/kinematics/
```

And installed to:

```
<install prefix>/lib/elite-cs-series-sdk-plugins/kinematics/
```

---

### 3. Write a Program to Compute FK and IK

Create a source file:

```bash
touch kinematics_example.cpp
```

Copy the following code into `kinematics_example.cpp`:

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

    // Step 1: Get MDH parameters from the robot
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

    // Step 2: Get current joint angles and TCP pose via RTSI
    auto io_interface = std::make_unique<RtsiIOInterface>(
        "output_recipe.txt", "input_recipe.txt", 250);
    if (!io_interface->connect(robot_ip)) {
        ELITE_LOG_FATAL("Connect robot RTSI port fail.");
        return 1;
    }

    auto current_joint = io_interface->getActualJointPositions();
    auto current_tcp   = io_interface->getActualTCPPose();
    ELITE_LOG_INFO("Got current joint positions and TCP pose.");

    // Step 3: Load the kinematics plugin
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

    // Step 4: Configure the solver with MDH parameters
    kin_solver->setMDH(kin_info->dh_alpha_, kin_info->dh_a_, kin_info->dh_d_);

    // Step 5: Forward kinematics
    vector6d_t fk_pose;
    if (!kin_solver->getPositionFK(current_joint, fk_pose)) {
        ELITE_LOG_FATAL("Get FK fail.");
        return 1;
    }

    // Step 6: Inverse kinematics
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

### 4. Code Explanation

#### 4.1 Obtain MDH Parameters

```cpp
auto primary = std::make_unique<PrimaryPortInterface>();
auto kin_info = std::make_shared<KinematicsInfo>();

primary->connect(robot_ip, 30001);
primary->getPackage(kin_info, 200);
primary->disconnect();
```

The robot stores its own calibration data (Modified DH parameters) internally. We read them from port 30001 using `PrimaryPortInterface` and the `KinematicsInfo` package. The three MDH vectors retrieved are:

- `dh_alpha_`: Link twist angles (radians)
- `dh_a_`: Link lengths (meters)
- `dh_d_`: Link offsets (meters)

---

#### 4.2 Read Current Robot State

```cpp
auto io_interface = std::make_unique<RtsiIOInterface>(
    "output_recipe.txt", "input_recipe.txt", 250);
io_interface->connect(robot_ip);

auto current_joint = io_interface->getActualJointPositions();
auto current_tcp   = io_interface->getActualTCPPose();
```

- `getActualJointPositions()` returns a `vector6d_t` with the six joint angles in radians.
- `getActualTCPPose()` returns a `vector6d_t` with the TCP pose as `[x, y, z, roll, pitch, yaw]` (meters and radians).

These are used as inputs to the FK and IK solvers.

---

#### 4.3 Load the Plugin

```cpp
ClassLoader loader("./libelite_kdl_kinematics.so");
loader.loadLib();
```

`ClassLoader` loads the shared library from the given path. When the library is loaded, its static initializers run and the `ELITE_CLASS_LOADER_REGISTER_CLASS` macro registers `ELITE::KdlKinematicsPlugin` into the `ClassRegistry`.

---

#### 4.4 Instantiate the Solver

```cpp
auto kin_solver = loader.createUniqueInstance<KinematicsBase>(
    "ELITE::KdlKinematicsPlugin");
```

`createUniqueInstance<KinematicsBase>()` looks up the registered class by name in `ClassRegistry`, invokes its factory function, and returns a `std::unique_ptr<KinematicsBase>`. All subsequent operations use the `KinematicsBase` interface, keeping the code independent of the specific plugin.

---

#### 4.5 Set MDH Parameters

```cpp
kin_solver->setMDH(kin_info->dh_alpha_, kin_info->dh_a_, kin_info->dh_d_);
```

This builds the KDL kinematic chain from the robot's calibration data and initializes the FK and IK solvers. **This must be called before any FK/IK query.**

---

#### 4.6 Forward Kinematics

```cpp
vector6d_t fk_pose;
kin_solver->getPositionFK(current_joint, fk_pose);
```

Given a set of joint angles, `getPositionFK()` computes the end-effector pose `[x, y, z, roll, pitch, yaw]`.

---

#### 4.7 Inverse Kinematics

```cpp
vector6d_t ik_joints;
KinematicsResult ik_result;
kin_solver->getPositionIK(current_tcp, current_joint, ik_joints, ik_result);
```

Given a desired TCP pose and a seed (initial guess) joint configuration, `getPositionIK()` computes the joint angles needed to reach that pose. The result status is returned in `ik_result.kinematic_error`.

---

### 5. Compilation

```bash
g++ -std=c++17 kinematics_example.cpp \
    -lelite-cs-series-sdk \
    -lboost_program_options \
    -o kinematics_example
```

> **Note**: The kinematics plugin library (`libelite_kdl_kinematics.so`) is loaded **at runtime** by `ClassLoader`, so it does not need to be specified at compile time. However, it must be accessible at the path passed to the `ClassLoader` constructor when the program runs.

---

### 6. Run

```bash
./kinematics_example --robot-ip <robot_ip>
```

Expected output (values depend on the robot's actual pose):

```
[INFO] Got robot kinematics info.
[INFO] Got current joint positions and TCP pose.
[INFO] Current TCP Pose: [0.450000, -0.100000, 0.500000, 0.000000, 1.570796, 0.000000, ]
[INFO] FK result Pose:   [0.450000, -0.100000, 0.500000, 0.000000, 1.570796, 0.000000, ]
[INFO] IK result Joints: [0.000000, -1.570796, 1.570796, -1.570796, -1.570796, 0.000000, ]
[INFO] Current Joints:   [0.000000, -1.570796, 1.570796, -1.570796, -1.570796, 0.000000, ]
```

The FK result pose should match the current TCP pose, and the IK result joints should be close to the current joint positions.

---

> **Tip**: To implement your own kinematics solver, inherit from `KinematicsBase`, implement all pure virtual methods, and register it with `ELITE_CLASS_LOADER_REGISTER_CLASS`. See the [API Reference](../../API/en/KinematicsBase.en.md) for details.
