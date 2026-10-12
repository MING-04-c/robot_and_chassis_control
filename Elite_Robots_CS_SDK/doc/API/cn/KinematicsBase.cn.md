# KinematicsBase 模块

## 简介

KinematicsBase 模块定义了运动学求解器的接口，并提供了内置的基于KDL的插件（`KdlKinematicsPlugin`）。结合 `ClassLoader` 插件机制，任何实现了 `KinematicsBase` 的求解器都可以在运行时以共享库的形式加载。

---

## 头文件

```cpp
// 接口（必须包含）
#include <Elite/KinematicsBase.hpp>

// 插件加载（应用代码中必须包含）
#include <Elite/ClassLoader.hpp>

// 插件注册宏（编写自定义求解器插件时必须包含）
#include <Elite/ClassRegisterMacro.hpp>
```

---

# 一、KinematicError 枚举

```cpp
enum KinematicError {
    OK = 1,
    SOLVER_NOT_ACTIVE,
    NO_SOLUTION
};
```

## 说明

IK查询后通过 `KinematicsResult` 返回的错误码。

| 值 | 说明 |
|----|------|
| `OK` | 无错误；找到有效解。 |
| `SOLVER_NOT_ACTIVE` | 求解器尚未配置（例如，尚未调用 `setMDH()`）。 |
| `NO_SOLUTION` | 未找到给定位姿的有效关节解。 |

---

# 二、KinematicsResult 结构体

```cpp
struct KinematicsResult {
    KinematicError kinematic_error;
};
```

## 说明

携带IK查询的详细结果信息。

### 成员

| 成员 | 类型 | 说明 |
|------|------|------|
| `kinematic_error` | `KinematicError` | 错误码，成功时为 `KinematicError::OK`。 |

---

# 三、KinematicsBase 类

```cpp
class KinematicsBase
```

## 说明

定义所有运动学求解器插件接口的抽象基类。标记为 `virtual` 的方法必须由具体插件类实现。

---

## 常量

```cpp
const double DEFAULT_TIMEOUT = 1.0; // 秒
```

当求解器函数未提供超时参数时使用的默认超时值。

---

## 构造函数

```cpp
KinematicsBase() = default;
```

### 说明

默认构造函数，初始化基类状态。

---

## 析构函数

```cpp
virtual ~KinematicsBase() = default;
```

### 说明

虚析构函数，确保通过基类指针正确析构。

---

## setMDH

```cpp
virtual void setMDH(
    const vector6d_t& alpha,
    const vector6d_t& a,
    const vector6d_t& d
) = 0;
```

### 说明

设置用于构建机器人运动学链的Modified DH（MDH）参数。

**在进行任何FK或IK查询之前，必须调用此方法。**

MDH参数可通过 `PrimaryPortInterface` 和 `KinematicsInfo` 从机器人本体获取。

---

### 参数

| 参数 | 类型 | 说明 |
|------|------|------|
| `alpha` | `const vector6d_t&` | MDH连杆扭转角（弧度），每个关节一个。 |
| `a` | `const vector6d_t&` | MDH连杆长度（米），每个关节一个。 |
| `d` | `const vector6d_t&` | MDH连杆偏置（米），每个关节一个。 |

---

### 使用示例

```cpp
// 从机器人获取MDH参数
auto primary  = std::make_unique<ELITE::PrimaryPortInterface>();
auto kin_info = std::make_shared<ELITE::KinematicsInfo>();
primary->connect(robot_ip, 30001);
primary->getPackage(kin_info, 200);
primary->disconnect();

// 配置求解器
kin_solver->setMDH(kin_info->dh_alpha_, kin_info->dh_a_, kin_info->dh_d_);
```

---

## getPositionFK

```cpp
virtual bool getPositionFK(
    const vector6d_t& joint_angles,
    vector6d_t& poses
) const = 0;
```

### 说明

根据一组关节角度计算末端执行器位姿（正运动学）。

---

### 参数

| 参数 | 类型 | 说明 |
|------|------|------|
| `joint_angles` | `const vector6d_t&` | 输入关节角度（弧度），每个关节一个。 |
| `poses` | `vector6d_t&` | 输出末端执行器位姿 `[x, y, z, roll, pitch, yaw]`（米和弧度）。 |

---

### 返回值

| 值 | 说明 |
|----|------|
| `true` | 成功计算出有效的FK解。 |
| `false` | 求解器未配置或计算失败。 |

---

### 使用示例

```cpp
ELITE::vector6d_t joints = {0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0};
ELITE::vector6d_t pose;

if (kin_solver->getPositionFK(joints, pose)) {
    // pose = [x, y, z, roll, pitch, yaw]
}
```

---

## getPositionIK（单解）

```cpp
virtual bool getPositionIK(
    const vector6d_t& pose,
    const vector6d_t& near,
    vector6d_t& solution,
    KinematicsResult& result
) const = 0;
```

### 说明

计算期望末端执行器位姿对应的单组关节角度（逆运动学）。

求解器返回最接近初始猜测（`near`）的解，不进行随机重新初始化。

---

### 参数

| 参数 | 类型 | 说明 |
|------|------|------|
| `pose` | `const vector6d_t&` | 期望末端执行器位姿 `[x, y, z, roll, pitch, yaw]`（米和弧度）。 |
| `near` | `const vector6d_t&` | 初始猜测/种子关节构型（弧度）。 |
| `solution` | `vector6d_t&` | 输出IK解的关节角度（弧度）。 |
| `result` | `KinematicsResult&` | 输出包含错误码的结构体。 |

---

### 返回值

| 值 | 说明 |
|----|------|
| `true` | 找到有效的IK解。 |
| `false` | 未找到解或求解器未配置。检查 `result.kinematic_error` 了解详情。 |

---

### 使用示例

```cpp
ELITE::vector6d_t target_tcp   = {0.45, -0.10, 0.50, 0.0, 1.5708, 0.0};
ELITE::vector6d_t seed_joints  = {0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0};
ELITE::vector6d_t ik_joints;
ELITE::KinematicsResult ik_result;

if (kin_solver->getPositionIK(target_tcp, seed_joints, ik_joints, ik_result)) {
    // ik_joints 包含解
} else {
    // ik_result.kinematic_error 包含失败原因
}
```

---

## getPositionIK（多解）

```cpp
virtual bool getPositionIK(
    const vector6d_t& pose,
    const vector6d_t& near,
    std::vector<vector6d_t>& solutions,
    KinematicsResult& result
) const = 0;
```

### 说明

计算期望末端执行器位姿对应的一组或多组关节角度。

---

### 参数

| 参数 | 类型 | 说明 |
|------|------|------|
| `pose` | `const vector6d_t&` | 期望末端执行器位姿 `[x, y, z, roll, pitch, yaw]`（米和弧度）。 |
| `near` | `const vector6d_t&` | 初始猜测/种子关节构型（弧度）。 |
| `solutions` | `std::vector<vector6d_t>&` | 输出关节角度解的向量（弧度）。 |
| `result` | `KinematicsResult&` | 输出包含错误码的结构体。 |

---

### 返回值

| 值 | 说明 |
|----|------|
| `true` | 找到至少一个有效的IK解。 |
| `false` | 未找到解或求解器未配置。 |

---

## setDefaultTimeout

```cpp
void setDefaultTimeout(double timeout);
```

### 说明

设置默认超时时间（秒），用于接受超时参数但未显式传入时的求解器函数。

---

### 参数

| 参数 | 类型 | 说明 |
|------|------|------|
| `timeout` | `double` | 超时时间，单位秒。 |

---

## getDefaultTimeout

```cpp
double getDefaultTimeout() const;
```

### 说明

返回当前默认超时时间。

---

### 返回值

默认超时时间，单位秒。

---

# 四、KdlKinematicsPlugin 类

```cpp
class KdlKinematicsPlugin : public KinematicsBase
```

## 说明

基于 [Orocos KDL库](https://github.com/orocos/orocos_kinematics_dynamics) 的具体运动学求解器插件，实现了 `KinematicsBase` 的所有方法：

- **正运动学**：KDL递归FK求解器（`ChainFkSolverPos_recursive`）
- **逆运动学**：KDL Levenberg-Marquardt IK求解器（`ChainIkSolverPos_LMA`，容差 `1e-10`，最大迭代次数 `10000`）

该类以**插件**形式提供，应通过 `ClassLoader` 加载后使用 `KinematicsBase` 接口进行操作。

---

## 注册类名

```
"ELITE::KdlKinematicsPlugin"
```

调用 `ClassLoader::createUniqueInstance<KinematicsBase>()` 时必须使用此名称。

---

## 插件库文件

| 平台 | 库文件 |
|------|--------|
| Linux | `libelite_kdl_kinematics.so` |
| Windows | `elite_kdl_kinematics.dll` |

---

## setMDH

```cpp
virtual void setMDH(
    const vector6d_t& alpha,
    const vector6d_t& a,
    const vector6d_t& d
) override;
```

### 说明

根据MDH参数构建KDL运动学链，并（重新）初始化FK和IK求解器。线程安全（内部使用互斥锁保护）。

---

## getPositionFK

```cpp
virtual bool getPositionFK(
    const vector6d_t& joint_angles,
    vector6d_t& poses
) const override;
```

### 说明

使用KDL递归FK求解器计算末端执行器位姿。若未调用 `setMDH()`，则返回 `false` 并记录错误日志。

---

## getPositionIK（单解）

```cpp
virtual bool getPositionIK(
    const vector6d_t& pose,
    const vector6d_t& near,
    vector6d_t& solution,
    KinematicsResult& result
) const override;
```

### 说明

使用KDL LMA求解器计算单个IK解。`result.kinematic_error` 设置为：

- `KinematicError::OK`：成功。
- `KinematicError::SOLVER_NOT_ACTIVE`：未调用 `setMDH()`。
- `KinematicError::NO_SOLUTION`：LMA求解器未找到解。

---

## getPositionIK（多解）

```cpp
virtual bool getPositionIK(
    const vector6d_t& pose,
    const vector6d_t& near,
    std::vector<vector6d_t>& solutions,
    KinematicsResult& result
) const override;
```

### 说明

对单解 `getPositionIK()` 的封装，将结果放入大小为1的向量中返回。

---

# 五、完整使用示例

```cpp
#include <Elite/ClassLoader.hpp>
#include <Elite/KinematicsBase.hpp>
#include <Elite/PrimaryPortInterface.hpp>
#include <Elite/RobotConfPackage.hpp>
#include <Elite/RtsiIOInterface.hpp>
#include <Elite/Log.hpp>
#include <memory>

int main() {
    const std::string robot_ip = "192.168.1.100";

    // 1. 从机器人获取MDH参数
    auto primary  = std::make_unique<ELITE::PrimaryPortInterface>();
    auto kin_info = std::make_shared<ELITE::KinematicsInfo>();
    primary->connect(robot_ip, 30001);
    primary->getPackage(kin_info, 200);
    primary->disconnect();

    // 2. 通过RTSI获取当前机器人状态
    auto rtsi = std::make_unique<ELITE::RtsiIOInterface>(
        "output_recipe.txt", "input_recipe.txt", 250);
    rtsi->connect(robot_ip);
    auto current_joints = rtsi->getActualJointPositions();
    auto current_tcp    = rtsi->getActualTCPPose();

    // 3. 加载插件
    ELITE::ClassLoader loader("./libelite_kdl_kinematics.so");
    if (!loader.loadLib()) {
        ELITE_LOG_FATAL("Failed to load kinematics plugin.");
        return 1;
    }

    // 4. 通过KinematicsBase接口实例化求解器
    auto solver = loader.createUniqueInstance<ELITE::KinematicsBase>(
        "ELITE::KdlKinematicsPlugin");
    if (!solver) {
        ELITE_LOG_FATAL("Failed to create KdlKinematicsPlugin instance.");
        return 1;
    }

    // 5. 配置求解器
    solver->setMDH(kin_info->dh_alpha_, kin_info->dh_a_, kin_info->dh_d_);

    // 6. 正运动学
    ELITE::vector6d_t fk_pose;
    if (!solver->getPositionFK(current_joints, fk_pose)) {
        ELITE_LOG_ERROR("FK computation failed.");
    }

    // 7. 逆运动学（单解）
    ELITE::vector6d_t ik_joints;
    ELITE::KinematicsResult ik_result;
    if (!solver->getPositionIK(current_tcp, current_joints, ik_joints, ik_result)) {
        ELITE_LOG_ERROR("IK computation failed, error: %d",
                        static_cast<int>(ik_result.kinematic_error));
    }

    return 0;
}
```

---

# 六、编写自定义运动学插件

如需提供自己的运动学实现：

1. **继承** `KinematicsBase`，实现所有纯虚方法。
2. 在 `.cpp` 文件中使用 `ELITE_CLASS_LOADER_REGISTER_CLASS` 宏**注册**类。
3. **编译**为共享库。

```cpp
// my_kin_plugin.cpp
#include <Elite/KinematicsBase.hpp>
#include <Elite/ClassRegisterMacro.hpp>

namespace MY_NS {

class MyKinPlugin : public ELITE::KinematicsBase {
public:
    void setMDH(const ELITE::vector6d_t& alpha,
                const ELITE::vector6d_t& a,
                const ELITE::vector6d_t& d) override {
        // 构建运动学模型
    }

    bool getPositionFK(const ELITE::vector6d_t& joints,
                       ELITE::vector6d_t& pose) const override {
        // 实现正运动学
        return true;
    }

    bool getPositionIK(const ELITE::vector6d_t& pose,
                       const ELITE::vector6d_t& near,
                       ELITE::vector6d_t& solution,
                       ELITE::KinematicsResult& result) const override {
        // 实现逆运动学
        result.kinematic_error = ELITE::KinematicError::OK;
        return true;
    }

    bool getPositionIK(const ELITE::vector6d_t& pose,
                       const ELITE::vector6d_t& near,
                       std::vector<ELITE::vector6d_t>& solutions,
                       ELITE::KinematicsResult& result) const override {
        solutions.resize(1);
        return getPositionIK(pose, near, solutions[0], result);
    }
};

} // namespace MY_NS

ELITE_CLASS_LOADER_REGISTER_CLASS(MY_NS::MyKinPlugin, ELITE::KinematicsBase)
```

加载方式与内置插件相同：

```cpp
ELITE::ClassLoader loader("./libmy_kin_plugin.so");
loader.loadLib();
auto solver = loader.createUniqueInstance<ELITE::KinematicsBase>("MY_NS::MyKinPlugin");
```

---

# 七、注意事项

1. 在进行任何FK或IK查询之前，**必须**调用 `setMDH()`。
2. 必须先通过 `ClassLoader::loadLib()` 加载插件共享库，才能调用 `createUniqueInstance()`。
3. 传给 `createUniqueInstance()` 的类名字符串必须与 `ELITE_CLASS_LOADER_REGISTER_CLASS` 中使用的名称完全一致。
4. 必须使用SDK的**动态库**；静态库不支持插件机制。
5. 不建议跨不同编译器或ABI混用插件。
6. 位姿向量遵循 `[x, y, z, roll, pitch, yaw]` 约定（米和弧度）。
