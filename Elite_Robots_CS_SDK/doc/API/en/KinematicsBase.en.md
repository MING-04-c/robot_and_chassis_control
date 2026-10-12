# KinematicsBase Module

## Introduction

The KinematicsBase module defines the interface for kinematics solvers and provides the built-in KDL-based plugin (`KdlKinematicsPlugin`). Together with the `ClassLoader` plugin mechanism, it allows any solver that implements `KinematicsBase` to be loaded at runtime as a shared library.

---

## Header Files

```cpp
// Interface (always required)
#include <Elite/KinematicsBase.hpp>

// Plugin loading (required in application code)
#include <Elite/ClassLoader.hpp>

// Plugin registration macro (required when writing a custom solver plugin)
#include <Elite/ClassRegisterMacro.hpp>
```

---

# 1. KinematicError Enum

```cpp
enum KinematicError {
    OK = 1,
    SOLVER_NOT_ACTIVE,
    NO_SOLUTION
};
```

## Description

Error codes returned as part of a `KinematicsResult` after an IK query.

| Value | Description |
|-------|-------------|
| `OK` | No errors; a valid solution was found. |
| `SOLVER_NOT_ACTIVE` | The solver has not been configured yet (e.g., `setMDH()` has not been called). |
| `NO_SOLUTION` | No valid joint solution was found for the given pose. |

---

# 2. KinematicsResult Struct

```cpp
struct KinematicsResult {
    KinematicError kinematic_error;
};
```

## Description

Carries detailed result information from an IK query.

### Members

| Member | Type | Description |
|--------|------|-------------|
| `kinematic_error` | `KinematicError` | Error code indicating the type of failure, or `KinematicError::OK` on success. |

---

# 3. KinematicsBase Class

```cpp
class KinematicsBase
```

## Description

Abstract base class that defines the interface for all kinematics solver plugins. All methods marked `virtual` must be implemented by concrete plugin classes.

---

## Constants

```cpp
const double DEFAULT_TIMEOUT = 1.0; // seconds
```

Default timeout value used when no timeout argument is supplied to a solver function.

---

## Constructor

```cpp
KinematicsBase() = default;
```

### Description

Default constructor. Initializes base class state.

---

## Destructor

```cpp
virtual ~KinematicsBase() = default;
```

### Description

Virtual destructor. Ensures correct destruction through a base class pointer.

---

## setMDH

```cpp
virtual void setMDH(
    const vector6d_t& alpha,
    const vector6d_t& a,
    const vector6d_t& d
) = 0;
```

### Description

Sets the Modified DH (MDH) parameters used to build the robot's kinematic chain.

**This method must be called before any FK or IK query.**

The MDH parameters can be obtained from the robot itself via `PrimaryPortInterface` and `KinematicsInfo`.

---

### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `alpha` | `const vector6d_t&` | MDH link twist angles (radians), one per joint. |
| `a` | `const vector6d_t&` | MDH link lengths (meters), one per joint. |
| `d` | `const vector6d_t&` | MDH link offsets (meters), one per joint. |

---

### Usage Example

```cpp
// Obtain MDH parameters from the robot
auto primary = std::make_unique<ELITE::PrimaryPortInterface>();
auto kin_info = std::make_shared<ELITE::KinematicsInfo>();
primary->connect(robot_ip, 30001);
primary->getPackage(kin_info, 200);
primary->disconnect();

// Configure the solver
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

### Description

Computes the end-effector pose from a set of joint angles (Forward Kinematics).

---

### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `joint_angles` | `const vector6d_t&` | Input joint angles (radians), one per joint. |
| `poses` | `vector6d_t&` | Output end-effector pose as `[x, y, z, roll, pitch, yaw]` (meters and radians). |

---

### Return Value

| Value | Description |
|-------|-------------|
| `true` | A valid FK solution was computed. |
| `false` | The solver is not configured, or the computation failed. |

---

### Usage Example

```cpp
ELITE::vector6d_t joints = {0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0};
ELITE::vector6d_t pose;

if (kin_solver->getPositionFK(joints, pose)) {
    // pose = [x, y, z, roll, pitch, yaw]
}
```

---

## getPositionIK (single solution)

```cpp
virtual bool getPositionIK(
    const vector6d_t& pose,
    const vector6d_t& near,
    vector6d_t& solution,
    KinematicsResult& result
) const = 0;
```

### Description

Computes a single set of joint angles for a desired end-effector pose (Inverse Kinematics).

The solver returns the solution closest to the seed state (`near`). Random re-seeding is not performed.

---

### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `pose` | `const vector6d_t&` | Desired end-effector pose as `[x, y, z, roll, pitch, yaw]` (meters and radians). |
| `near` | `const vector6d_t&` | Initial guess / seed joint configuration (radians). |
| `solution` | `vector6d_t&` | Output joint angles of the IK solution (radians). |
| `result` | `KinematicsResult&` | Output struct containing the error code. |

---

### Return Value

| Value | Description |
|-------|-------------|
| `true` | A valid IK solution was found. |
| `false` | No solution found, or the solver is not configured. Check `result.kinematic_error` for details. |

---

### Usage Example

```cpp
ELITE::vector6d_t target_tcp = {0.45, -0.10, 0.50, 0.0, 1.5708, 0.0};
ELITE::vector6d_t seed_joints = {0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0};
ELITE::vector6d_t ik_joints;
ELITE::KinematicsResult ik_result;

if (kin_solver->getPositionIK(target_tcp, seed_joints, ik_joints, ik_result)) {
    // ik_joints contains the solution
} else {
    // ik_result.kinematic_error contains the failure reason
}
```

---

## getPositionIK (multiple solutions)

```cpp
virtual bool getPositionIK(
    const vector6d_t& pose,
    const vector6d_t& near,
    std::vector<vector6d_t>& solutions,
    KinematicsResult& result
) const = 0;
```

### Description

Computes one or more sets of joint angles for a desired end-effector pose.

---

### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `pose` | `const vector6d_t&` | Desired end-effector pose as `[x, y, z, roll, pitch, yaw]` (meters and radians). |
| `near` | `const vector6d_t&` | Initial guess / seed joint configuration (radians). |
| `solutions` | `std::vector<vector6d_t>&` | Output vector of joint angle solutions (radians). |
| `result` | `KinematicsResult&` | Output struct containing the error code. |

---

### Return Value

| Value | Description |
|-------|-------------|
| `true` | At least one valid IK solution was found. |
| `false` | No solution found, or the solver is not configured. |

---

## setDefaultTimeout

```cpp
void setDefaultTimeout(double timeout);
```

### Description

Sets the default timeout (in seconds) used by solver functions that accept a timeout argument but are called without one.

---

### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `timeout` | `double` | Timeout value in seconds. |

---

## getDefaultTimeout

```cpp
double getDefaultTimeout() const;
```

### Description

Returns the current default timeout value.

---

### Return Value

Default timeout in seconds.

---

# 4. KdlKinematicsPlugin Class

```cpp
class KdlKinematicsPlugin : public KinematicsBase
```

## Description

Concrete kinematics solver plugin based on the [Orocos KDL library](https://github.com/orocos/orocos_kinematics_dynamics). It implements all methods of `KinematicsBase` using:

- **Forward Kinematics**: KDL recursive FK solver (`ChainFkSolverPos_recursive`)
- **Inverse Kinematics**: KDL Levenberg-Marquardt IK solver (`ChainIkSolverPos_LMA`, tolerance `1e-10`, max iterations `10000`)

This class is provided as a **plugin** and should only be used through the `KinematicsBase` interface after being loaded via `ClassLoader`.

---

## Registered Class Name

```
"ELITE::KdlKinematicsPlugin"
```

This name must be used when calling `ClassLoader::createUniqueInstance<KinematicsBase>()`.

---

## Plugin Library

| Platform | Library file |
|----------|-------------|
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

### Description

Builds the KDL kinematic chain from MDH parameters and (re-)initializes the FK and IK solvers. Thread-safe (protected by an internal mutex).

---

## getPositionFK

```cpp
virtual bool getPositionFK(
    const vector6d_t& joint_angles,
    vector6d_t& poses
) const override;
```

### Description

Computes the end-effector pose using the KDL recursive FK solver. Returns `false` and logs an error if `setMDH()` has not been called.

---

## getPositionIK (single solution)

```cpp
virtual bool getPositionIK(
    const vector6d_t& pose,
    const vector6d_t& near,
    vector6d_t& solution,
    KinematicsResult& result
) const override;
```

### Description

Computes a single IK solution using the KDL LMA solver. Sets `result.kinematic_error` to:

- `KinematicError::OK` on success.
- `KinematicError::SOLVER_NOT_ACTIVE` if `setMDH()` has not been called.
- `KinematicError::NO_SOLUTION` if the LMA solver could not find a solution.

---

## getPositionIK (multiple solutions)

```cpp
virtual bool getPositionIK(
    const vector6d_t& pose,
    const vector6d_t& near,
    std::vector<vector6d_t>& solutions,
    KinematicsResult& result
) const override;
```

### Description

Wraps the single-solution `getPositionIK()` and returns the result in a vector of size 1.

---

# 5. Complete Usage Example

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

    // 1. Get MDH parameters from the robot
    auto primary  = std::make_unique<ELITE::PrimaryPortInterface>();
    auto kin_info = std::make_shared<ELITE::KinematicsInfo>();
    primary->connect(robot_ip, 30001);
    primary->getPackage(kin_info, 200);
    primary->disconnect();

    // 2. Get current robot state via RTSI
    auto rtsi = std::make_unique<ELITE::RtsiIOInterface>(
        "output_recipe.txt", "input_recipe.txt", 250);
    rtsi->connect(robot_ip);
    auto current_joints = rtsi->getActualJointPositions();
    auto current_tcp    = rtsi->getActualTCPPose();

    // 3. Load the plugin
    ELITE::ClassLoader loader("./libelite_kdl_kinematics.so");
    if (!loader.loadLib()) {
        ELITE_LOG_FATAL("Failed to load kinematics plugin.");
        return 1;
    }

    // 4. Instantiate the solver via KinematicsBase interface
    auto solver = loader.createUniqueInstance<ELITE::KinematicsBase>(
        "ELITE::KdlKinematicsPlugin");
    if (!solver) {
        ELITE_LOG_FATAL("Failed to create KdlKinematicsPlugin instance.");
        return 1;
    }

    // 5. Configure the solver
    solver->setMDH(kin_info->dh_alpha_, kin_info->dh_a_, kin_info->dh_d_);

    // 6. Forward kinematics
    ELITE::vector6d_t fk_pose;
    if (!solver->getPositionFK(current_joints, fk_pose)) {
        ELITE_LOG_ERROR("FK computation failed.");
    }

    // 7. Inverse kinematics (single solution)
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

# 6. Writing a Custom Kinematics Plugin

To provide your own kinematics implementation:

1. **Inherit** from `KinematicsBase` and implement all pure virtual methods.
2. **Register** the class using the `ELITE_CLASS_LOADER_REGISTER_CLASS` macro in a `.cpp` file.
3. **Build** as a shared library.

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
        // Build kinematic model
    }

    bool getPositionFK(const ELITE::vector6d_t& joints,
                       ELITE::vector6d_t& pose) const override {
        // Implement FK
        return true;
    }

    bool getPositionIK(const ELITE::vector6d_t& pose,
                       const ELITE::vector6d_t& near,
                       ELITE::vector6d_t& solution,
                       ELITE::KinematicsResult& result) const override {
        // Implement IK
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

Load it the same way as the built-in plugin:

```cpp
ELITE::ClassLoader loader("./libmy_kin_plugin.so");
loader.loadLib();
auto solver = loader.createUniqueInstance<ELITE::KinematicsBase>("MY_NS::MyKinPlugin");
```

---

# 7. Important Notes

1. `setMDH()` **must** be called before any FK or IK query.
2. The plugin shared library must be loaded via `ClassLoader::loadLib()` before `createUniqueInstance()` is called.
3. The class name string passed to `createUniqueInstance()` must exactly match the name used in `ELITE_CLASS_LOADER_REGISTER_CLASS`.
4. The SDK's **dynamic library** must be used; the static library does not support the plugin mechanism.
5. Mixing plugins across different compilers or ABIs is not recommended.
6. Pose vectors follow the convention `[x, y, z, roll, pitch, yaw]` (meters and radians).
