[首页](./UserGuide.cn.md)

# 插件系统：ClassLoader

## 目标

使用SDK的 `ClassLoader`，编写一个自定义插件，将其构建为共享库，并在运行时动态加载——应用程序在编译时无需依赖插件的具体实现。

## 背景

SDK提供了一套基于两个组件的轻量级插件系统：

- **`ClassLoader`**：在运行时加载共享库（`.so` / `.dll`），并按类名创建已注册类的实例。
- **`ELITE_CLASS_LOADER_REGISTER_CLASS`**：放在插件共享库内部的宏，当库被加载时，自动将派生类注册到SDK的 `ClassRegistry` 中。

该机制参考了ROS 2的 `pluginlib` 设计，适用于以下场景：

- 将算法实现与接口解耦。
- 在运行时选择或替换实现，无需重新编译应用程序。
- 以独立共享库的形式分发插件实现。

SDK内置的[运动学插件](./Kinematics-Plugin.cn.md)本身就是这套系统的使用示例。本指南将从零开始演示如何创建自定义插件。

完整的API参考请见 [ClassLoader API](../../API/cn/ClassLoader.cn.md)。

> ### ⚠️ 必须使用 SDK 共享库
>
> SDK 会同时构建**静态库**和**共享（动态）库**。  
> **只有当进程中的每个二进制文件都链接 SDK 共享库时，ClassLoader 才能正常工作。**
>
> `ClassRegistry` 是存在于 SDK 共享库中的进程级单例。若链接**静态库**，每个二进制文件各自拥有独立的 `ClassRegistry` 副本，插件在其副本中完成注册，而应用程序在另一个副本中查找类——导致每次 `createUniqueInstance` 调用都返回 `nullptr`。
>
> **规则：** 插件共享库和加载程序可执行文件**都必须**链接 `elite_cs_series_sdk::shared`，不能使用 `elite_cs_series_sdk::static`。

---

## 任务

本指南构建一个完整的端到端示例：

- **接口**：`AlgorithmBase` — 带有单个 `compute()` 方法的抽象基类。
- **插件**：`SquarePlugin` — 对输入值求平方的具体实现。
- **加载程序**：使用 `ClassLoader` 加载插件并调用 `compute()` 的小程序。

---

### 1. 定义接口头文件

创建头文件 `AlgorithmBase.hpp`，定义抽象接口：

```cpp
// AlgorithmBase.hpp
#pragma once
#include <string>

class AlgorithmBase {
public:
    virtual ~AlgorithmBase() = default;

    // 返回算法名称
    virtual std::string name() const = 0;

    // 执行计算并返回结果
    virtual double compute(double input) const = 0;
};
```

> **注意**：基类**必须**具有虚析构函数，以确保通过基类指针删除实例时能正确析构。

---

### 2. 实现插件

创建源文件 `square_plugin.cpp`，实现 `AlgorithmBase` 并注册：

```cpp
// square_plugin.cpp
#include "AlgorithmBase.hpp"
#include <Elite/ClassRegisterMacro.hpp>
#include <string>
#include <cmath>

class SquarePlugin : public AlgorithmBase {
public:
    SquarePlugin()  = default;
    ~SquarePlugin() = default;

    std::string name() const override {
        return "SquarePlugin";
    }

    double compute(double input) const override {
        return input * input;
    }
};

// 将 SquarePlugin 注册为实现 AlgorithmBase 的插件。
// 第一个参数必须是完整限定类名（如有命名空间则需包含）。
ELITE_CLASS_LOADER_REGISTER_CLASS(SquarePlugin, AlgorithmBase)
```

#### 关键点

- `ELITE_CLASS_LOADER_REGISTER_CLASS(SquarePlugin, AlgorithmBase)` 必须写在 **`.cpp` 文件**中，不能写在头文件中。
- 该宏使用**字符串化的类名** `"SquarePlugin"` 进行查找——这是后续调用 `createUniqueInstance()` 时传入的名称。
- 如果类在命名空间内（例如 `MyNS`），注册名称包含命名空间：`ELITE_CLASS_LOADER_REGISTER_CLASS(MyNS::SquarePlugin, AlgorithmBase)`，查找字符串也必须为 `"MyNS::SquarePlugin"`。

---

### 3. 将插件构建为共享库

创建 `CMakeLists.txt`，将 `square_plugin.cpp` 编译为共享库：

```cmake
cmake_minimum_required(VERSION 3.16)
project(square_plugin_example CXX)

# ---- 插件共享库 ----
add_library(square_plugin SHARED square_plugin.cpp)

# AlgorithmBase.hpp 需对插件可见
target_include_directories(square_plugin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# ✅ 链接 SDK 共享库——ClassRegistry 依赖此库
find_package(elite-cs-series-sdk REQUIRED)
target_link_libraries(square_plugin PRIVATE elite_cs_series_sdk::shared)

# ---- 加载程序可执行文件 ----
add_executable(plugin_loader plugin_loader.cpp)
target_include_directories(plugin_loader PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
# ✅ 应用程序同样必须链接 SDK 共享库
target_link_libraries(plugin_loader PRIVATE elite_cs_series_sdk::shared)
```

> **⚠️ 两个目标均须链接 `elite_cs_series_sdk::shared`。**  
> SDK 会同时构建静态库和共享库。  
> `ClassRegistry` 是共享库中的进程级单例。若插件或应用程序任何一方链接**静态库**，各自会拥有独立的 `ClassRegistry` 副本，插件的注册信息对应用程序不可见——`createUniqueInstance` 将始终返回 `nullptr`。  
> 参与 ClassLoader 系统的所有二进制文件，均不得使用 `elite_cs_series_sdk::static`。

---

### 4. 编写加载程序

创建 `plugin_loader.cpp`，使用 `ClassLoader` 加载插件库并调用插件：

```cpp
// plugin_loader.cpp
#include "AlgorithmBase.hpp"
#include <Elite/ClassLoader.hpp>
#include <Elite/Log.hpp>
#include <iostream>
#include <memory>

int main(int argc, char** argv) {
    // 插件共享库的路径
    // Linux 下编译产物为 libsquare_plugin.so
    const std::string lib_path =
        (argc > 1) ? argv[1] : "./libsquare_plugin.so";

    // 步骤1：创建加载器对象，指定库路径。
    //         此时库尚未加载。
    ELITE::ClassLoader loader(lib_path);

    // 步骤2：加载共享库。
    //         这会触发库内部的静态初始化器，
    //         执行 ELITE_CLASS_LOADER_REGISTER_CLASS 宏，
    //         将 SquarePlugin 注册到 ClassRegistry 中。
    if (!loader.loadLib()) {
        ELITE_LOG_FATAL("Failed to load plugin library: %s", lib_path.c_str());
        return 1;
    }

    // 步骤3：通过注册的类名创建插件实例。
    //         模板参数为基类类型。
    //         字符串参数必须与宏中使用的名称完全一致。
    auto algo = loader.createUniqueInstance<AlgorithmBase>("SquarePlugin");
    if (!algo) {
        ELITE_LOG_FATAL("Failed to create SquarePlugin instance.");
        return 1;
    }

    // 步骤4：通过基类接口使用插件。
    double input = 5.0;
    double result = algo->compute(input);
    std::cout << algo->name() << "(" << input << ") = " << result << std::endl;
    // 预期输出：SquarePlugin(5) = 25

    return 0;
}
```

---

### 5. 编译与运行

#### Ubuntu

```bash
mkdir build && cd build

cmake ..

make -j

# 运行加载程序（以插件库路径作为参数）
./plugin_loader ./libsquare_plugin.so
```

预期输出：

```
SquarePlugin(5) = 25
```

#### Windows

```shell
mkdir build && cd build

cmake -DCMAKE_TOOLCHAIN_FILE=<your vcpkg path>\scripts\buildsystems\vcpkg.cmake ..

cmake --build ./ --config Release

.\Release\plugin_loader.exe .\Release\square_plugin.dll
```

---

## 代码说明

### 5.1 库加载与注册

```cpp
ELITE::ClassLoader loader(lib_path);
loader.loadLib();
```

`ClassLoader::loadLib()` 调用平台的动态库加载函数（Linux上为 `dlopen`，Windows上为 `LoadLibrary`）。作为副作用，运行时会执行库内所有静态初始化器。`ELITE_CLASS_LOADER_REGISTER_CLASS` 宏生成一个全局静态代理对象，其构造函数调用 `ClassRegistry::instance().registerClass(...)`。这就是 `SquarePlugin` 能够按名称被发现的原理。

---

### 5.2 实例创建

```cpp
auto algo = loader.createUniqueInstance<AlgorithmBase>("SquarePlugin");
```

`createUniqueInstance<AlgorithmBase>()` 在 `ClassRegistry` 中查找 `"SquarePlugin"`，调用其工厂函数（`new SquarePlugin()` 转型为 `AlgorithmBase*`），并将结果包装在 `std::unique_ptr<AlgorithmBase>` 中返回。后续所有交互都通过 `AlgorithmBase` 接口进行——加载程序在编译时不需要了解 `SquarePlugin` 的任何细节。

---

### 5.3 生命周期

`ClassLoader` 对象保持共享库处于加载状态。**在插件实例仍然存活期间，不要销毁 `ClassLoader`**——否则库会被卸载，留下悬空的虚函数表指针。

---

## 小结

| 步骤 | 文件 | 作用 |
|------|------|------|
| 1 | `AlgorithmBase.hpp` | 定义抽象接口 |
| 2 | `square_plugin.cpp` | 实现并注册插件 |
| 3 | `CMakeLists.txt` | 将插件编译为共享库 |
| 4 | `plugin_loader.cpp` | 运行时加载并使用插件 |

---

> **参见**：[ClassLoader API 参考](../../API/cn/ClassLoader.cn.md) · [运动学插件使用指南](./Kinematics-Plugin.cn.md)

[>>> 下一章：运动学插件](./Kinematics-Plugin.cn.md)
