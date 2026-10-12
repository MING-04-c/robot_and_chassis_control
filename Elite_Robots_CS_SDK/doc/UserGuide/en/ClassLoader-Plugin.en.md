[Home](./UserGuide.en.md)

# Plugin System: ClassLoader

## Objective

Use the SDK's `ClassLoader` to write a custom plugin, build it as a shared library, and load it at runtime without any compile-time dependency on the plugin implementation.

## Background

The SDK provides a lightweight plugin system built on two components:

- **`ClassLoader`**: Loads a shared library (`.so` / `.dll`) at runtime and creates instances of registered classes by name.
- **`ELITE_CLASS_LOADER_REGISTER_CLASS`**: A macro placed inside the plugin shared library that automatically registers a derived class into the SDK's `ClassRegistry` when the library is loaded.

This mechanism is modelled after the ROS 2 `pluginlib` pattern and is useful whenever you want to:

- Decouple an algorithm implementation from its interface.
- Select or swap implementations at runtime without recompiling the application.
- Distribute plugin implementations as independent shared libraries.

The built-in [Kinematics Plugin](./Kinematics-Plugin.en.md) is itself an example of this system. This guide shows how to create your own plugin from scratch.

For the complete API reference, see [ClassLoader API](../../API/en/ClassLoader.en.md).

> ### ⚠️ You must use the SDK shared library
>
> The SDK produces **both** a static library and a shared (dynamic) library.  
> **ClassLoader only works correctly when every binary in your process links the SDK's shared library.**
>
> `ClassRegistry` is a process-wide singleton inside the SDK. If you link the **static** library, each binary gets its own private copy of `ClassRegistry`. The plugin registers into one copy; the application looks up classes in a different copy — so every `createUniqueInstance` call returns `nullptr`.
>
> **Rule:** link `elite_cs_series_sdk::shared` for **both** the plugin shared library and the loader application. Never use `elite_cs_series_sdk::static` when ClassLoader is involved.

---

## Task

This guide builds a minimal but complete end-to-end example:

- **Interface**: `AlgorithmBase` — an abstract base class with a single `compute()` method.
- **Plugin**: `SquarePlugin` — a concrete implementation that squares its input.
- **Loader**: a small program that uses `ClassLoader` to load the plugin and call `compute()`.

---

### 1. Define the Interface Header

Create a header file `AlgorithmBase.hpp` that defines the abstract interface:

```cpp
// AlgorithmBase.hpp
#pragma once
#include <string>

class AlgorithmBase {
public:
    virtual ~AlgorithmBase() = default;

    // Returns the name of this algorithm.
    virtual std::string name() const = 0;

    // Performs the computation and returns the result.
    virtual double compute(double input) const = 0;
};
```

> **Note**: The base class **must** have a virtual destructor so that instances can be safely deleted through a base-class pointer.

---

### 2. Implement the Plugin

Create a source file `square_plugin.cpp` that implements `AlgorithmBase` and registers the implementation:

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

// Register SquarePlugin as a plugin implementing AlgorithmBase.
// The first argument must be the fully-qualified class name (including namespace if any).
ELITE_CLASS_LOADER_REGISTER_CLASS(SquarePlugin, AlgorithmBase)
```

#### Key points

- `ELITE_CLASS_LOADER_REGISTER_CLASS(SquarePlugin, AlgorithmBase)` must appear in a **`.cpp` file**, not a header.
- The macro uses the **stringified class name** `"SquarePlugin"` for lookup — this is the name you will pass to `createUniqueInstance()` later.
- If the class is inside a namespace (e.g., `MyNS`), the registration name includes the namespace: `ELITE_CLASS_LOADER_REGISTER_CLASS(MyNS::SquarePlugin, AlgorithmBase)`, and the lookup string must also be `"MyNS::SquarePlugin"`.

---

### 3. Build the Plugin as a Shared Library

Create a `CMakeLists.txt` to build `square_plugin.cpp` as a shared library:

```cmake
cmake_minimum_required(VERSION 3.16)
project(square_plugin_example CXX)

# ---- Plugin shared library ----
add_library(square_plugin SHARED square_plugin.cpp)

# AlgorithmBase.hpp must be visible to the plugin
target_include_directories(square_plugin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# ✅ Link the SDK shared library — required for ClassRegistry
find_package(elite-cs-series-sdk REQUIRED)
target_link_libraries(square_plugin PRIVATE elite_cs_series_sdk::shared)

# ---- Loader executable ----
add_executable(plugin_loader plugin_loader.cpp)
target_include_directories(plugin_loader PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
# ✅ The application must also link the SDK shared library
target_link_libraries(plugin_loader PRIVATE elite_cs_series_sdk::shared)
```

> **⚠️ Both targets must link `elite_cs_series_sdk::shared`.**  
> The SDK builds a static library **and** a shared library.  
> `ClassRegistry` is a process-wide singleton inside the shared library. If either the plugin or the application links the **static** library instead, they each get a separate copy of `ClassRegistry`, and the plugin's registrations become invisible to the application — `createUniqueInstance` will always return `nullptr`.  
> Never substitute `elite_cs_series_sdk::static` for any binary that participates in the ClassLoader system.

---

### 4. Write the Loader Program

Create `plugin_loader.cpp` that uses `ClassLoader` to load the plugin library and call the plugin:

```cpp
// plugin_loader.cpp
#include "AlgorithmBase.hpp"
#include <Elite/ClassLoader.hpp>
#include <Elite/Log.hpp>
#include <iostream>
#include <memory>

int main(int argc, char** argv) {
    // Path to the plugin shared library.
    // On Linux it is built as libsquare_plugin.so
    const std::string lib_path =
        (argc > 1) ? argv[1] : "./libsquare_plugin.so";

    // Step 1: Create the loader with the library path.
    //         The library is NOT loaded yet at this point.
    ELITE::ClassLoader loader(lib_path);

    // Step 2: Load the shared library.
    //         This triggers the static initializers inside the library,
    //         which execute the ELITE_CLASS_LOADER_REGISTER_CLASS macro
    //         and register SquarePlugin into the ClassRegistry.
    if (!loader.loadLib()) {
        ELITE_LOG_FATAL("Failed to load plugin library: %s", lib_path.c_str());
        return 1;
    }

    // Step 3: Create an instance of the plugin by its registered class name.
    //         The template parameter is the base class type.
    //         The string argument must exactly match the name used in the macro.
    auto algo = loader.createUniqueInstance<AlgorithmBase>("SquarePlugin");
    if (!algo) {
        ELITE_LOG_FATAL("Failed to create SquarePlugin instance.");
        return 1;
    }

    // Step 4: Use the plugin through the base class interface.
    double input = 5.0;
    double result = algo->compute(input);
    std::cout << algo->name() << "(" << input << ") = " << result << std::endl;
    // Expected output: SquarePlugin(5) = 25

    return 0;
}
```

---

### 5. Build and Run

#### Ubuntu

```bash
mkdir build && cd build

cmake ..

make -j

# Run the loader (pass the path to the plugin library as an argument)
./plugin_loader ./libsquare_plugin.so
```

Expected output:

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

## Code Explanation

### 5.1 Library Loading and Registration

```cpp
ELITE::ClassLoader loader(lib_path);
loader.loadLib();
```

`ClassLoader::loadLib()` calls the platform's dynamic library loading function (`dlopen` on Linux, `LoadLibrary` on Windows). As a side effect, the runtime executes all static initializers inside the library. The `ELITE_CLASS_LOADER_REGISTER_CLASS` macro generates a global static proxy object whose constructor calls `ClassRegistry::instance().registerClass(...)`. This is how `SquarePlugin` becomes discoverable by name.

---

### 5.2 Instance Creation

```cpp
auto algo = loader.createUniqueInstance<AlgorithmBase>("SquarePlugin");
```

`createUniqueInstance<AlgorithmBase>()` looks up `"SquarePlugin"` in the `ClassRegistry`, calls its factory function (`new SquarePlugin()` cast to `AlgorithmBase*`), and wraps the result in a `std::unique_ptr<AlgorithmBase>`. All subsequent interaction happens through the `AlgorithmBase` interface — the loader program has no compile-time knowledge of `SquarePlugin`.

---

### 5.3 Lifetime

The `ClassLoader` object keeps the shared library loaded. **Do not destroy the `ClassLoader` while plugin instances are still alive** — doing so unloads the library and leaves dangling vtable pointers.

---

## Summary

| Step | File | Purpose |
|------|------|---------|
| 1 | `AlgorithmBase.hpp` | Define the abstract interface |
| 2 | `square_plugin.cpp` | Implement and register the plugin |
| 3 | `CMakeLists.txt` | Build plugin as shared library |
| 4 | `plugin_loader.cpp` | Load and use the plugin at runtime |

---

> **See also**: [ClassLoader API Reference](../../API/en/ClassLoader.en.md) · [Kinematics Plugin User Guide](./Kinematics-Plugin.en.md)

[>>> Next: Kinematics Plugin](./Kinematics-Plugin.en.md)
