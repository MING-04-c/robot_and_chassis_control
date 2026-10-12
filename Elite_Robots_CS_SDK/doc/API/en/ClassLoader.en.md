# ClassLoader Module

## Introduction

The ClassLoader module provides a lightweight plugin loading and class registration mechanism.  
Through macro definitions and the runtime registry (`ClassRegistry`), it enables:

* Automatic registration of plugin classes (static registration)
* Dynamic library loading
* Runtime object instantiation by class name (factory pattern)

This module is suitable for scenarios requiring dynamic extensibility, such as algorithm modules or feature extension components.

---

## Header Files

```cpp
#include <Elite/ClassLoader.hpp>
#include <Elite/ClassRegisterMacro.hpp>
```

---

## ⚠️ SDK Library Requirement

> **The SDK builds both a static library and a shared (dynamic) library.**  
> **ClassLoader can only be used when the application (and every plugin) links against the SDK's shared library.**  
> **Linking the static library instead will silently break ClassLoader — `createUniqueInstance` will always return `nullptr`.**

**Why?**  
`ClassRegistry` is a process-wide singleton that lives inside the SDK library.  
When the **static** library is linked, each binary (e.g. the main application and a plugin) gets its own **separate copy** of `ClassRegistry`. A class registered in the plugin's copy is invisible to the application's copy, so lookups always fail.  
When the **shared** library is used, all binaries share the same single `ClassRegistry` instance, so registrations are visible across the whole process.

**CMake — always use `elite_cs_series_sdk::shared`:**

```cmake
# ✅ Correct — both the plugin and the application must link the shared target
target_link_libraries(my_plugin    PRIVATE elite_cs_series_sdk::shared)
target_link_libraries(my_app       PRIVATE elite_cs_series_sdk::shared)

# ❌ Wrong — static library breaks ClassLoader
# target_link_libraries(my_app    PRIVATE elite_cs_series_sdk::static)
```

---

# 1. Class Registration Macro

## ELITE_CLASS_LOADER_REGISTER_CLASS

```cpp
ELITE_CLASS_LOADER_REGISTER_CLASS(Derived, Base)
```

### Description

Registers the derived class `Derived` into the class registry, enabling dynamic instantiation at runtime via `ClassLoader`.

Internally, this macro:

* Generates a proxy structure within an anonymous namespace
* Automatically executes registration during the static global object construction phase
* Binds the class name string to a factory function
* Uses `__COUNTER__` to ensure unique registration object names

---

### Parameters

* `Derived`: The derived class type
* `Base`: The base class type

---

### Usage Example

```cpp
class BasePlugin {
public:
    virtual ~BasePlugin() = default;
    virtual void run() = 0;
};

class MyPlugin : public BasePlugin {
public:
    void run() override {
        std::cout << "MyPlugin running" << std::endl;
    }
};

ELITE_CLASS_LOADER_REGISTER_CLASS(MyPlugin, BasePlugin)
```

---

### Registration Principle

This macro is equivalent to executing:

```cpp
ClassRegistry::instance().registerClass(
    "MyPlugin",
    "BasePlugin",
    []() -> void* {
        return static_cast<BasePlugin*>(new MyPlugin());
    }
);
```

Registration occurs during:

* Dynamic library loading (`dlopen` / `LoadLibrary`)
* The static global object construction phase

---

### Important Notes

1. The registration macro must be placed after the class definition.
2. The macro should be placed in a `.cpp` file, not a header file.
3. The plugin class must:
   * Have a parameterless constructor
   * Be safely convertible to `Base*`
4. The dynamic library containing the plugin must be successfully loaded by `ClassLoader`; otherwise, registration will not take effect.

---

# 2. ClassLoader Class

## Class Definition

```cpp
class ClassLoader
```

Used to load dynamic libraries and instantiate registered classes.

---

## Constructor

```cpp
explicit ClassLoader(const std::string& lib_path);
```

### Description

Constructs a ClassLoader object.

**Note:**  
The constructor only stores the dynamic library path; it does not load the library immediately.

---

### Parameters

* `lib_path`: Path to the dynamic library file  
  * Linux: `./libplugin.so`  
  * Windows: `plugin.dll`

---

## Destructor

```cpp
~ClassLoader();
```

### Description

Destroys the ClassLoader object.  
Typically unloads the loaded dynamic library (behavior depends on implementation).

---

## loadLib

```cpp
bool loadLib();
```

### Description

Loads the dynamic library from the specified path.

---

### Return Value

* `true`: Loading succeeded
* `false`: Loading failed

---

### Remarks

* Upon successful loading, global static objects within the dynamic library are constructed.
* At this point, registration macros take effect.
* Plugin classes are written into the `ClassRegistry`.

---

## hasLoadedLib

```cpp
bool hasLoadedLib() const;
```

### Description

Checks whether the dynamic library has been successfully loaded.

---

### Return Value

* `true`: Loaded
* `false`: Not loaded

---

## createUniqueInstance

```cpp
template <typename Base>
std::unique_ptr<Base> createUniqueInstance(
    const std::string& derived_class_name
);
```

---

### Description

Dynamically creates an object instance based on the derived class name.

Internal process:

1. Queries the `ClassRegistry` for:
   * The derived class name
   * The base class type name (`typeid(Base).name()`)
2. Retrieves the corresponding factory function
3. Invokes the factory function to create the object
4. Returns a `std::unique_ptr<Base>`

---

### Template Parameters

* `Base`: The base class type

---

### Parameters

* `derived_class_name`: The name of the derived class (the class name string used during registration)

---

### Return Value

* Success: Returns `std::unique_ptr<Base>`
* Failure: Returns `nullptr`

---

### Usage Example

```cpp
ELITE::ClassLoader loader("./libmy_plugin.so");

if (!loader.loadLib()) {
    std::cerr << "Load library failed\n";
    return;
}

auto plugin = loader.createUniqueInstance<BasePlugin>("MyPlugin");

if (plugin) {
    plugin->run();
}
```

---

### Failure Scenarios

`nullptr` may be returned in the following cases:

1. The dynamic library is not loaded.
2. The class is not registered.
3. The base class type does not match.
4. No corresponding factory function is found in the registry.

---

# 3. Workflow Description

The overall plugin loading workflow is as follows:

```
1. Compile the plugin into a dynamic library.
2. Inside the plugin, use registration macros to register classes.
3. ClassLoader loads the dynamic library.
4. Static registration code executes.
5. The class is written into ClassRegistry.
6. createUniqueInstance creates the object.
```

---

# 5. Important Notes

1. The base class must have a virtual destructor.
2. Plugin classes must be constructible via `new Derived()`.
3. Instances can only be created after the dynamic library is successfully loaded.
4. The class name must exactly match the name used during registration.
5. Mixing plugins across different compilers or ABIs is not recommended.
6. `typeid(Base).name()` may differ across compilers; cross-platform use of plugin libraries is not recommended.
7. If you want to use this feature, you must use the SDK's dynamic library; the static library cannot be used.