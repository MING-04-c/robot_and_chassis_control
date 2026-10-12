# ClassLoader 模块

## 简介

ClassLoader 模块提供了一套轻量级的插件加载与类注册机制。
通过宏定义与运行时注册表（ClassRegistry）配合，实现：

* 插件类的自动注册（静态注册）
* 动态库加载
* 运行时按类名创建对象实例（工厂模式）

该模块适用于需要动态扩展功能的场景，例如算法模块、功能扩展组件等。

---

## 头文件

```cpp
#include <Elite/ClassLoader.hpp>
#include <Elite/ClassRegisterMacro.hpp>
```

---

## ⚠️ SDK 库类型要求

> **SDK 会同时构建静态库和共享（动态）库。**  
> **使用 ClassLoader 时，应用程序（以及每个插件）必须链接 SDK 的共享库。**  
> **若链接静态库，ClassLoader 将静默失效——`createUniqueInstance` 始终返回 `nullptr`。**

**原因说明：**  
`ClassRegistry` 是存在于 SDK 库中的进程级单例。  
链接**静态库**时，每个二进制文件（例如主程序和插件）各自拥有独立的 `ClassRegistry` 副本，插件中注册的类对主程序不可见，查找始终失败。  
链接**共享库**时，所有二进制文件共享同一个 `ClassRegistry` 实例，注册信息在整个进程中可见。

**CMake——始终使用 `elite_cs_series_sdk::shared`：**

```cmake
# ✅ 正确——插件和应用程序均须链接共享目标
target_link_libraries(my_plugin    PRIVATE elite_cs_series_sdk::shared)
target_link_libraries(my_app       PRIVATE elite_cs_series_sdk::shared)

# ❌ 错误——静态库将导致 ClassLoader 失效
# target_link_libraries(my_app    PRIVATE elite_cs_series_sdk::static)
```

---

# 一、类注册宏

## ELITE_CLASS_LOADER_REGISTER_CLASS

```cpp
ELITE_CLASS_LOADER_REGISTER_CLASS(Derived, Base)
```

### 功能

将派生类 `Derived` 注册到类注册表中，使其能够在运行时通过 `ClassLoader` 动态创建。

该宏内部会：

* 生成一个匿名命名空间中的代理结构体
* 在全局静态对象构造阶段自动执行注册
* 将类名字符串与工厂函数绑定
* 使用 `__COUNTER__` 保证注册对象名称唯一

---

### 参数

* `Derived`：派生类类型
* `Base`：基类类型

---

### 使用示例

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

### 注册原理说明

注册时等价于执行：

```cpp
ClassRegistry::instance().registerClass(
    "MyPlugin",
    "BasePlugin",
    []() -> void* {
        return static_cast<BasePlugin*>(new MyPlugin());
    }
);
```

注册发生在：

* 动态库加载时（dlopen / LoadLibrary）
* 静态全局对象构造阶段

---

### 注意事项

1. 注册宏必须写在类定义之后
2. 宏应写在 `.cpp` 文件中，而不是头文件中
3. 插件类必须：

   * 具有无参构造函数
   * 可安全转换为 `Base*`
4. 插件所在动态库必须被 `ClassLoader` 成功加载，否则注册不会生效

---

# 二、ClassLoader 类

## 类定义

```cpp
class ClassLoader
```

用于加载动态库并创建已注册的类实例。

---

## 构造函数

```cpp
explicit ClassLoader(const std::string& lib_path);
```

### 功能

构造一个类加载器对象。

注意：
构造函数仅保存动态库路径，不会立即加载动态库。

---

### 参数

* `lib_path`：动态库文件路径

  * Linux: `./libplugin.so`
  * Windows: `plugin.dll`

---

## 析构函数

```cpp
~ClassLoader();
```

### 功能

销毁 ClassLoader 对象。

通常会卸载已加载的动态库（具体行为取决于实现）。

---

## loadLib

```cpp
bool loadLib();
```

### 功能

加载指定路径的动态库。

---

### 返回值

* `true`：加载成功
* `false`：加载失败

---

### 说明

* 加载成功后，动态库中的全局静态对象会执行构造
* 此时注册宏会生效
* 插件类会被写入 ClassRegistry

---

## hasLoadedLib

```cpp
bool hasLoadedLib() const;
```

### 功能

判断动态库是否已经成功加载。

---

### 返回值

* `true`：已加载
* `false`：未加载

---

## createUniqueInstance

```cpp
template <typename Base>
std::unique_ptr<Base> createUniqueInstance(
    const std::string& derived_class_name
);
```

---

### 功能

根据派生类名称动态创建对象实例。

内部流程：

1. 从 ClassRegistry 查询：

   * 派生类名
   * 基类类型名（typeid(Base).name()）
2. 获取工厂函数
3. 调用工厂函数创建对象
4. 返回 `std::unique_ptr<Base>`

---

### 模板参数

* `Base`：基类类型

---

### 参数

* `derived_class_name`：派生类名称（注册时使用的类名字符串）

---

### 返回值

* 成功：返回 `std::unique_ptr<Base>`
* 失败：返回 `nullptr`

---

### 使用示例

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

### 失败场景

返回 `nullptr` 的情况包括：

1. 动态库未加载
2. 类未注册
3. 基类类型不匹配
4. 注册表未找到对应工厂函数

---

# 三、工作流程说明

整体插件加载流程如下：

```
1. 编译插件动态库
2. 插件内部使用注册宏注册类
3. ClassLoader 加载动态库
4. 静态注册代码执行
5. 类写入 ClassRegistry
6. createUniqueInstance 创建对象
```

---

# 五、注意事项

1. 基类必须具有虚析构函数
2. 插件类必须可通过 `new Derived()` 构造
3. 动态库必须成功加载后才能创建实例
4. 类名必须与注册时使用的名称完全一致
5. 不建议跨不同编译器或 ABI 混用插件
6. `typeid(Base).name()` 在不同编译器之间可能不同，不建议跨平台混用插件库
7. 如果要使用此功能，只能使用SDK的动态库，不能使用静态库。
