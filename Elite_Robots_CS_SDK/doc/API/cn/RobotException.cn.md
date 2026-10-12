# RobotException

## 简介

机器人在运行过程中如果出现异常，会通过primary端口发送给客户端机器人异常的报文。SDK通过解析这部分报文，获取异常信息。关于异常的详细描述，需要查阅`CS_用户手册_机器人状态报文_v2.14.2.xlsx`里的内容。

---

# RobotException 类

## 简介

机器人异常的基类，表示所有异常共有的基本信息，例如异常类型与时间戳。

## 头文件引用

```cpp
#include <Elite/RobotException.hpp>
```

## 异常类型定义

```cpp
enum class Type : int8_t {
    ROBOT_TCP_DISCONNECTED = -1
    ROBOT_ERROR = 6,
    SCRIPT_RUNTIME = 10
};
```

* `ROBOT_TCP_DISCONNECTED` : 表示与机器人断开了连接
* `ROBOT_ERROR`：表示机器人运行错误。
* `SCRIPT_RUNTIME`：表示运行时异常，如语法错误、脚本执行错误等。

## 构造函数

### ***构造函数***

```cpp
RobotException::RobotException(Type type, uint64_t ts)
```

* **功能**：构造异常对象，指定类型与发生时间。
* **参数**

  * `type`：异常类型（ROBOT_ERROR 或 SCRIPT_RUNTIME）。
  * `ts`：时间戳（单位为微秒）。

## 接口函数

### 获取异常类型

```cpp
Type getType()
```

* **返回值**：异常类型。

---

### 获取时间戳

```cpp
uint64_t getTimestamp()
```

* **返回值**：异常发生的时间戳（微秒）。

---

# RobotError 类

## 简介

表示机器人错误异常，包括错误码、错误来源、级别和附加数据等信息。用于详细描述控制器或硬件级别错误。

## 错误来源模块枚举

```cpp
enum class Source : int
```

* `SAFETY`：安全模块
* `GUI`：示教器界面
* `CONTROLLER`：控制器
* `RTSI`：RTSI 协议
* `JOINT`：关节模块
* `TOOL`：工具模块
* `TP`：示教器
* `JOINT_FPGA`：关节 FPGA
* `TOOL_FPGA`：工具 FPGA

---

## 错误等级枚举

```cpp
enum class Level : int
```

* `INFO`：信息提示
* `WARNING`：警告
* `ERROR`：错误
* `FATAL`：致命错误

---

## 错误数据类型枚举

```cpp
enum class DataType : uint32_t
```

* `NONE`：未知类型或无类型，uint32_t
* `UNSIGNED`：无符号整型，uint32_t
* `SIGNED`：有符号整型，int32_t
* `FLOAT`：浮点数，float
* `HEX`：十六进制数据，无符号整型（建议使用16进制表示）
* `STRING`：字符串
* `JOINT`：关节异常数据，int32_t

---

## 构造函数

```cpp
RobotError::RobotError(uint64_t ts, int code, int sc, Source es, Level el, DataType et, Data data)
```

* **功能**：创建一个错误异常对象。
* **参数**

  * `ts`：时间戳。
  * `code`：错误码。
  * `sc`：子错误码。
  * `es`：错误来源模块。
  * `el`：错误等级。
  * `et`：错误数据类型。
  * `data`：附加数据（可能是字符串、整型、浮点数）。

---

## 接口函数

```cpp
int getErrorCode()
int getSubErrorCode()
Source getErrorSouce()
Level getErrorLevel()
DataType getErrorDataType()
Data getData()
```

* **功能**：分别获取错误码、子错误码、来源模块、错误等级、错误数据类型和附加数据。

---

# RobotRuntimeException 类

## 简介

表示机器人运行时异常，例如脚本执行期间发生的语法错误、运行错误等，通常携带源代码中的行列号及错误描述信息。

## 构造函数

```cpp
RobotRuntimeException::RobotRuntimeException(uint64_t ts, int line, int column, std::string&& msg)
```

* **功能**：构造运行时异常对象。
* **参数**

  * `ts`：时间戳。
  * `line`：错误所在行号。
  * `column`：错误所在列号。
  * `msg`：错误信息。

---

## 接口函数

```cpp
int getLine()
int getColumn()
const std::string& getMessage()
```

* **功能**：分别获取错误的行号、列号和信息描述。

---

# 异常共享指针类型

以下是 SDK 中异常类的共享指针类型定义，用于回调注册与智能内存管理。

```cpp
using RobotExceptionSharedPtr = std::shared_ptr<RobotException>;
using RobotErrorSharedPtr = std::shared_ptr<RobotError>;
using RobotRuntimeExceptionSharedPtr = std::shared_ptr<RobotRuntimeException>;
```

---



