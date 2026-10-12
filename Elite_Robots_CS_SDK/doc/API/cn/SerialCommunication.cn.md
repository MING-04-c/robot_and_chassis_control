# RS485串口通讯

## 摘要

机器人的末端和控制柜有RS485通讯接口，SDK 提供了相应的功能。

SDK 读写机器人串口本质是，机器人将 RS485 接口的数据转发到指定的 TCP 端口，SDK直接去读写该TCP端口。


# SerialConfig 类

## 描述

包含了串口波特率、奇偶校验等配置变量，以及枚举定义。

## 头文件
```cpp
#include <Elite/SerialCommunication.hpp>
```

## 参数

- `baud_rate`
    - 类型：`SerialConfig::BaudRate`
    - 描述：串口波特率。缺省值为`BR_115200`，即115200波特率。

- `parity`
    - 类型：`SerialConfig::Parity`
    - 描述：串口奇偶校验。缺省值为`NONE`，即无奇偶校验。

- `stop_bits`
    - 类型：`SerialConfig::StopBits`
    - 描述：串口停止位。缺省值为`ONE`，即停止位为1。


# SerialCommunication 类

## 描述

串口转发的 TCP 客户端。

## 头文件
```cpp
#include <Elite/SerialCommunication.hpp>
```

## 接口

---

### ***连接***
```cpp
bool connect(int timeout_ms)
```
- ***功能***
    
    连接到机器人串口转发的服务端。

- ***参数***
    - timeout_ms：超时，单位：毫秒
    
- ***返回值***：连接成功返回true。

---

### ***断开连接***
```cpp
void disconnect()
```

- ***功能***
    
    断开连接。

---

### ***向串口写数据***
```cpp
int write(const uint8_t* data, size_t size)
```

- ***功能***
    
    向串口写数据

- ***参数***
    - data：数据缓存
    
    - size：数据大小
    
- ***返回值***：写入的大小。

---

### ***读取串口数据***
```cpp
int read(uint8_t* data, size_t size, int timeout_ms)
```

- ***功能***
    
    读取串口数据

- ***参数***
    - data：数据缓存
    
    - size：数据大小
    
    - timeout_ms：超时时间，小于等于0时，视为无限等待。
    
- ***返回值***：读取的大小。

---

### ***是否连接到服务端***
```cpp
bool isConnected()
```

- ***功能***
    
    是否连接到服务端

    
- ***返回值***：如果连接到服务端，则返回true

---

### ***获取socat PID***
```cpp
int getSocatPid() const
```

- ***功能***
    
    获取机器人控制柜上进行端口映射socat的进程号

    
- ***返回值***：socat的进程号。如果小于0，则是无效的PID。

--- 