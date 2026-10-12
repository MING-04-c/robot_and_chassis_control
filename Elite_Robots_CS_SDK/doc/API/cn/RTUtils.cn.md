# 实时工具

一些能提升实时性的接口

## 头文件
```cpp
#include <Elite/RtUtils.hpp>
```

## 接口

### 设置实时线程
```cpp
bool setThreadFiFoScheduling(std::thread::native_handle_type& thread, const int priority);
```

- ***功能***

    设置线程为FIFO调度，并设置优先级

- ***参数***

  - `thread`: 线程句柄
  - `priority`: 优先级

- ***返回值***
    - `true` ： 设置成功
    - `false` ：设置失败

### 获取线程最高优先级
```cpp
int getThreadFiFoMaxPriority();
```

- ***功能***

    获取线程最高优先级

- ***返回值***：最高优先级

### 绑定线程到CPU核心
```cpp
bool bindThreadToCpus(std::thread::native_handle_type& thread, const int cpu)
``` 

- ***功能***

    绑定线程到CPU核心

- ***参数***

  - `thread`: 线程句柄
  - `cpu`: CPU核心编号

- ***返回值***
    - `true` ： 绑定成功
    - `false` ：绑定失败
