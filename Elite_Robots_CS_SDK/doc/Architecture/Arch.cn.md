# SDK 的架构

## 1. 架构
下图粗略地展示了SDK与机器人之间的数据流关系，也是SDK的架构：
![SDKDataFlow](./SDKDataFlow.drawio.png)

下面是关于架构图的一些说明：
- Script File：指 `external_control.script` 脚本位于，会被`EliDriver`读取。
- EliDriver：指 `EliDriver` 类。
- Robot：指机器人本体。
- RTSI Client：指SDK中RTSI模块。
- DashboardClient：指SDK中Dashboard模块。

接下来会详细说明 EliDriver 的工作原理，其余的模块直接与机器人的功能对应，可以通过[User Guide](../UserGuide/cn/UserGuide.cn.md)来了解使用，其背后的工作原理并不难理解，这里就不多赘述。

## 2. EliDriver

EliDriver 是一个控制机器人运动、设置机器人配置的类，本文主要阐述其工作原理。在开始说明之前，要先了解一下`ExternalControl`插件的工作流程。  

### 2.1 ExternalControl 插件
简单来说`ExternalControl`是：“请求脚本--->接收与处理脚本--->运行脚本”的流程，在开始运行任务后，插件会向外部控制器发送`request\n`字符串来请求脚本，外部控制器在接收到请求后会向插件发送脚本，脚本的格式如下：
```python
# HEADER_BEGIN
...
# HEADER_END

# NODE_CONTROL_LOOP_BEGINS
...
# NODE_CONTROL_LOOP_ENDS

```
插件依据以上格式将脚本分割为“header”和“control loop”两个部分，这两个部分的内容分别是“声明”和“执行”，例如：
```python
# HEADER_BEGIN
# 声明了printHello()函数
def printHello():
    textmsg("Hello")
# HEADER_END

# NODE_CONTROL_LOOP_BEGINS
# 执行printHello()
printHello()
# NODE_CONTROL_LOOP_ENDS

```
可以猜测到，外部控制器需要创建一个TCP的Server，端口就对应着插件配置的`Custom port`，并且在接收到`request\n`字符串时需要回复一个符合格式要求的脚本。下图简述了整个工作流：
![ExternalControl](./ExternalControl.drawio.png)

### 2.2 EliDriver工作原理

#### 2.2.1 ScriptSender

> `ScriptSender`是一个类，它在构造时创建了一个TCP服务器，当收到`request\n`时就回复脚本。主要代码：`source/Control/ScriptSender.cpp`

既然`ExternalControl`插件是接收脚本、运行脚本的流程，那么`EliDriver`在初始化时会读取脚本——“external_control.script”，并且按照规则写入参数。在收到请求脚本后，会调用 `ScriptSender` 来发送脚本给插件。  


#### 2.2.2 external_control.script

> 脚本位置：`source/resources/external_control.script`

`external_control.script`脚本的基本逻辑是：使用socket接口，连接`EliDriver`外部控制器，通过socket来读取指令和数据并执行。  

我们直接来看脚本中`NODE_CONTROL_LOOP_BEGINS`后面的内容，脚本最开始调用了`socket_open()`创建了几个socket，后文会结合SDK中的代码说明每个socket的作用，这里先主要看`reverse_socket`。这个socket主要是接收外部控制器的运动指令和数据，接着往后看能看到一个`while`循环，这里面就是机器人的控制循环，通过`reverse_socket`接收到的报文来获取指令和点位或者速度等信息。以下是此循环的流程：
![reverse_socket](./ControlScript.drawio.png)

> 这里可以看到`{{SERVER_IP_REPLACE}}`类似格式的字符串，在脚本执行时时会被替换为机器人的IP、端口等参数，这些参数都是`EliDriver`构造时传入的。

在执行“Receive command and data from reverse_socket”步骤的右边，有一个“If time out”的判断，在首次执行时，time out值为0，也就是无限等待（可参考Elite CS 脚本手册“socket_read_binary_integer()”指令的内容）。当接收到指令后，time out值会被更新，更新的值包含在指令中。也就是在执行完指令之后，再次接收`reverse_socket`的指令和数据时会依据上一次接收到的time out值来等待。  
接收到指令与数据之后会判断是否是“停止机器人”的指令，如果是，将会执行“stopj()”指令。

#### 2.2.3 ReverseInterface

> `ReverseInterface`是一个类，它在构造时创建了一个TCP服务器，用于向机器人的`reverse_socket`socket发送指令和数据。主要代码：`source/Control/ReverseInterface.cpp`

我们现在来看`EliDriver`中的`writeServoj()`、`writeSpeedl()`、`writeSpeedj()`、`writeIdle()`接口，对于这几个接口来说，都是都是通过`ReverseInterface`向机器人的`reverse_socket`发送“指令+点位或速度”，脚本在接收到指令后，会启动一个线程来执行运动，而主线程会保持接收`reverse_socket`的数据来更新运动数据或者模式。  


#### 2.2.4 TrajectoryInterface

> `TrajectoryInterface`是一个类，它在构造时创建了一个TCP服务器，用于向机器人的`trajectory_socket`socket发送指令和数据。主要代码：`source/Control/TrajectoryInterface.cpp`。

这个接口需要与`ReverseInterface`一起看，下面是`TrajectoryInterface`的工作流程图：
![trajectory_socket](./ControlScript-trajectory_socket.drawio.png)

`EliDriver`中有以下几个接口`writeTrajectoryPoint()`、`writeTrajectoryControlAction()`、`setTrajectoryResultCallback()`。  

脚本中先是通过`reverse_socket`接收到“Trajectory”的指令，如果是start指令（对应`writeTrajectoryControlAction(TrajectoryControlAction::START, ...)`），就会启动一个线程，此线程的`trajectory_socket`负责接收点位并且运动到点位(对应`writeTrajectoryPoint(...)`)。与此同时的，主线程会保持接收`reverse_socket`的指令和数据，因此在机器人运动带点位之前需要发送“Trajectory”的空操作，来保证`reverse_socket`不会超时。当运动结束时，如果使用了`setTrajectoryResultCallback()`设置回调，就会调用此回调函数，并告知结果。


#### 2.2.5 ScriptCommandInterface

> `ScriptCommandInterface`是一个类，它在构造时创建了一个TCP服务器，用于向机器人的`script_command_socket`socket发送指令和数据。主要代码：`source/Control/ScriptCommandInterface.cpp`。

`external_control.script`脚本在打开socket连接的时候，打开了一个名为“script_command_socket”的socket连接，并且在初始化`script_command_thread_handle`变量时，直接创建了一个线程，这个线程会持续接收来自“script_command_socket”的指令与数据，并且执行相应的动作，例如设置负载、设置工具电压等。这个接口主要用于机器人的配置。

