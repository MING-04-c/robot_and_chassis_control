[Home](./UserGuide.en.md)  

# Communicating with the Robot's Primary Port  

## Objective  
Call the SDK interfaces to communicate with the robot's primary port (30001), implementing packet parsing, exception retrieval, and script sending functionality.  

## Background  
The primary port is a communication interface for Elite CS series robots. It transmits robot status data at 10Hz and can receive and execute scripts. For detailed protocol specifications, please refer to the documentation available on the official website.  

## Task  

### 1. Write a Simple Client Program  

Create a code file:  
```bash  
touch primary_example.cpp  
```  

Copy the following code into `primary_example.cpp` using your preferred text editor:  

```cpp  
#include <Elite/Log.hpp>  
#include <Elite/PrimaryPortInterface.hpp>  
#include <Elite/RobotConfPackage.hpp>  
#include <Elite/RobotException.hpp>  

#include <chrono>  
#include <iostream>  
#include <memory>  
#include <string>  
#include <thread>  

using namespace std::chrono;  

// Callback for robot exceptions  
void robotExceptionCb(ELITE::RobotExceptionSharedPtr ex) {  
    if (ex->getType() == ELITE::RobotException::Type::RUNTIME) {  
        auto r_ex = std::static_pointer_cast<ELITE::RobotRuntimeException>(ex);  
        ELITE_LOG_INFO("Robot throw exception: %s", r_ex->getMessage().c_str());  
    }  
}  

int main(int argc, const char** argv) {  
    if (argc < 2) {  
        std::cout << "Must provide robot IP. Example: ./primary_example aaa.bbb.ccc.ddd" << std::endl;  
        return 1;  
    }  
    std::string robot_ip = argv[1];  

    auto primary = std::make_unique<ELITE::PrimaryPortInterface>();  
    auto kin = std::make_shared<ELITE::KinematicsInfo>();  

    primary->connect(robot_ip, 30001);  
    primary->registerRobotExceptionCallback(robotExceptionCb);  
    primary->getPackage(kin, 200);  

    std::string dh_param = "\n\tDH parameter a: ";  
    for (auto i : kin->dh_a_) {  
        dh_param += std::to_string(i);  
        dh_param += '\t';  
    }  
    dh_param += '\n';  

    dh_param += "\n\tDH parameter d: ";  
    for (auto i : kin->dh_d_) {  
        dh_param += std::to_string(i);  
        dh_param += '\t';  
    }  
    dh_param += '\n';  

    dh_param += "\n\tDH parameter alpha: ";  
    for (auto i : kin->dh_alpha_) {  
        dh_param += std::to_string(i);  
        dh_param += '\t';  
    }  
    dh_param += '\n';  

    ELITE_LOG_INFO("%s", dh_param.c_str());  

    std::string script = "def hello():\n\ttextmsg(\"hello world\")\nend\n";  
    primary->sendScript(script);  

    script = "def exFunc():\n\t1abcd\nend\n";  
    primary->sendScript(script);  

    // Wait for robot exception  
    std::this_thread::sleep_for(1s);  
    primary->disconnect();  

    return 0;  
}  
```  

### 2. Code Explanation  
This code demonstrates how to use the **Elite SDK** to establish primary communication with the robot via **PrimaryPortInterface**, retrieve kinematic parameters (DH parameters), send custom scripts, and handle runtime exceptions. The main workflow includes:  

1. Connecting to the robot's primary port (30001).  
2. Registering an exception callback to handle runtime exceptions.  
3. Retrieving and printing DH parameters.  
4. Sending both valid and invalid scripts to trigger exception callbacks.  
5. Waiting to receive exception information before disconnecting.  

#### 2.1 Headers and Namespaces  
```cpp  
#include <Elite/Log.hpp>  
#include <Elite/PrimaryPortInterface.hpp>  
#include <Elite/RobotConfPackage.hpp>  
#include <Elite/RobotException.hpp>  

#include <chrono>  
#include <iostream>  
#include <memory>  
#include <string>  
#include <thread>  
```  

* **Elite SDK Components**  
  - `Log.hpp`: Logging interface for printing messages.  
  - `PrimaryPortInterface.hpp`: Primary communication port class for data transmission.  
  - `RobotConfPackage.hpp`: Robot configuration structure (includes DH parameters).  
  - `RobotException.hpp`: Robot exception class definitions.  

* **Standard Library**  
  - `<chrono>`, `<thread>`: For timing delays.  
  - `<memory>`: Smart pointers.  
  - `<iostream>`, `<string>`: Basic I/O and string handling.  

#### 2.2 Exception Callback  
```cpp  
void robotExceptionCb(ELITE::RobotExceptionSharedPtr ex) {  
    if (ex->getType() == ELITE::RobotException::Type::RUNTIME) {  
        auto r_ex = std::static_pointer_cast<ELITE::RobotRuntimeException>(ex);  
        ELITE_LOG_INFO("Robot throw exception: %s", r_ex->getMessage().c_str());  
    }  
}  
```  
- This callback handles robot exceptions.  
- Filters for **runtime exceptions** (script execution errors) versus hardware errors (`ROBOT_ERROR`).  
- Converts the exception to `RobotRuntimeException` and logs the message.  

#### 2.3 Main Program Entry  
```cpp  
if (argc < 2) {  
    std::cout << "Must provide robot IP. Example: ./primary_example aaa.bbb.ccc.ddd" << std::endl;  
    return 1;  
}  
```  
- Requires the robot IP as a command-line argument.  

#### 2.4 Create Communication Interface  
```cpp  
auto primary = std::make_unique<ELITE::PrimaryPortInterface>();  
auto kin = std::make_shared<ELITE::KinematicsInfo>();  
```  
- `PrimaryPortInterface`: Manages communication with port 30001.  
- `KinematicsInfo`: Stores DH parameters.  

#### 2.5 Connect and Register Callback  
```cpp  
primary->connect(robot_ip, 30001);  
primary->registerRobotExceptionCallback(robotExceptionCb);  
```  
- Establishes connection and registers the exception handler.  

#### 2.6 Retrieve and Print DH Parameters  
```cpp  
primary->getPackage(kin, 200);  
...  
ELITE_LOG_INFO("%s", dh_param.c_str());  
```  
- Fetches DH parameters (a, d, alpha) with a 200ms timeout.  
- Formats and logs the parameters.  

> **DH parameters** (Denavit–Hartenberg) standardize the spatial relationship between robotic arm joints and links.  

#### 2.7 Send Scripts  
```cpp  
std::string script = "def hello():\n\ttextmsg(\"hello world\")\nend\n";  
primary->sendScript(script);  
```  
- **Valid script**: Defines `hello()` to print "hello world" on the robot.  

```cpp  
script = "def exFunc():\n\t1abcd\nend\n";  
primary->sendScript(script);  
```  
- **Invalid script**: Contains syntax errors (`1abcd`) to trigger an exception.  

#### 2.8 Wait and Disconnect  
```cpp  
std::this_thread::sleep_for(1s);  
primary->disconnect();  
```  
- Waits 1 second to ensure exception delivery before disconnecting.  

### 3. Compile and Run  
Compile with:  
```bash  
g++ primary_example.cpp -o primary_example -lelite-cs-series-sdk  
```  

Run (ensure the robot is powered on and brakes are released):  
```bash  
./primary_example <your_robot_ip>  
```  

Expected output:  
- DH parameters printed to log.  
- Runtime exception logged.  
- "hello world" displayed in the robot's teach pendant log.  

> **Tip**: The `PrimaryPortInterface` functionality is also integrated into the `EliteDriver` class (covered later).  

---  
[>>> Next: How to Parse Primary Port Packets](./How-to-Parser-30001.en.md)  