[Home](./UserGuide.en.md)  

# Power On  

## Objective  
Call the SDK interfaces to power on the robot and release the brakes.  

## Background  
The Dashboard Shell provides a way to interact with the robot through TCP port 29999, enabling operations like power cycling, brake release, and task loading/querying. The SDK includes the `DashboardClient` class which implements most dashboard interfaces.  

This section demonstrates using the robot's Dashboard Shell functionality to power on the robot and release its brakes.  

## Task  

### 1. Write a Simple Dashboard Client Program  

Create a code file:  
```bash  
touch dashboard_example.cpp  
```  

Copy the following code into `dashboard_example.cpp` using your preferred text editor:  

```cpp  
#include <iostream>  
#include <memory>  
#include <string>  
#include <Elite/DashboardClient.hpp>  

using namespace ELITE;  

int main(int argc, char* argv[]) {  
    if (argc < 2) {  
        std::cout << "Must provide robot IP. Example: ./dashboard_example aaa.bbb.ccc.ddd" << std::endl;  
        return 1;  
    }  
    std::string robot_ip = argv[1];  

    std::unique_ptr<DashboardClient> my_dashboard;  
    my_dashboard.reset(new DashboardClient());  

    if (!my_dashboard->connect(robot_ip)) {  
        std::cout << "Could not connect to robot" << std::endl;  
        return 1;  
    } else {  
        std::cout << "Connected to robot" << std::endl;  
    }  

    // Power on  
    if (!my_dashboard->powerOn()) {  
        std::cout << "Failed to send Power On command" << std::endl;  
        return 1;  
    } else {  
        std::cout << "Robot powered on" << std::endl;  
    }  

    // Brake release  
    if (!my_dashboard->brakeRelease()) {  
        std::cout << "Failed to send Brake Release command" << std::endl;  
        return 1;  
    } else {  
        std::cout << "Brakes released" << std::endl;  
    }  
    
    my_dashboard->disconnect();  

    return 0;  
}  
```  

### 2. Code Explanation  

This code uses the Elite Robot SDK's `DashboardClient` control interface to:  
1. Connect to the robot  
2. Power it on  
3. Release the brakes  
4. Disconnect  

#### 2.1 Included Headers  
```cpp  
#include <iostream>     // Standard I/O  
#include <memory>       // Smart pointers (std::unique_ptr)  
#include <string>       // String handling  
#include <Elite/DashboardClient.hpp>  // SDK Dashboard control class  
```  

#### 2.2 Namespace  
```cpp  
using namespace ELITE;  
```  
Simplifies access to `DashboardClient` within the `ELITE` namespace.  

#### 2.3 Main Function  
```cpp  
int main(int argc, char* argv[])  
```  
Program entry point that accepts the robot IP as a command-line argument.  

#### 2.4 Argument Validation  
```cpp  
if (argc < 2) {  
    std::cout << "Must provide robot IP. Example: ./dashboard_example aaa.bbb.ccc.ddd" << std::endl;  
    return 1;  
}  
```  
Requires robot IP input, otherwise exits with instructions.  

#### 2.5 Create and Connect Dashboard Client  
```cpp  
std::unique_ptr<DashboardClient> my_dashboard;  
my_dashboard.reset(new DashboardClient());  

if (!my_dashboard->connect(robot_ip)) {  
    std::cout << "Connection failed" << std::endl;  
    return 1;  
}  
```  
Manages the `DashboardClient` instance with `std::unique_ptr` for automatic cleanup.  

#### 2.6 Power On  
```cpp  
if (!my_dashboard->powerOn()) {  
    std::cout << "Power on failed" << std::endl;  
    return 1;  
}  
```  
Sends power-on command, exits on failure.  

#### 2.7 Brake Release  
```cpp  
if (!my_dashboard->brakeRelease()) {  
    std::cout << "Brake release failed" << std::endl;  
    return 1;  
}  
```  
Releases the mechanical brakes that lock the robot's joints.  

#### 2.8 Disconnect  
```cpp  
my_dashboard->disconnect();  
```  
Closes the connection when done.  

### 3. Compile and Run  
Compile with:  
```bash  
g++ dashboard_example.cpp -o dashboard_example -lelite-cs-series-sdk  
```  

Execute (replace with your robot's IP):  
```bash  
./dashboard_example 192.168.1.100  
```  
Successful execution will power on the robot and release its brakes.  

---  
[>>> Next: Get Robot State](./Get-Robot-State.en.md)