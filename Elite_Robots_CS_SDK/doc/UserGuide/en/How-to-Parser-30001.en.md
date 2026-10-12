[Home](./UserGuide.en.md)

# How to Parse Primary Port Messages

## Objective
Write a program to read the robot's TCP coordinates.

## Background
Primary port messages consist of multiple sub-messages, but the SDK does not provide parsing for all sub-messages. Therefore, users need to parse the required data themselves.

## Tasks

### 1. Write a Class for Parsing TCP Coordinates

Create a header file:
```bash
touch RobotTCPPackage.hpp
```

Copy the following code and paste it into `RobotTCPPackage.hpp` using your preferred text editor:

```cpp
#pragma once

#include <Elite/PrimaryPackage.hpp>
#include <algorithm>
#include <type_traits>
#include <cstring>
#include <iostream>

class RobotTCPPackage : public ELITE::PrimaryPackage {
private:
    constexpr static int ROBOT_TCP_PKG_TYPE = 4;
public:
    RobotTCPPackage() : PrimaryPackage(ROBOT_TCP_PKG_TYPE) { }

    ~RobotTCPPackage() = default;

    uint32_t unpackUInt32(const std::vector<uint8_t>::const_iterator& iter) {
        uint8_t bytes[sizeof(uint32_t)];
        std::copy(iter, iter + sizeof(uint32_t), bytes);

        std::reverse(std::begin(bytes), std::end(bytes));
        
        uint32_t result;
        std::memcpy(&result, bytes, sizeof(uint32_t));
        return result;
    }

    double unpackDouble(const std::vector<uint8_t>::const_iterator& iter) {
        uint8_t bytes[sizeof(double)];
        std::copy(iter, iter + sizeof(double), bytes);

        std::reverse(std::begin(bytes), std::end(bytes));
        
        double result;
        std::memcpy(&result, bytes, sizeof(double));
        return result;
    }

    virtual void parser(int len, const std::vector<uint8_t>::const_iterator& iter) override {
        int offset = 0;
        std::cout << "Package len: " << unpackUInt32(iter + offset) << std::endl;
        offset += sizeof(uint32_t);

        std::cout << "Package type: " << (int)*(iter + offset) << std::endl;
        offset += sizeof(uint8_t);

        std::cout << "TCP Position X:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Position Y:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Position Z:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Rotation X:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Rotation Y:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Rotation Z:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Offset Position X:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Offset Position Y:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Offset Position Z:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Offset Rotation X:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Offset Rotation Y:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Offset Rotation Z:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

    }
};
```

### 2. Code Analysis

RobotTCPPackage inherits from `ELITE::PrimaryPackage` and is specifically used to parse robot TCP (Tool Center Point) data packets.
TCP data includes the position (X, Y, Z) and orientation (rotation angles) of the robot arm's end effector in space, as well as position/orientation offsets.
This class implements custom binary parsing logic to decode the received byte stream into actual values and output them.

Before starting to parse the code, you need to open the "CS_User_Manual_Robot_State_Messages.xlsx" spreadsheet, find the sheet corresponding to your robot version, and locate the "Cartesian Data Sub-message".

---

#### 2.1 Header Files
```cpp
#include <Elite/PrimaryPackage.hpp>
#include <algorithm>
#include <type_traits>
#include <cstring>
#include <iostream>
```

`Elite/PrimaryPackage.hpp`: This header file must be included because we are inheriting from `ELITE::PrimaryPackage`.

---

#### 2.2 Construction and Destruction

```cpp
RobotTCPPackage() : PrimaryPackage(ROBOT_TCP_PKG_TYPE) { }
~RobotTCPPackage() = default;
```
* When constructing, the parent class constructor is called to bind the package type. From the "Cartesian Data Sub-message" section in the aforementioned "CS_User_Manual_Robot_State_Messages.xlsx" spreadsheet, we know that the type value of this sub-message is `4`. Therefore, `ROBOT_TCP_PKG_TYPE` in the code is a constant with a value of 4.
* The destructor uses default and has no special cleaning logic.

---

#### 2.3 Data Unpacking Utility Functions
* Unpacking 32-bit unsigned integers
    ```cpp
    uint32_t unpackUInt32(const std::vector<uint8_t>::const_iterator& iter)
    ```
    1. Copy 4 bytes from the `iter` position to the local array `bytes`.
    2. Reverse the byte order (since network byte order is big-endian).
    3. Use `memcpy` to copy the bytes into a `uint32_t` variable and return it.

* Unpacking double-precision floating-point numbers
    ```cpp
    double unpackDouble(const std::vector<uint8_t>::const_iterator& iter)
    ```
    * The logic is similar to `unpackUInt32`, but it processes 8 bytes and converts them to the `double` type.

---

#### 2.4 Parsing Function (Core)

```cpp
virtual void parser(int len, const std::vector<uint8_t>::const_iterator& iter) override
```

* This function overrides the parent class's `parser()` method and is used to parse the received byte stream according to the protocol format.
* Refer to the "Cartesian Data Sub-message" section in the "CS_User_Manual_Robot_State_Messages.xlsx" spreadsheet and parse based on the data format in that section.
* The parameter `iter` is an iterator to the first byte of the sub-message.
* The parsing order strictly follows the order of the fields defined in the data packet.


An `offset` is maintained during parsing to indicate the current parsing position. After parsing each field, `offset` is advanced by the number of bytes of that field.

1. **Package Length**

   ```cpp
   unpackUInt32(iter + offset);
   offset += sizeof(uint32_t);
   ```

2. **Package Type** (single byte)

   ```cpp
   (int)*(iter + offset);
   offset += sizeof(uint8_t);
   ```

3. **TCP End Effector Position** (X, Y, Z, each as an 8-byte double)

   * TCP Position X
   * TCP Position Y
   * TCP Position Z

4. **TCP Orientation** (Rotation X, Y, Z, each as an 8-byte double)

   * TCP Rotation X
   * TCP Rotation Y
   * TCP Rotation Z

5. **TCP Position Offset** (X, Y, Z, each as an 8-byte double)

   * TCP Offset Position X
   * TCP Offset Position Y
   * TCP Offset Position Z

6. **TCP Orientation Offset** (Rotation X, Y, Z, each as an 8-byte double)

   * TCP Offset Rotation X
   * TCP Offset Rotation Y
   * TCP Offset Rotation Z


### 3. Using ***RobotTCPPackage***

In the same directory, create a code file:
```bash
touch primary_pkg_example.cpp
```

Copy the following code and paste it into `primary_pkg_example.cpp` using your preferred text editor:
```cpp
#include <Elite/PrimaryPortInterface.hpp>
#include "RobotTCPPackage.hpp"
#include <memory>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono;

int main(int argc, const char** argv) {
    if (argc < 2) {
        std::cout << "Must provide robot IP. Example: ./primary_pkg_example aaa.bbb.ccc.ddd" << std::endl;
        return 1;
    }
    std::string robot_ip = argv[1];

    auto primary = std::make_unique<ELITE::PrimaryPortInterface>();

    auto robotPackage = std::make_shared<RobotTCPPackage>();

    primary->connect(robot_ip, 30001);

    primary->getPackage(robotPackage, 200);

    primary->disconnect();

    return 0;
}
```

### 4. Code Analysis
This code demonstrates how to obtain **TCP (Tool Center Point) data packets** from the robot's primary port using the `PrimaryPortInterface` of the **ELITE SDK** and parse the data using the custom `RobotTCPPackage` class.

The running process is very concise:

1. Read the robot's IP address from the command line.
2. Create a main communication interface object and a TCP data packet parsing object.
3. Connect to the robot's primary port (30001).
4. Obtain and parse a frame of TCP data.
5. Disconnect and exit.

---

#### 4.1 Header Files

```cpp
#include <Elite/PrimaryPortInterface.hpp>
#include "RobotTCPPackage.hpp"
#include <memory>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
```
* `<Elite/PrimaryPortInterface.hpp>`: The primary port communication interface class, used for connecting, sending, and receiving data packets.
* `"RobotTCPPackage.hpp"`: The custom TCP data packet parsing class, which contains the definition of the `RobotTCPPackage` class mentioned earlier.

---

#### 4.2 Checking Command-Line Arguments

```cpp
if (argc < 2) {
    std::cout << "Must provide robot IP. Example: ./primary_pkg_example aaa.bbb.ccc.ddd" << std::endl;
    return 1;
}
std::string robot_ip = argv[1];
```

* The program must receive the robot's IP through parameters.
* Example running method:

  ```bash
  ./primary_pkg_example 192.168.1.10
  ```

---

#### 4.3 Creating Communication Object and Data Packet Parsing Object

```cpp
auto primary = std::make_unique<ELITE::PrimaryPortInterface>();
auto robotPackage = std::make_shared<RobotTCPPackage>();
```

* `PrimaryPortInterface`: The core interface for communicating with the robot's primary port.
* `RobotTCPPackage`: Custom class inherited from `PrimaryPackage`, responsible for parsing TCP pose data (X, Y, Z position + orientation + offsets).

---

#### 4.4 Connecting to the Robot's Primary Port

```cpp
primary->connect(robot_ip, 30001);
```

* Connect to the **Primary Interface** of the specified IP, with port number **30001**.
* This port is the main channel used by the Elite robot SDK to transmit real-time control and status data.

---

#### 4.5 Obtaining and Parsing Data Packets

```cpp
primary->getPackage(robotPackage, 200);
```

* Read a **PrimaryPackage** data frame from the robot (here, the `RobotTCPPackage` object is passed in).
* **200** indicates the timeout time (in milliseconds), after which it returns failure.
* `PrimaryPortInterface` will call `RobotTCPPackage::parser()` to parse the byte stream and output TCP pose information.

---

#### 4.6 Disconnecting

```cpp
primary->disconnect();
```

* Release the network connection and clean up resources.

### 5. Compilation and Running
Use the gcc compiler to compile `primary_pkg_example.cpp`:
```bash
g++ primary_pkg_example.cpp -o primary_pkg_example -lelite-cs-series-sdk
```

```bash
./primary_pkg_example <your robot ip>
```

After running, it should print the package type, length, as well as the TCP pose and offsets.

---

[>>>Next Chapter: Get the Robot Moving](./Let-Robot-Move.en.md)