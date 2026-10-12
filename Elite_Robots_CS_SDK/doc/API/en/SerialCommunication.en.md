# RS485 Serial Communication

## Summary

The robot's end effector and control cabinet are equipped with RS485 communication interfaces, and the SDK provides corresponding functionalities.

The essence of reading/writing robot serial ports through the SDK is that the robot forwards data from the RS485 interface to a specified TCP port, and the SDK directly reads/writes to that TCP port.

# SerialConfig Class

## Description

Contains configuration variables for serial port baud rate, parity, etc., along with enumeration definitions.

## Header File
```cpp
#include <Elite/SerialCommunication.hpp>
```

## Parameters

- `baud_rate`
    - Type: `SerialConfig::BaudRate`
    - Description: Serial port baud rate. Default value is `BR_115200` (115200 baud).

- `parity`
    - Type: `SerialConfig::Parity`
    - Description: Serial port parity check. Default value is `NONE` (no parity).

- `stop_bits`
    - Type: `SerialConfig::StopBits`
    - Description: Serial port stop bits. Default value is `ONE` (1 stop bit).

# SerialCommunication Class

## Description

TCP client for serial port forwarding.

## Header File
```cpp
#include <Elite/SerialCommunication.hpp>
```

## Interface

---

### ***Connect***
```cpp
bool connect(int timeout_ms)
```
- ***Description***
    
    Connect to the server forwarding the robot's serial port.

- ***Parameters***
    - `timeout_ms`: Timeout in milliseconds
    
- ***Return Value***: Returns true if connection is successful.

---

### ***Disconnect***
```cpp
void disconnect()
```

- ***Description***
    
    Disconnect from the server.

---

### ***Write Data to Serial Port***
```cpp
int write(const uint8_t* data, size_t size)
```

- ***Description***
    
    Write data to the serial port.

- ***Parameters***
    - `data`: Data buffer
    
    - `size`: Data size
    
- ***Return Value***: Number of bytes written.

---

### ***Read Data from Serial Port***
```cpp
int read(uint8_t* data, size_t size, int timeout_ms)
```

- ***Description***
    
    Read data from the serial port.

- ***Parameters***
    - `data`: Data buffer
    
    - `size`: Data size
    
    - `timeout_ms`: Timeout in milliseconds. Values ≤ 0 indicate infinite waiting.
    
- ***Return Value***: Number of bytes read.

---

### ***Check Server Connection***
```cpp
bool isConnected()
```

- ***Description***
    
    Check if connected to the server.

    
- ***Return Value***: Returns true if connected to the server.

---

### ***Get socat PID***
```cpp
int getSocatPid() const
```

- ***Description***
    Get the process ID (PID) of the port-mapping socat process on the robot control cabinet.

    
- ***Return Value***: The process ID (PID). If it is less than 0, it is an invalid PID.

--- 