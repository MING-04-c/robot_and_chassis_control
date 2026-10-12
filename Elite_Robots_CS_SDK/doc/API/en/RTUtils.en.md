# Real-Time Utilities

Some interfaces to improve real-time performance

## Header File
```cpp
#include <Elite/RtUtils.hpp>
```

## Interfaces

### Set Real-Time Thread
```cpp
bool setThreadFiFoScheduling(std::thread::native_handle_type& thread, const int priority);
```

- ***Function***

    Sets the thread scheduling policy to FIFO and sets the priority

- ***Parameters***

  - `thread`: Thread handle
  - `priority`: Priority level

- ***Return Value***
    - `true`: Success
    - `false`: Failure

### Get Maximum FIFO Thread Priority
```cpp
int getThreadFiFoMaxPriority();
```

- ***Function***

    Retrieves the maximum allowed priority for FIFO scheduled threads

- ***Return Value***: Maximum priority value

### Bind Thread to CPU Core
```cpp
bool bindThreadToCpus(std::thread::native_handle_type& thread, const int cpu)
``` 

- ***Function***

    Binds the thread to a specific CPU core

- ***Parameters***

  - `thread`: Thread handle
  - `cpu`: CPU core identifier

- ***Return Value***
    - `true`: Binding successful
    - `false`: Binding failed