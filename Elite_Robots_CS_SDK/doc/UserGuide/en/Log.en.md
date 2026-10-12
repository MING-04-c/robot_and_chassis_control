[Home](./UserGuide.en.md)
# Log

## Background

The SDK has a logging function and supports users to customize the logging method and level.

## Tasks

### Change logging level

Decide the log level that needs to be printed in the program. By default, only logs of INFO level or higher will be printed. Calling the `setLogLevel()` function can set the log level.

```cpp
#include <Elite/Log.hpp>

int main() {
    ELITE::setLogLevel(ELITE::LogLevel::DEBUG);

    ELITE_LOG_DEBUG("Log level set debug");

    return 0;
}
```

### Create new log handle

The SDK provides a default log recording method that prints to the terminal. If needed, you can customize the log output method. The following code may give you inspiration:

```cpp
#include <Elite/Log.hpp>
#include <iostream>
#include <memory>

class NewLogHandler : public ELITE::LogHandler
{
private:
    
public:
    NewLogHandler() = default;
    ~NewLogHandler() = default;

    void log(const char* file, int line, ELITE::LogLevel level, const char* log) {
        switch (level) {
        case ELITE::LogLevel::DEBUG:
            std::cout << "[DEBUG] " << file << ":" << line << ": " << log << std::endl;
            break;
        case ELITE::LogLevel::INFO:
            std::cout << "[INFO] " << file << ":" << line << ": " << log << std::endl;
            break;
        case ELITE::LogLevel::WARN:
            std::cout << "[WARN] " << file << ":" << line << ": " << log << std::endl;
            break;
        case ELITE::LogLevel::ERROR:
            std::cout << "[ERROR] " << file << ":" << line << ": " << log << std::endl;
            break;
        case ELITE::LogLevel::FATAL:
            std::cout << "[FATAL] " << file << ":" << line << ": " << log << std::endl;
            break;
        case ELITE::LogLevel::NONE:
            std::cout << "[NONE] " << file << ":" << line << ": " << log << std::endl;
            break;
        default:
            break;
        }
    }

};

int main() {
    std::unique_ptr<NewLogHandler> log_handle(new NewLogHandler);
    ELITE::registerLogHandler(std::move(log_handle));

    ELITE_LOG_INFO("Register new log handler");

    return 0;
}
```

[>>>Next Chapter: RS485 Communication](./Serial-Communication.en.md)