[Home](./UserGuide.cn.md)

# 自定义日志

## 目标

使用SDK提供的接口，自定义日志打印。

## 背景说明

SDK拥有日志功能，并且支持用户自定义日志记录方式和等级。

## 任务

### 1. 调整日志等级接口

决定需要在程序中打印的日志等级，默认情况下，只有INFO或更高等级的日志才会被打印。调用`setLogLevel()`函数便可设置日志等级。

```cpp
#include <Elite/Log.hpp>

int main() {
    ELITE::setLogLevel(ELITE::LogLevel::DEBUG);

    ELITE_LOG_DEBUG("Log level set debug");

    return 0;
}
```

### 2. 自定义日志输出

SDK中提供了默认的打印到终端的日志记录方式，如果需要，可以自定义日志的输出方式，下面的代码或许会给你灵感：

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

[>>>下一章：RS485 串口通讯](./Serial-Communication.cn.md)