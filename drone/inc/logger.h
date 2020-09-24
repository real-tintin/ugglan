#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <stdexcept>

enum class LogLevel {
    debug,
    info,
    warn,
    error,
    off
};

class Logger
{
public:
    void debug(std::string msg) { if (_level <= LogLevel::debug) { _disp_msg(msg, LogLevel::debug); } }
    void info(std::string msg) { if (_level <= LogLevel::info) { _disp_msg(msg, LogLevel::info); } }
    void warn(std::string msg) { if (_level <= LogLevel::warn) { _disp_msg(msg, LogLevel::warn); } }
    void error(std::string msg) { if (_level <= LogLevel::error) { _disp_msg(msg, LogLevel::error); } }

    void set_level(LogLevel level) { _level = level; };
private:
    LogLevel _level = LogLevel::info;

    void _disp_msg(std::string msg, LogLevel level)
    {
        std::time_t t_now = std::time(nullptr);
        std::string level_str;

        switch(level)
        {
            case LogLevel::debug:
                level_str = "DEBUG";
                break;
            case LogLevel::info:
                level_str = "INFO ";
                break;
            case LogLevel::warn:
                level_str = "WARN ";
                break;
            case LogLevel::error:
                level_str = "ERROR";
                break;
            default:
                throw std::runtime_error("Invalid logger level");
                break;
        }

        std::cout << std::put_time(std::localtime(&t_now), "[%F %T]") << " : "
                  << level_str << " : " << msg << std::endl;
    }
};

#endif /* LOGGER_H */
