#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <stdexcept>
#include <mutex>
#include <cstdlib>

enum class LogLevel {
    debug,
    info,
    warn,
    error,
    off
};

inline const std::string LOGGER_LEVEL_STR_DEBUG = "DEBUG";
inline const std::string LOGGER_LEVEL_STR_INFO  = "INFO ";
inline const std::string LOGGER_LEVEL_STR_WARN  = "WARN ";
inline const std::string LOGGER_LEVEL_STR_ERROR = "ERROR";

class Logger
{
public:
    void debug(std::string msg);
    void info(std::string msg);
    void warn(std::string msg);
    void error(std::string msg);

    void set_level(LogLevel level);
    LogLevel get_level();
private:
    LogLevel _level = LogLevel::info;
    std::mutex _mutex;

    void _disp_msg(std::string msg, LogLevel level);
};

extern Logger logger;

#endif /* LOGGER_H */
