#include <logger.h>

Logger logger;

void Logger::debug(std::string msg)
{
    const std::lock_guard<std::mutex> lock(_mutex);

    if (_level <= LogLevel::debug)
    {
        _disp_msg(msg, LogLevel::debug);
    }
}

void Logger::info(std::string msg)
{
    const std::lock_guard<std::mutex> lock(_mutex);

    if (_level <= LogLevel::info)
    {
        _disp_msg(msg, LogLevel::info);
    }
}

void Logger::warn(std::string msg)
{
    const std::lock_guard<std::mutex> lock(_mutex);

    if (_level <= LogLevel::warn)
    {
        _disp_msg(msg, LogLevel::warn);
    }
}

void Logger::error(std::string msg)
{
    const std::lock_guard<std::mutex> lock(_mutex);

    if (_level <= LogLevel::error)
    {
        _disp_msg(msg, LogLevel::error);
    }
}

void Logger::set_level(LogLevel level)
{
    const std::lock_guard<std::mutex> lock(_mutex);

    _level = level;
}

LogLevel Logger::get_level()
{
    const std::lock_guard<std::mutex> lock(_mutex);

    return _level;
}

void Logger::_disp_msg(std::string msg, LogLevel level)
{
    std::time_t t_now = std::time(nullptr);
    std::string level_str;

    switch(level)
    {
        case LogLevel::debug:
            level_str = LOGGER_LEVEL_STR_DEBUG;
            break;
        case LogLevel::info:
            level_str = LOGGER_LEVEL_STR_INFO;
            break;
        case LogLevel::warn:
            level_str = LOGGER_LEVEL_STR_WARN;
            break;
        case LogLevel::error:
            level_str = LOGGER_LEVEL_STR_ERROR;
            break;
        default:
            throw std::runtime_error("Invalid logger level");
            break;
    }

    std::cout << std::put_time(std::localtime(&t_now), "[%F %T]") << " : "
                << level_str << " : " << msg << std::endl;
}
