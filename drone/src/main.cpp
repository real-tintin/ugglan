#include <filesystem>
#include <string>
#include <utils.h>

std::filesystem::path DATA_LOG_ROOT = get_env_str("DATA_LOG_ROOT");
std::string LOGGER_LEVEL = get_env_str("LOGGER_LEVEL");

void set_logger_level()
{
    if      (LOGGER_LEVEL == "DEBUG") { logger.set_level(LogLevel::debug); }
    else if (LOGGER_LEVEL == "INFO")  { logger.set_level(LogLevel::info); }
    else if (LOGGER_LEVEL == "WARN")  { logger.set_level(LogLevel::warn); }
    else if (LOGGER_LEVEL == "ERROR") { logger.set_level(LogLevel::error); }
    else if (LOGGER_LEVEL == "OFF")   { logger.set_level(LogLevel::off); }
    else                              { /* Keep default. */ }
}

#if !defined(UNIT_TEST)
int main()
{
    set_logger_level();

    // TODO: Glue!
    return 0;
}
#endif
