#include <utils.h>

std::string get_env_str(std::string env)
{
    const char* val = std::getenv(env.c_str());
    return (val == NULL) ? std::string() : std::string(val);
}
