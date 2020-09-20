#include <main.h>

std::filesystem::path DATA_LOG_ROOT = get_env_str(ENV_DATA_LOG_ROOT);

std::string get_env_str(std::string env)
{
    const char* val = std::getenv(env.c_str());
    return (val == NULL) ? std::string() : std::string(val);
}

#if !defined(UNIT_TEST)
int main()
{
    // TODO: Glue!
    return 0;
}
#endif
