#include <filesystem>
#include <utils.h>

std::filesystem::path DATA_LOG_ROOT = get_env_str("DATA_LOG_ROOT");

// TODO: Set logger level from env.

#if !defined(UNIT_TEST)
int main()
{
    // TODO: Glue!
    return 0;
}
#endif
