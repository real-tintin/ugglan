#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <cstdint>
#include <cstdlib>
#include <sstream>
#include <type_traits>
#include "logger.h"

namespace utils
{
    template <typename T>
    T get_env(std::string env, T default_val)
    {
        const char* val = std::getenv(env.c_str());

        if (val == NULL)
        {
            return default_val;
        }

        if constexpr (std::is_same<T, std::string>::value)
        {
            return std::string(val);
        }

        if constexpr (std::is_same<T, double>::value)
        {
            return std::stod(val);
        }

        throw std::runtime_error("Unsupported data type");
    }
}

#endif /* UTILS_H */
