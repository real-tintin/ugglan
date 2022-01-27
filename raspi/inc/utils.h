#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <cstdint>
#include <cstdlib>
#include <sstream>
#include <type_traits>
#include <ios>
#include <bitset>
#include <logger.h>

namespace utils
{
    std::string get_env(std::string);

    template <typename T>
    void read_and_cast_env(T &dst, std::string name)
    {
        std::string val_as_str = get_env(name);

        if constexpr (std::is_same<T, std::string>::value)
        {
            dst = val_as_str;
        }
        else if constexpr (std::is_same<T, double>::value)
        {
            dst = std::stod(val_as_str);
        }
        else if constexpr (std::is_same<T, uint8_t>::value)
        {
            dst = static_cast<uint8_t>(std::stoul(val_as_str));
        }
        else if constexpr (std::is_same<T, uint32_t>::value)
        {
            dst = static_cast<uint32_t>(std::stoul(val_as_str));
        }
        else
        {
            throw std::runtime_error("Unsupported data type");
        }
    }

    std::string byte_to_hex_str(uint8_t byte);
    std::string byte_to_bit_str(uint8_t byte);
}

#endif /* UTILS_H */
