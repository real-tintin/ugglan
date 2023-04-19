#ifndef COMMON_UTILS_HPP
#define COMMON_UTILS_HPP

#include <bitset>
#include <cstdint>
#include <cstdlib>
#include <ios>
#include <sstream>
#include <string>
#include <type_traits>

#include <base64/base64.hpp>
#include <gzip/compress.hpp>
#include <gzip/decompress.hpp>

#include <logger.hpp>

namespace common_utils {
std::string get_env(std::string);

template <typename T> void read_and_cast_env(T &dst, std::string name)
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

std::string unpack_base64_gzip(const std::string packed);
std::string pack_gzip_base64(const std::string unpacked);
} // namespace common_utils

#endif /* COMMON_UTILS_HPP */
