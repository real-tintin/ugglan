#include <common_utils.h>

namespace common_utils
{
std::string get_env(std::string name)
{
    const char* value = std::getenv(name.c_str());

    if (value == NULL)
    {
        throw std::runtime_error("Environmental variable doesn't exist: " + name);
    }

    return std::string(value);
}

std::string byte_to_hex_str(uint8_t byte)
{
    std::stringstream buf;

    buf << "0x" << ((byte < 16) ? "0" : "") <<
    std::uppercase << std::hex << int(byte);

    return buf.str();
}

std::string byte_to_bit_str(uint8_t byte)
{
    std::stringstream buf;

    buf << "0b" << std::bitset<8>{byte};

    return buf.str();
}

std::string unpack_base64_gzip(const std::string packed)
{
    std::string decoded = base64_decode(packed);
    std::string decompressed = gzip::decompress(decoded.c_str(), decoded.size());

    return decompressed;
}

std::string pack_gzip_base64(const std::string unpacked)
{
    std::string compressed = gzip::compress(unpacked.c_str(), unpacked.size());
    std::string encoded = base64_encode(compressed.c_str(), compressed.size());

    return encoded;
}
} /* common_utils */
