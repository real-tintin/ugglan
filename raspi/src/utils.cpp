#include <utils.h>

namespace utils
{
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
}
