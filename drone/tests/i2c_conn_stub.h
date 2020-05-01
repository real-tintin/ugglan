#ifndef I2C_CONN_STUB_H
#define I2C_CONN_STUB_H

#include <stdio.h>
#include <fcntl.h>
#include <cstdint>
#include <cstring>
#include <map>
#include <vector>
#include <algorithm>

typedef std::map<uint8_t, uint8_t> I2cReadByteMap;
typedef std::map<uint8_t, uint8_t*> I2cReadBlockMap;

typedef std::vector<uint8_t> I2cWriteMap;

class I2cConn
{
public:
    I2cConn() = default;

    void set_read_byte_map(I2cReadByteMap read_byte_map) { _read_byte_map = read_byte_map; }

    void set_read_block_map(I2cReadBlockMap read_block_map) { _read_block_map = read_block_map; }

    void set_write_map(I2cWriteMap write_map) { _write_map = write_map; }

    bool open() { return true; }

    bool read_byte_data(uint8_t reg, uint8_t* data)
    {
        auto pair = _read_byte_map.find(reg);

        if (pair != _read_byte_map.end())
        {
            *data = pair->second;
            return true;
        }
        return false;
    }

    bool read_block_data(uint8_t reg, uint8_t size, uint8_t* buf)
    {
        auto pair = _read_block_map.find(reg);

        if (pair != _read_block_map.end())
        {
            std::memcpy(buf, pair->second, size);
            return true;
        }
        return false;
    }

    bool write_byte_data(uint8_t reg, uint8_t data)
    {
        if (std::count(_write_map.begin(),_write_map.end(), reg)) { return true; }
        return false;
    }

    bool write_block_data(uint8_t reg, uint8_t size, uint8_t* buf)
    {
        if (std::count(_write_map.begin(),_write_map.end(), reg)) { return true; }
        return false;
    }
private:
    I2cReadByteMap _read_byte_map;
    I2cReadBlockMap _read_block_map;

    I2cWriteMap _write_map;
};

#endif
