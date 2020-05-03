#include <i2c_conn_stub.h>

I2cConn::I2cConn()
{
}

void I2cConn::set_read_byte_map(I2cReadByteMap read_byte_map)
{
    _read_byte_map = read_byte_map;
}

void I2cConn::set_read_block_map(I2cReadBlockMap read_block_map)
{
    _read_block_map = read_block_map;
}

void I2cConn::set_write_map(I2cWriteMap write_map)
{
    _write_map = write_map;
}

bool I2cConn::open()
{
    return true;
}

bool I2cConn::read_byte_data(uint8_t reg, uint8_t* data)
{
    auto it = _read_byte_map.find(reg);

    if (it != _read_byte_map.end())
    {
        *data = it->second;
        return true;
    }
    return false;
}

bool I2cConn::read_block_data(uint8_t reg, uint8_t size, uint8_t* buf)
{
    auto it = _read_block_map.find(reg);

    if (it != _read_block_map.end())
    {
        std::memcpy(buf, it->second, size);
        return true;
    }
    return false;
}

bool I2cConn::write_byte_data(uint8_t reg, uint8_t data)
{
    if (std::count(_write_map.begin(),_write_map.end(), reg))
    {
        return true;
    }
    return false;
}

bool I2cConn::write_block_data(uint8_t reg, uint8_t size, uint8_t* buf)
{
    if (std::count(_write_map.begin(),_write_map.end(), reg))
    {
        return true;
    }
    return false;
}
