#include <i2c_conn_stub.hpp>

void I2cConnStub::set_read_byte_map(I2cReadByteMap read_byte_map)
{
    _read_byte_map = std::move(read_byte_map);
}

void I2cConnStub::set_read_block_map(I2cReadBlockMap read_block_map)
{
    _read_block_map = std::move(read_block_map);
}

void I2cConnStub::set_write_map(I2cWriteMap write_map)
{
    _write_map = std::move(write_map);
}

bool I2cConnStub::open()
{
    return true;
}

bool I2cConnStub::close()
{
    return true;
}

bool I2cConnStub::read_byte_data(uint8_t reg, uint8_t *data)
{
    auto it = _read_byte_map.find(reg);
    _n_calls_read_byte_data++;

    if (it != _read_byte_map.end())
    {
        *data = it->second;
        return true;
    }
    return false;
}

bool I2cConnStub::read_block_data(uint8_t reg, uint8_t size, uint8_t *buf)
{
    auto it = _read_block_map.find(reg);
    _n_calls_read_block_data++;

    if (it != _read_block_map.end())
    {
        std::memcpy(buf, it->second, size);
        return true;
    }
    return false;
}

bool I2cConnStub::write_byte_data(uint8_t reg, uint8_t data)
{
    _n_calls_write_byte_data++;

    return std::count(_write_map.begin(), _write_map.end(), reg) != 0;
}

bool I2cConnStub::write_block_data(uint8_t reg, uint8_t size, uint8_t *buf)
{
    _n_calls_write_block_data++;

    return std::count(_write_map.begin(), _write_map.end(), reg) != 0;
}
