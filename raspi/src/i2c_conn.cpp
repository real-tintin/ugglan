#include <i2c_conn.h>

#if !defined(UNIT_TEST)

I2cConn::I2cConn(std::string device, uint8_t address) :
    _device(device),
    _address(address)
{
}

I2cConn::~I2cConn()
{
    ::close(_fd);
}

bool I2cConn::open()
{
    if (((_fd = ::open(_device.c_str(), O_RDWR)) >= 0) && (ioctl(_fd, I2C_SLAVE, _address) >= 0))
    {
        logger.debug("Successfully opened i2c connection at: " + _device + " (" +
            utils::byte_to_hex_str(_address) + ")");

        return true;
    }
    else
    {
        logger.error("Failed to open i2c connection at: " + _device + " (" +
            utils::byte_to_hex_str(_address) + ")");

        return false;
    }
}

bool I2cConn::read_byte_data(uint8_t reg, uint8_t* data)
{
    int32_t status = i2c_smbus_read_byte_data(_fd, reg);
    *data = (uint8_t) status;
    return status >= 0;
}

bool I2cConn::read_block_data(uint8_t reg, uint8_t size, uint8_t* buf)
{
    int32_t status = i2c_smbus_read_i2c_block_data(_fd, reg, size, buf);
    return status >= 0;
}

bool I2cConn::write_byte_data(uint8_t reg, uint8_t data)
{
    int32_t status = i2c_smbus_write_byte_data(_fd, reg, data);
    return status >= 0;
}

bool I2cConn::write_block_data(uint8_t reg, uint8_t size, uint8_t* buf)
{
    int32_t status = i2c_smbus_write_i2c_block_data(_fd, reg, size, buf);
    return status >= 0;
}

#endif
