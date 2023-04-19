#include <i2c_conn.hpp>

I2cConn::I2cConn(std::string device, uint8_t address) : _device(device), _address(address)
{
}

bool I2cConn::open()
{
    if (((_fd = ::open(_device.c_str(), O_RDWR)) != -1) && (ioctl(_fd, I2C_SLAVE, _address) != -1))
    {
        logger.debug("Successfully opened i2c connection at: " + _device + " (" +
                     common_utils::byte_to_hex_str(_address) + ")");

        return true;
    }
    else
    {
        logger.error("Failed to open i2c connection at: " + _device + " (" + common_utils::byte_to_hex_str(_address) +
                     ")");

        return false;
    }
}

bool I2cConn::close()
{
    if (::close(_fd) != -1)
    {
        logger.debug("Successfully closed i2c connection at: " + _device);
        return true;
    }
    else
    {
        logger.error("Failed to close i2c connection at: " + _device);
        return false;
    }
}

bool I2cConn::read_byte_data(uint8_t reg, uint8_t *data)
{
    int32_t status = i2c_smbus_read_byte_data(_fd, reg);
    *data = static_cast<uint8_t>(status);
    return status >= 0;
}

bool I2cConn::read_block_data(uint8_t reg, uint8_t size, uint8_t *buf)
{
    int32_t status = i2c_smbus_read_i2c_block_data(_fd, reg, size, buf);
    return status >= 0;
}

bool I2cConn::write_byte_data(uint8_t reg, uint8_t data)
{
    int32_t status = i2c_smbus_write_byte_data(_fd, reg, data);
    return status >= 0;
}

bool I2cConn::write_block_data(uint8_t reg, uint8_t size, uint8_t *buf)
{
    int32_t status = i2c_smbus_write_i2c_block_data(_fd, reg, size, buf);
    return status >= 0;
}
