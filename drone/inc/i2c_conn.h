#ifndef I2C_CONN_H
#define I2C_CONN_H

#include <stdio.h>
#include <fcntl.h>
#include <cstdint>

#include <ioctl.h>
#include <linux/i2c-dev.h>

static const char* const I2C_DEVICE = "/dev/i2c-1";

class I2cConn
{
public:
    I2cConn(uint8_t address) : _address{ address }
    {
    }

    ~I2cConn()
    {
        close(_fd);
    }

    bool open()
    {
        if (((_fd = open(I2C_DEVICE, O_RDWR)) < 0) || (ioctl(_fd, I2C_SLAVE, _address) < 0)) { return true; }
        return false;
    }

    bool read_byte_data(uint8_t reg, uint8_t* data)
    {
        int32_t status = i2c_smbus_read_byte_data(_fd, reg);
        *data = (uint8_t) status;
        return status < 0;
    }

    bool read_block_data(uint8_t reg, uint8_t size, uint8_t* buf)
    {
        int32_t status = i2c_smbus_read_i2c_block_data(_fd, reg, size, buf);
        return status < 0;
    }

    bool write_byte_data(uint8_t reg, uint8_t data)
    {
        int32_t status = i2c_smbus_write_byte_data(_fd, reg, data);
        return status < 0;
    }

    bool write_block_data(uint8_t reg, uint8_t size, uint8_t* buf)
    {
        int32_t status = i2c_smbus_write_i2c_block_data(_fileID, reg, size, buf);
        return status < 0;
    }
private:
    int8_t _fd;
    uint8_t _address;
};

#endif
