#ifndef I2C_CONN_HPP
#define I2C_CONN_HPP

#include <cstdint>
#include <fcntl.h>
#include <stdio.h>
#include <string>
extern "C"
{
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
}

#include <common_utils.hpp>
#include <logger.hpp>

class I2cConn
{
public:
    I2cConn(std::string device, uint8_t address);

    virtual bool open();
    virtual bool close();

    virtual bool read_byte_data(uint8_t reg, uint8_t *data);
    virtual bool read_block_data(uint8_t reg, uint8_t size, uint8_t *buf);

    virtual bool write_byte_data(uint8_t reg, uint8_t data);
    virtual bool write_block_data(uint8_t reg, uint8_t size, uint8_t *buf);

private:
    int _fd;
    std::string _device;
    uint8_t _address;
};

#endif /* I2C_CONN_HPP */
