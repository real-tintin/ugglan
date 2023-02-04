#ifndef I2C_CONN_H
#define I2C_CONN_H

#include <string>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include <sys/ioctl.h>
extern "C"
{
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}
#include <logger.h>
#include <common_utils.h>

class I2cConn
{
public:
    I2cConn(std::string device, uint8_t address);

    virtual bool open();
    virtual bool close();

    virtual bool read_byte_data(uint8_t reg, uint8_t* data);
    virtual bool read_block_data(uint8_t reg, uint8_t size, uint8_t* buf);

    virtual bool write_byte_data(uint8_t reg, uint8_t data);
    virtual bool write_block_data(uint8_t reg, uint8_t size, uint8_t* buf);
private:
    int8_t _fd;
    std::string _device;
    uint8_t _address;
};

#endif /* I2C_CONN_H */
