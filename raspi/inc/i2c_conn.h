#ifndef I2C_CONN_H
#define I2C_CONN_H

#include <string>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <logger.h>
#include <utils.h>

#if !defined(UNIT_TEST)
class I2cConn
{
public:
    I2cConn(std::string device, uint8_t address);

    ~I2cConn();

    bool open();

    bool read_byte_data(uint8_t reg, uint8_t* data);
    bool read_block_data(uint8_t reg, uint8_t size, uint8_t* buf);

    bool write_byte_data(uint8_t reg, uint8_t data);

    bool write_block_data(uint8_t reg, uint8_t size, uint8_t* buf);
private:
    int8_t _fd;
    std::string _device;
    uint8_t _address;
};
#endif

#endif /* I2C_CONN_H */
