#ifndef I2C_CONN_H
#define I2C_CONN_H

#include <stdio.h>
#include <fcntl.h>
#include <cstdint>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

inline const char* const I2C_DEVICE = "/dev/i2c-1";

#if !defined(UNIT_TEST)
class I2cConn
{
public:
    I2cConn(uint8_t address);

    ~I2cConn();

    bool open();

    bool read_byte_data(uint8_t reg, uint8_t* data);
    bool read_block_data(uint8_t reg, uint8_t size, uint8_t* buf);

    bool write_byte_data(uint8_t reg, uint8_t data);

    bool write_block_data(uint8_t reg, uint8_t size, uint8_t* buf);
private:
    int8_t _fd;
    uint8_t _address;
};
#endif

#endif /* I2C_CONN_H */
