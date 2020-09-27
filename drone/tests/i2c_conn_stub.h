#ifndef I2C_CONN_STUB_H
#define I2C_CONN_STUB_H

#include <i2c_conn.h>

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
    I2cConn(uint8_t address = 0);

    void set_read_byte_map(I2cReadByteMap read_byte_map);
    void set_read_block_map(I2cReadBlockMap read_block_map);
    void set_write_map(I2cWriteMap write_map);

    bool open();

    bool read_byte_data(uint8_t reg, uint8_t* data);
    bool read_block_data(uint8_t reg, uint8_t size, uint8_t* buf);

    bool write_byte_data(uint8_t reg, uint8_t data);
    bool write_block_data(uint8_t reg, uint8_t size, uint8_t* buf);
private:
    uint8_t _address;
    
    I2cReadByteMap _read_byte_map;
    I2cReadBlockMap _read_block_map;

    I2cWriteMap _write_map;
};

#endif /* I2C_CONN_STUB_H */
