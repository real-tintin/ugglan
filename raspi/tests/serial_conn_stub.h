#ifndef SERIAL_CONN_STUB_H
#define SERIAL_CONN_STUB_H

#include <stdio.h>
#include <fcntl.h>
#include <cstdint>
#include <cstring>
#include <cassert>
#include <algorithm>
#include <serial_conn.h>

inline const uint32_t MAX_BUF_SIZE = 4096;

class SerialConnStub : public SerialConn
{
public:
    SerialConnStub() : SerialConn("not/used") {};

    void set_read_buf(uint8_t* buf, uint32_t size);

    bool open(Mode mode, ControlFlags flags) override;
    bool close() override;

    uint32_t bytes_available() override;
    uint32_t read(uint8_t* buf, uint32_t size) override;
private:
    uint8_t _read_buf[MAX_BUF_SIZE] = {0};
    uint32_t _read_buf_size = 0;
};

#endif /* SERIAL_CONN_STUB_H */
