#ifndef SERIAL_CONN_STUB_H
#define SERIAL_CONN_STUB_H

#include <stdio.h>
#include <fcntl.h>
#include <cstdint>
#include <cstring>
#include <cassert>
#include <algorithm>
#include <serial_conn.h>

inline const size_t MAX_BUF_SIZE = 4096;

class SerialConnStub : public SerialConn
{
public:
    SerialConnStub() : SerialConn("not/used") {};

    void set_read_buf(uint8_t* buf, size_t size);

    bool open(Mode mode, ControlFlags flags) override;
    bool close() override;

    size_t bytes_available() override;
    size_t read(uint8_t* buf, size_t size) override;
private:
    uint8_t _read_buf[MAX_BUF_SIZE] = {0};
    size_t _read_buf_size = 0;
};

#endif /* SERIAL_CONN_STUB_H */
