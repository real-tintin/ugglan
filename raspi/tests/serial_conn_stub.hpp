#ifndef SERIAL_CONN_STUB_HPP
#define SERIAL_CONN_STUB_HPP

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
extern "C"
{
#include <fcntl.h>
#include <stdio.h>
}

#include <serial_conn.h>

inline const size_t MAX_BUF_SIZE = 4096;

class SerialConnStub : public SerialConn {
  public:
    SerialConnStub() : SerialConn("not/used"){};

    void set_read_buf(uint8_t *buf, size_t size);

    bool open(Mode mode, ControlFlags flags) override;
    bool close() override;

    size_t bytes_available() override;
    size_t read(uint8_t *buf, size_t size) override;

  private:
    uint8_t _read_buf[MAX_BUF_SIZE] = {0};
    size_t _read_buf_size = 0;
};

#endif /* SERIAL_CONN_STUB_HPP */
