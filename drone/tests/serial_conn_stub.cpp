#include <serial_conn_stub.h>

SerialConn::SerialConn()
{
}

void SerialConn::set_read_buf(uint8_t* buf)
{
    _read_buf_size = sizeof(buf);

    assert(_read_buf_size < MAX_BUF_SIZE);
    std::memcpy(buf, _read_buf, _read_buf_size);
}

bool SerialConn::open(Mode mode, ControlFlags flags)
{
    return true;
}

uint32_t SerialConn::read(uint8_t* buf)
{
    std::memcpy(_read_buf, buf, _read_buf_size);
    return _read_buf_size;
}
