#include <serial_conn_stub.h>

void SerialConnStub::set_read_buf(uint8_t* buf, uint32_t size)
{
    _read_buf_size = size;

    assert(_read_buf_size <= MAX_BUF_SIZE);
    std::memcpy(_read_buf, buf, _read_buf_size);
}

bool SerialConnStub::open(Mode mode, ControlFlags flags)
{
    return true;
}

uint32_t SerialConnStub::bytes_available()
{
    return _read_buf_size;
}

uint32_t SerialConnStub::read(uint8_t* buf, uint32_t size)
{
    uint32_t bytes_to_read = std::min(_read_buf_size, size);
    std::memcpy(buf, _read_buf, bytes_to_read);

    return bytes_to_read;
}
