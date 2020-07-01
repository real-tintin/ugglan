#ifndef SERIAL_CONN_H
#define SERIAL_CONN_H

#include <stdio.h>
#include <fcntl.h>
#include <cstdint>

#include <termios.h>

typedef struct
{
    tcflag_t c_cflag;
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_lflag;
} ControlFlags;

typedef uint16_t Mode;

#if !defined(UNIT_TEST)
class SerialConn
{
public:
    SerialConn(const char* device);

    ~SerialConn();

    bool open(Mode mode, ControlFlags flags);

    uint32_t read(uint8_t* buf);
private:
    int8_t _fd;
    const char* _device;
};
#endif

#endif /* SERIAL_CONN_H */
