#ifndef SERIAL_CONN_H
#define SERIAL_CONN_H

#include <string>
#include <cstdint>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <logger.h>

struct ControlFlags {
    tcflag_t c_cflag;
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_lflag;
};

typedef uint16_t Mode;

#if !defined(UNIT_TEST)
class SerialConn
{
public:
    SerialConn(std::string device);

    ~SerialConn();

    bool open(Mode mode, ControlFlags flags);

    uint32_t get_bytes_available();
    uint32_t read(uint8_t* buf, uint32_t size);
private:
    int8_t _fd;
    std::string _device;
};
#endif

#endif /* SERIAL_CONN_H */
