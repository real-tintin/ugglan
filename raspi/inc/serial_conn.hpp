#ifndef SERIAL_CONN_HPP
#define SERIAL_CONN_HPP

#include <cstdint>
#include <string>
extern "C"
{
#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
}

#include <logger.hpp>

struct ControlFlags
{
    tcflag_t c_cflag;
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_lflag;
};

typedef uint16_t Mode;

class SerialConn
{
public:
    SerialConn(std::string device);

    virtual bool open(Mode mode, ControlFlags flags);
    virtual bool close();

    virtual size_t bytes_available();
    virtual size_t read(uint8_t *buf, size_t size);

private:
    int8_t _fd;
    std::string _device;
};

#endif /* SERIAL_CONN_HPP */
