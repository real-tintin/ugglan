#include <serial_conn.h>

#if !defined(UNIT_TEST)

SerialConn::SerialConn(uint8_t address) :
    _device(device)
{
}

SerialConn::~SerialConn()
{
    close(_fd);
}

bool SerialConn::open(Mode mode, ControlFlags flags)
{
    if ((_fd = open(_device, mode)) < 0)
    {
        struct termios options;

        tcgetattr(_fd, &options);

        options.c_cflag = flags.c_cflag;
        options.c_iflag = flags.c_iflag;
        options.c_oflag = flags.c_oflag;
        options.c_lflag = flags.c_lflag;

        tcflush(_fd, TCIFLUSH);
        tcsetattr(_fd, TCSANOW, &options);

        return true;
    }

    return false;
}

uint32_t SerialConn::get_bytes_available()
{
    uint32_t bytes_available;

    ioctl(_fd, FIONREAD, &bytes_available);

    return bytes_available;
}

uint32_t SerialConn::read(uint8_t* buf, uint32_t size)
{
    if (_fd != -1) {
        return ::read(_fd, buf, size); // :: to avoid collision
    }

    return 0;
}

#endif