#include <serial_conn.hpp>

SerialConn::SerialConn(std::string device) : _device(device)
{
}

bool SerialConn::open(Mode mode, ControlFlags flags)
{
    if ((_fd = ::open(_device.c_str(), mode)) != -1)
    {
        struct termios options;

        tcgetattr(_fd, &options);

        options.c_cflag = flags.c_cflag;
        options.c_iflag = flags.c_iflag;
        options.c_oflag = flags.c_oflag;
        options.c_lflag = flags.c_lflag;

        tcflush(_fd, TCIFLUSH);
        tcsetattr(_fd, TCSANOW, &options);

        logger.debug("Successfully opened serial connection at: " + _device);
        return true;
    }

    logger.error("Failed to open serial connection at: " + _device);
    return false;
}

bool SerialConn::close()
{
    if (::close(_fd) != -1)
    {
        logger.debug("Successfully closed serial connection at: " + _device);
        return true;
    }

    logger.error("Failed to close serial connection at: " + _device);
    return false;
}

size_t SerialConn::bytes_available()
{
    uint32_t bytes_available;
    ioctl(_fd, FIONREAD, &bytes_available);

    return bytes_available;
}

size_t SerialConn::read(uint8_t *buf, size_t size)
{
    ssize_t n_bytes = -1;
    if (_fd != -1)
    {
        n_bytes = ::read(_fd, buf, size);
    }

    if (n_bytes >= 0)
    {
        return n_bytes;
    }

    return 0;
}
