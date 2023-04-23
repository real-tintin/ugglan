#include <zmq_conn.hpp>

zmq::context_t ZmqConn::_context;

ZmqConn::ZmqConn(std::string address, zmq::socket_type type) : _address(address), _socket(_context, type)
{
}

bool ZmqConn::open()
{
    try
    {
        _socket.bind(_address);
        logger.debug("Successfully opened zmq connection at: " + _address);

        return true;
    }
    catch (...)
    {
        logger.error("Failed to open zmq connection at: " + _address);

        return false;
    }
}

bool ZmqConn::close()
{
    _socket.close();

    return true;
}

size_t ZmqConn::recv(zmq::message_t &msg, bool non_blocking)
{
    zmq::recv_flags flags = zmq::recv_flags::none;

    if (non_blocking)
    {
        flags = zmq::recv_flags::dontwait;
    }

    zmq::recv_result_t result = _socket.recv(msg, flags);

    return result.value_or(0);
}

size_t ZmqConn::send(zmq::message_t &msg, bool non_blocking)
{
    zmq::send_flags flags = zmq::send_flags::none;

    if (non_blocking)
    {
        flags = zmq::send_flags::dontwait;
    }

    zmq::send_result_t result = _socket.send(msg, flags);

    return result.value_or(0);
}
