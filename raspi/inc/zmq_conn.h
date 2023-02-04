#ifndef ZMQ_CONN_H
#define ZMQ_CONN_H

#include <cstdint>
#include <string>
#include <zmq/zmq.hpp>
#include <logger.h>

class ZmqConn
{
public:
    ZmqConn(std::string address, zmq::socket_type type);

    virtual bool open();
    virtual bool close();

    virtual size_t recv(zmq::message_t& msg, bool non_blocking = true);
    virtual size_t send(zmq::message_t& msg, bool non_blocking = true);
protected:
    static zmq::context_t _context;

    std::string _address;
    zmq::socket_t _socket;
};

class ZmqRep : public ZmqConn
{
public:
    ZmqRep(std::string address) : ZmqConn(address,  zmq::socket_type::rep) {}
};

class ZmqPush : public ZmqConn
{
public:
    ZmqPush(std::string address) : ZmqConn(address,  zmq::socket_type::push) {}
};

#endif /* ZMQ_CONN_H */
