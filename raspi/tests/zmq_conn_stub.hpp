#ifndef ZMQ_CONN_STUB_HPP
#define ZMQ_CONN_STUB_HPP

#include <cstdint>
#include <memory>
#include <string>

#include <zmq/zmq.hpp>

#include <zmq_conn.hpp>

#define ZMQ_CONN_STUB_IMP(ZmqConnStub, ZmqConnParent)                                                                  \
    class ZmqConnStub : public ZmqConnParent                                                                           \
    {                                                                                                                  \
    public:                                                                                                            \
        ZmqConnStub() : ZmqConnParent("not/used"){};                                                                   \
                                                                                                                       \
        bool open() override                                                                                           \
        {                                                                                                              \
            return true;                                                                                               \
        };                                                                                                             \
                                                                                                                       \
        bool close() override                                                                                          \
        {                                                                                                              \
            return true;                                                                                               \
        };                                                                                                             \
                                                                                                                       \
        size_t recv(zmq::message_t &msg, bool non_blocking = true) override                                            \
        {                                                                                                              \
            msg.rebuild(_exp_recv_msg.data(), _exp_recv_msg.size());                                                   \
            _n_calls_recv++;                                                                                           \
                                                                                                                       \
            return _exp_recv_msg.size();                                                                               \
        }                                                                                                              \
                                                                                                                       \
        size_t send(zmq::message_t &msg, bool non_blocking = true) override                                            \
        {                                                                                                              \
            _last_send_msg.rebuild(msg.data(), msg.size());                                                            \
            _n_calls_send++;                                                                                           \
                                                                                                                       \
            return _last_send_msg.size();                                                                              \
        }                                                                                                              \
                                                                                                                       \
        void set_exp_recv_msg(zmq::message_t &msg)                                                                     \
        {                                                                                                              \
            _exp_recv_msg.rebuild(msg.data(), msg.size());                                                             \
        }                                                                                                              \
                                                                                                                       \
        uint32_t get_n_calls_recv() const                                                                              \
        {                                                                                                              \
            return _n_calls_recv;                                                                                      \
        }                                                                                                              \
                                                                                                                       \
        uint32_t get_n_calls_send() const                                                                              \
        {                                                                                                              \
            return _n_calls_send;                                                                                      \
        }                                                                                                              \
                                                                                                                       \
        const zmq::message_t &get_last_send_msg()                                                                      \
        {                                                                                                              \
            return _last_send_msg;                                                                                     \
        }                                                                                                              \
                                                                                                                       \
        void clear_last_send_msg()                                                                                     \
        {                                                                                                              \
            return _last_send_msg.rebuild();                                                                           \
        }                                                                                                              \
                                                                                                                       \
    private:                                                                                                           \
        zmq::message_t _exp_recv_msg;                                                                                  \
        zmq::message_t _last_send_msg;                                                                                 \
                                                                                                                       \
        uint32_t _n_calls_recv = 0;                                                                                    \
        uint32_t _n_calls_send = 0;                                                                                    \
    };

ZMQ_CONN_STUB_IMP(ZmqRepStub, ZmqRep)
ZMQ_CONN_STUB_IMP(ZmqPushStub, ZmqPush)

#endif /* ZMQ_CONN_STUB_HPP */
