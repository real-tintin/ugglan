
#ifndef STREAMER_SERVER_HPP
#define STREAMER_SERVER_HPP

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <data_log_queue.hpp>
#include <data_log_signals.hpp>
#include <data_log_utils.hpp>
#include <logger.hpp>
#include <streamer_server_msg.hpp>
#include <wall_time.hpp>
#include <zmq_conn.hpp>

namespace streamer
{
using json = nlohmann::ordered_json;

inline const size_t MAX_SELECTED_DATA_LOG_SIGNALS = 25;

class Server
{
public:
    Server(ZmqRep &request, ZmqPush &stream, DataLogQueue &data_log_queue);

    void connect();
    void execute();
    void disconnect();

private:
    ZmqRep &_request;
    ZmqPush &_stream;

    DataLogQueue &_data_log_queue; // NOLINT(clang-diagnostic-unused-private-field)

    std::vector<DataLogSignal> _sel_data_log_signals{};

    bool _is_streaming = false;

    bool _recv_request(msg::Request &req);

    void _send_data_log_metadata_on_request();
    void _send_selected_data_log_signals_on_request();
    void _send_ok_on_request();
    void _send_error_on_request();
    void _send_response_on_request(msg::Response &res);

    bool _set_selected_data_log_signals(json data);

    void _start_stream();
    void _stop_stream();
    void _send_on_stream();

    void _pack_stream_bytes(std::vector<uint8_t> &package);

    static void _append_bytes_to_vector(std::vector<uint8_t> &v, const uint8_t *const bytes, size_t size);
    static bool _is_unique(std::vector<DataLogSignal> signals);
};
} // namespace streamer

#endif /* STREAMER_SERVER_HPP */
