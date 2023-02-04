#include <streamer_server.h>

namespace streamer
{
Server::Server(ZmqRep& request, ZmqPush& stream, DataLogQueue& data_log_queue) :
    _request(request),
    _stream(stream),
    _data_log_queue(data_log_queue)
{
}

void Server::connect()
{
    _request.open();
    _stream.open();
}

void Server::execute()
{
    msg::Request req{};

    if (_recv_request(req))
    {
        if (req.valid())
        {
            switch (req.method())
            {
            case msg::Method::Get_DataLogMetadata:
                _send_data_log_metadata_on_request();
                break;

            case msg::Method::Get_SelectedDataLogSignals:
                _send_selected_data_log_signals_on_request();
                break;

            case msg::Method::Set_StartStream:
                _start_stream();
                _send_ok_on_request();
                break;

            case msg::Method::Set_StopStream:
                _stop_stream();
                _send_ok_on_request();
                break;

            case msg::Method::Set_SelectedDataLogSignals:
                if (_set_selected_data_log_signals(req.data()))
                {
                    _send_ok_on_request();
                }
                else
                {
                    _send_error_on_request();
                }
                break;

            default:
                break;
            }
        }
        else
        {
            _send_error_on_request();
        }
    }

    if (_is_streaming)
    {
        _send_on_stream();
    }
}

void Server::disconnect()
{
    _request.close();
    _stream.close();
}

bool Server::_recv_request(msg::Request& req)
{
    zmq::message_t msg{};
    _request.recv(msg);

    if (msg.size() > 0)
    {
        req.from_msg(msg);
        logger.debug("Streamer server received request: " + req.as_string());

        return true;
    }

    return false;
}

void Server::_send_data_log_metadata_on_request()
{
    json data;
    data_log::utils::add_data_log_metadata_to_json(data);

    msg::Response res{msg::Code::Ok, data};

    _send_response_on_request(res);
}

void Server::_send_selected_data_log_signals_on_request()
{
    json data(_sel_data_log_signals);

    msg::Response res{msg::Code::Ok, data};

    _send_response_on_request(res);
}

void Server::_send_ok_on_request()
{
    msg::Response res{msg::Code::Ok};

    _send_response_on_request(res);
}

void Server::_send_error_on_request()
{
    msg::Response res{msg::Code::Error};

    _send_response_on_request(res);
}

void Server::_send_response_on_request(msg::Response& res)
{
    zmq::message_t msg{};

    res.to_msg(msg);
    _request.send(msg);

    logger.debug("Streamer server sent response: " + res.as_string());
}

bool Server::_set_selected_data_log_signals(json data)
{
    if (data.type() != json::value_t::array)
    {
        return false;
    }

    if (data.size() > MAX_SELECTED_DATA_LOG_SIGNALS)
    {
        return false;
    }

    std::vector<DataLogSignal> new_sel_data_log_signals;

    for (json::iterator it = data.begin(); it != data.end(); it++)
    {
        if (it->type() == json::value_t::number_unsigned)
        {
            DataLogSignal signal = static_cast<DataLogSignal>(*it);

            if (DATA_LOG_SIGNAL_MAP.count(signal))
            {
                new_sel_data_log_signals.push_back(signal);
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    if (_is_unique(new_sel_data_log_signals))
    {
        _sel_data_log_signals = new_sel_data_log_signals;

        return true;
    }
    else
    {
        return false;
    }
}

void Server::_start_stream()
{
    _is_streaming = true;
}

void Server::_stop_stream()
{
    _is_streaming = false;
}

void Server::_send_on_stream()
{
    std::vector<uint8_t> package{};

    _pack_stream_bytes(package);
    zmq::message_t msg{package.data(), package.size()};

    _stream.send(msg);
}

void Server::_pack_stream_bytes(std::vector<uint8_t>& package)
{
    uint64_t signal_bytes;
    uint32_t now_timestamp_ms = wall_time.millis();

     _append_bytes_to_vector(package, (uint8_t*) &now_timestamp_ms, sizeof(uint32_t));

    for (auto const& signal: _sel_data_log_signals)
    {
        DataLogSignalInfo info = data_log::utils::get_data_log_signal_info(signal);
        size_t signal_size = data_log::utils::get_data_log_type_size(info.type);

        _data_log_queue.last_signal_data(&signal_bytes, signal);

        _append_bytes_to_vector(package, (uint8_t*) &signal, sizeof(uint16_t));
        _append_bytes_to_vector(package, (uint8_t*) &signal_bytes, signal_size);
    }
}

void Server::_append_bytes_to_vector(std::vector<uint8_t>& v, const uint8_t* const bytes, size_t size)
{
    for (size_t i_byte = 0; i_byte < size; i_byte++)
    {
        v.push_back(bytes[i_byte]);
    }
}

bool Server::_is_unique(std::vector<DataLogSignal> signals)
{
    std::sort(signals.begin(), signals.end());
    bool is_unique = std::adjacent_find(signals.begin(), signals.end()) == signals.end();

    return is_unique;
}
} /* streamer */
