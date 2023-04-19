
#ifndef STREAMER_SERVER_MSG_HPP
#define STREAMER_SERVER_MSG_HPP

#include <string>

#include <nlohmann/json.hpp>

#include <common_utils.h>
#include <logger.h>
#include <zmq/zmq.hpp>
#include <zmq_conn.h>

namespace streamer {
namespace msg {
using json = nlohmann::ordered_json;

enum class Method
{
    Get_DataLogMetadata = 0,
    Get_SelectedDataLogSignals = 1,

    Set_StartStream = 2,
    Set_StopStream = 3,
    Set_SelectedDataLogSignals = 4
};

enum class Code
{
    Ok = 0,
    Error = 1
};

template <class T> class Base {
  public:
    Base() : _metadata{}, _data({}), _valid{false}
    {
    }

    Base(T metadata) : _metadata{metadata}, _data({}), _valid{true}
    {
    }

    Base(T metadata, json data) : _metadata{metadata}, _data(data), _valid{true}
    {
    }

    Base(zmq::message_t &msg)
    {
        from_msg(msg);
    }

    const json data()
    {
        return _data;
    }
    const bool valid()
    {
        return _valid;
    }

    void from_msg(const zmq::message_t &msg)
    {
        try
        {
            std::string packed_as_str{(const char *)msg.data(), msg.size()};
            std::string unpacked_as_str = common_utils::unpack_base64_gzip(packed_as_str);
            json unpacked_as_json = json::parse(unpacked_as_str);

            _metadata = static_cast<T>(unpacked_as_json[_get_metadata_key()]);
            _data = unpacked_as_json[_get_data_key()];

            _valid = true;
        } catch (const std::exception &e)
        {
            logger.error("Unable to unpack request: " + std::string(e.what()));
            _valid = false;
        }
    }

    void to_msg(zmq::message_t &msg)
    {
        if (_valid)
        {
            json unpacked_as_json = as_json();
            std::string unpacked_as_str = unpacked_as_json.dump();
            std::string packed_as_str = common_utils::pack_gzip_base64(unpacked_as_str);

            msg.rebuild(packed_as_str);
        }
        else
        {
            logger.error("Message is invalid, can't be packed");
        }
    }

    std::string as_string()
    {
        json j = as_json();

        return j.dump();
    }

    json as_json()
    {
        json j = json("invalid");

        if (_valid)
        {
            j = json({{_get_metadata_key(), static_cast<int>(_metadata)}, {_get_data_key(), _data}});
        }

        return j;
    }

  protected:
    const T _get_metadata()
    {
        return _metadata;
    }

    virtual const std::string _get_metadata_key()
    {
        std::logic_error("Not implemented");
        return "";
    }

  private:
    T _metadata;
    json _data;
    bool _valid;

    const std::string _get_data_key()
    {
        return "data";
    }
};

class Request : public Base<Method> {
  public:
    using Base::Base;

    const Method method()
    {
        return _get_metadata();
    }

  protected:
    const std::string _get_metadata_key() override
    {
        return "method";
    }
};

class Response : public Base<Code> {
  public:
    using Base::Base;

    const Code code()
    {
        return _get_metadata();
    }

  protected:
    const std::string _get_metadata_key() override
    {
        return "code";
    }
};
} // namespace msg
} // namespace streamer

#endif /* STREAMER_SERVER_MSG_HPP */
