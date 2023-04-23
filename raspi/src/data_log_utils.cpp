#include <data_log_utils.hpp>

namespace data_log
{
namespace utils
{
namespace
{
using json = nlohmann::ordered_json;

void _add_types(json &header)
{
    for (auto const &it : DATA_LOG_TYPE_MAP)
    {
        std::string type_id = std::to_string(static_cast<uint16_t>(it.first));
        std::string type_str = it.second;

        header["types"][type_id] = type_str;
    }
}

void _add_groups(json &header)
{
    for (auto const &it : DATA_LOG_GROUP_MAP)
    {
        std::string group_id = std::to_string(static_cast<uint16_t>(it.first));
        DataLogGroupInfo group_info = it.second;

        header["groups"][group_id] = group_info.name;
    }
}

void _add_signals(json &header)
{
    for (auto const &it : DATA_LOG_SIGNAL_MAP)
    {
        std::string signal_id = std::to_string(static_cast<uint16_t>(it.first));
        DataLogSignalInfo signal_info = it.second;

        std::string signal_name = signal_info.name;
        uint16_t signal_group = static_cast<uint16_t>(signal_info.group);
        uint16_t signal_type = static_cast<uint16_t>(signal_info.type);

        header["signals"][signal_id] = {{"name", signal_name}, {"group", signal_group}, {"type", signal_type}};
    }
}
} // namespace

void add_data_log_metadata_to_json(json &data)
{
    _add_types(data);
    _add_groups(data);
    _add_signals(data);
}

size_t get_data_log_type_size(const DataLogType &type)
{
    switch (type)
    {
    case DataLogType::BOOL:
        return sizeof(bool);
    case DataLogType::UINT8:
        return sizeof(uint8_t);
    case DataLogType::UINT16:
        return sizeof(uint16_t);
    case DataLogType::UINT32:
        return sizeof(uint32_t);
    case DataLogType::SINT8:
        return sizeof(int8_t);
    case DataLogType::SINT16:
        return sizeof(int16_t);
    case DataLogType::SINT32:
        return sizeof(int32_t);
    case DataLogType::FLOAT:
        return sizeof(float);
    case DataLogType::DOUBLE:
        return sizeof(double);
    default:
        throw std::runtime_error("Unknown signal type");
    }
}

DataLogSignalInfo get_data_log_signal_info(const DataLogSignal &signal)
{
    auto it = DATA_LOG_SIGNAL_MAP.find(signal);

    if (it != DATA_LOG_SIGNAL_MAP.end())
    {
        DataLogSignalInfo info = it->second;
        return info;
    }

    throw std::runtime_error("Unknown signal type");
}
} /* namespace utils */
} /* namespace data_log */
