#include <data_logger.h>

using json = nlohmann::ordered_json;

void _add_header_start_time(json& header, std::time_t start_time);
void _add_header_types(json& header);
void _add_header_groups(json& header);
void _add_header_signals(json& header);

std::string generate_header(std::time_t start_time)
{
    json header;

    _add_header_start_time(header, start_time);
    _add_header_types(header);
    _add_header_groups(header);
    _add_header_signals(header);

    return header.dump(4);
}

void _add_header_start_time(json& header, std::time_t t)
{
    std::stringstream buf;

    std::tm tm = *std::localtime(&t);
    buf << std::put_time(&tm, "%FT%TZ"); // ISO-8601.

    header["start_time"] = buf.str();
}

void _add_header_types(json& header)
{
    for (auto const& it: DATA_LOG_TYPE_MAP)
    {
        std::string type_id = std::to_string((uint16_t) it.first);
        std::string type_str = it.second;

        header["types"][type_id] = type_str;
    }
}

void _add_header_groups(json& header)
{
    for (auto const& it: DATA_LOG_GROUP_MAP)
    {
        std::string group_id = std::to_string((uint16_t) it.first);
        DataLogGroupInfo group_info = it.second;

        header["groups"][group_id] = group_info.name;
    }
}

void _add_header_signals(json& header)
{
    for (auto const& it: DATA_LOG_SIGNAL_MAP)
    {
        std::string signal_id = std::to_string((uint16_t) it.first);
        DataLogSignalInfo signal_info = it.second;

        std::string signal_name = signal_info.name;
        uint16_t signal_group = (uint16_t) signal_info.group;
        uint16_t signal_type = (uint16_t) signal_info.type;

        header["signals"][signal_id] = {
            {"name", signal_name},
            {"group", signal_group},
            {"type", signal_type}
        };
    }
}
