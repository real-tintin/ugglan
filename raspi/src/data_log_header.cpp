#include <data_log_header.h>

static const uint8_t JSON_INDENT_SIZE = 4;

using json = nlohmann::ordered_json;

void _add_header_start_time(json& header, std::time_t start_time);
void _add_header_data_log_metadata(json& header);

std::string generate_header(std::time_t start_time)
{
    json header;

    _add_header_start_time(header, start_time);
    _add_header_data_log_metadata(header);

    return header.dump(JSON_INDENT_SIZE);
}

void _add_header_start_time(json& header, std::time_t t)
{
    std::stringstream buf;

    std::tm tm = *std::localtime(&t);
    buf << std::put_time(&tm, "%FT%TZ"); // ISO-8601.

    header["start_time"] = buf.str();
}

void _add_header_data_log_metadata(json& header)
{
    data_log::utils::add_data_log_metadata_to_json(header);
}
