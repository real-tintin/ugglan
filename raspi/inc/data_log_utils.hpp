#ifndef DATA_LOG_UTILS_HPP
#define DATA_LOG_UTILS_HPP

#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include <data_log_signals.hpp>

namespace data_log
{
namespace utils
{
void add_data_log_metadata_to_json(nlohmann::ordered_json &data);

size_t get_data_log_type_size(const DataLogType &type);

DataLogSignalInfo get_data_log_signal_info(const DataLogSignal &signal);
} /* namespace utils */
} /* namespace data_log */

#endif /* DATA_LOG_UTILS_HPP */
