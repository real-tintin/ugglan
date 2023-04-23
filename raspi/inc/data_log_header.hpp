#ifndef DATA_LOG_HEADER_HPP
#define DATA_LOG_HEADER_HPP

#include <cstdint>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>

#include <data_log_utils.hpp>

std::string generate_header(std::time_t start_time);

#endif /* DATA_LOG_HEADER_HPP */
