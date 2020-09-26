#ifndef DATA_LOG_HEADER_H
#define DATA_LOG_HEADER_H

#include <cstdint>
#include <string>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <nlohmann/json.hpp>
#include <data_log_signals.h>

std::string generate_header(std::time_t start_time);

#endif /* DATA_LOG_HEADER_H */
