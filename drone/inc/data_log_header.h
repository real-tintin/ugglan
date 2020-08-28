#ifndef DATA_LOG_HEADER_H
#define DATA_LOG_HEADER_H

#include <cstdint>
#include <string>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <nlohmann/json.hpp>

#if defined(UNIT_TEST)
#include <data_log_test_signals.h>
#else
#include <data_log_signals.h>
#endif

std::string generate_header(std::time_t start_time);

#endif /* DATA_LOG_HEADER_H */
