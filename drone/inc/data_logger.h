#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <cstdint>
#include <string>
#include <ctime>

#if defined(UNIT_TEST)
#include <data_log_test_signals.h>
#else
#include <data_log_signals.h>
#endif

std::string generate_header(std::time_t start_time);

#endif /* DATA_LOGGER_H */
