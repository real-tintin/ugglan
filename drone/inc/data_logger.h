#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <cstdint>
#include <string>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <nlohmann/json.h>
#include <data_log_queue.h>

#if defined(UNIT_TEST)
#include <data_log_test_signals.h>
#else
#include <data_log_signals.h>
#endif

class DataLogger
{
public:
    DataLogger(DataLogQueue& data_log_queue);

    void start();
    void pack();
    void stop();
private:
    DataLogQueue& _data_log_queue;
};

#endif /* DATA_LOGGER_H */
