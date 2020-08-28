#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <cstdint>
#include <string>
#include <ctime>
#include <fstream>
#include <gzip/compress.hpp>
#include <data_log_header.h>
#include <data_log_queue.h>

#if defined(UNIT_TEST)
#include <data_log_test_signals.h>
#else
#include <data_log_signals.h>
#endif

class DataLogger
{
public:
    DataLogger(DataLogQueue& queue, std::string path);

    void start();
    void pack();
    void stop();
private:
    DataLogQueue& _queue;
    std::string _path;
    std::ofstream _ofs;

    void _create_and_write_header();

    void _open();
    void _write(const char* buf, uint32_t size);
    void _close();
};

#endif /* DATA_LOGGER_H */
