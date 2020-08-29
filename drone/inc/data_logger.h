#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <cstdint>
#include <string>
#include <ctime>
#include <fstream>
#include <filesystem>
#include <gzip/compress.hpp>
#include <data_log_header.h>
#include <data_log_queue.h>

#if defined(UNIT_TEST)
#include <data_log_test_signals.h>
#else
#include <data_log_signals.h>
#endif

inline const std::string DATA_LOG_FILE_EXT = "dat";

class DataLogger
{
public:
    DataLogger(DataLogQueue& queue, std::filesystem::path root_path);

    void start();
    void pack();
    void stop();

    std::filesystem::path get_file_path();
private:
    DataLogQueue& _queue;
    std::ofstream _ofs;

    std::filesystem::path _root_path;
    std::filesystem::path _file_path;

    void _create_and_write_header();
    void _create_file_path();

    void _open();
    void _write(const char* buf, uint32_t size);
    void _close();
};

#endif /* DATA_LOGGER_H */
