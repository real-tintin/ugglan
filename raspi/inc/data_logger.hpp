#ifndef DATA_LOGGER_HPP
#define DATA_LOGGER_HPP

#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <string>

#include <base64/base64.hpp>
#include <gzip/compress.hpp>

#include <common_utils.hpp>
#include <data_log_header.hpp>
#include <data_log_queue.hpp>
#include <data_log_signals.hpp>
#include <logger.hpp>

inline const std::string DATA_LOG_ENDL = "\n";
inline const std::string DATA_LOG_FILE_EXT = "dat";

class DataLogger
{
  public:
    DataLogger(DataLogQueue &queue, std::filesystem::path root_path);

    void start();
    void pack();
    void stop();

    std::filesystem::path get_file_path();

  private:
    DataLogQueue &_queue;
    std::ofstream _ofs;

    std::filesystem::path _root_path;
    std::filesystem::path _file_path;

    void _create_and_write_header();
    void _create_file_path();

    void _pack_queue_until_empty();

    void _open();
    void _write(uint8_t *buf, size_t size);
    void _close();
};

#endif /* DATA_LOGGER_HPP */
