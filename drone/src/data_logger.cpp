#include <data_logger.h>

DataLogger::DataLogger(DataLogQueue& queue, std::string path) :
    _queue(queue),
    _path(path)
{
}

void DataLogger::start()
{
    _open();
    _create_and_write_header();
}

void DataLogger::pack()
{
    // TODO: pack (write) while queue.size() > 0.
}

void DataLogger::stop()
{
    // TODO: flush data from queue to file.
    _close();
}

void DataLogger::_create_and_write_header()
{
    std::time_t now_time = std::time(nullptr);

    std::string header_json = generate_header(now_time);
    std::string header_gzip = gzip::compress(header_json.c_str(), header_json.size());

    _write(header_gzip.c_str(), header_gzip.size());
}

void DataLogger::_open()
{
    _ofs = std::ofstream(_path, std::ios::binary);
}

void DataLogger::_write(const char* buf, uint32_t size)
{
    _ofs.write(buf, size);
}

void DataLogger::_close()
{
    _ofs.close();
}
