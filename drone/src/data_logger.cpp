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
    std::string header = generate_header(now_time);

    // TODO: gzip header.
    _write(header.c_str(), header.size());
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
