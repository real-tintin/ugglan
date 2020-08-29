#include <data_logger.h>

DataLogger::DataLogger(DataLogQueue& queue, std::filesystem::path root_path) :
    _queue(queue),
    _root_path(root_path)
{
}

void DataLogger::start()
{
    _create_file_path();
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

std::filesystem::path DataLogger::get_file_path()
{
    return _file_path;
}

void DataLogger::_create_and_write_header()
{
    std::time_t now_time = std::time(nullptr);

    std::string header_json = generate_header(now_time);
    std::string header_gzip = gzip::compress(header_json.c_str(), header_json.size());

    _write(header_gzip.c_str(), header_gzip.size());
}

void DataLogger::_create_file_path()
{
    std::stringstream file_name;
    std::time_t now_time = std::time(nullptr);

    std::tm tm = *std::localtime(&now_time);
    file_name << std::put_time(&tm, "%Y%m%d%H%M%S") << "." << DATA_LOG_FILE_EXT;

    _file_path = _root_path / file_name.str();
}

void DataLogger::_open()
{
    _ofs = std::ofstream(_file_path, std::ios::binary);
}

void DataLogger::_write(const char* buf, uint32_t size)
{
    _ofs.write(buf, size);
}

void DataLogger::_close()
{
    _ofs.close();
}
