#include <data_logger.hpp>

DataLogger::DataLogger(DataLogQueue &queue, std::filesystem::path root_path) : _queue(queue), _root_path(root_path)
{
}

void DataLogger::start()
{
    _create_file_path();
    _open();
    _create_and_write_header();

    logger.info("Data logger starting new log file: " + _file_path.string());
}

void DataLogger::pack()
{
    _pack_queue_until_empty();
}

void DataLogger::stop()
{
    _pack_queue_until_empty();
    _close();
}

std::filesystem::path DataLogger::get_file_path()
{
    return _file_path;
}

void DataLogger::_create_and_write_header()
{
    std::time_t now_time = std::time(nullptr);

    std::string header_unpacked = generate_header(now_time);
    std::string header_packed = common_utils::pack_gzip_base64(header_unpacked);

    _write((uint8_t *)header_packed.c_str(), header_packed.size());
    _write((uint8_t *)DATA_LOG_ENDL.c_str(), DATA_LOG_ENDL.size());
}

void DataLogger::_create_file_path()
{
    std::stringstream file_name;
    std::time_t now_time = std::time(nullptr);

    std::tm tm = *std::localtime(&now_time);
    file_name << std::put_time(&tm, "%Y%m%d%H%M%S") << "." << DATA_LOG_FILE_EXT;

    _file_path = _root_path / file_name.str();
}

void DataLogger::_pack_queue_until_empty()
{
    DataLogSample sample;
    size_t size;

    while (!_queue.is_empty())
    {
        sample = _queue.pop();
        size = data_log::utils::get_data_log_type_size(sample.type);

        _write((uint8_t *)&sample.signal, sizeof(uint16_t));
        _write((uint8_t *)&sample.data, size);
        _write((uint8_t *)&sample.rel_timestamp_ms, sizeof(uint8_t));
    }
}

void DataLogger::_open()
{
    _ofs = std::ofstream(_file_path, std::ios_base::out | std::ios_base::binary);
}

void DataLogger::_write(uint8_t *buf, size_t size)
{
    _ofs.write((char *)buf, size);
}

void DataLogger::_close()
{
    _ofs.close();
}
