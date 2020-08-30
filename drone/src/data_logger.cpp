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

    std::string header_json = generate_header(now_time);
    std::string header_gzip = gzip::compress(header_json.c_str(), header_json.size());
    std::string header_base64 = base64_encode(header_gzip.c_str(), header_gzip.size());

    _write(header_base64.c_str(), header_base64.size());
    _write(DATA_LOG_ENDL.c_str(), DATA_LOG_ENDL.size());
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

    while (!_queue.is_empty())
    {
        sample = _queue.pop();

        _write((const char*) &sample.signal, sizeof(uint16_t));

        switch(sample.type)
        {
            case DataLogType::UINT8:
                _write((const char*) &sample.data, sizeof(uint8_t));
                break;
            case DataLogType::UINT16:
                 _write((const char*) &sample.data, sizeof(uint16_t));
                break;
            case DataLogType::UINT32:
                _write((const char*) &sample.data, sizeof(uint32_t));
                break;
            case DataLogType::SINT8:
                _write((const char*) &sample.data, sizeof(int8_t));
                break;
            case DataLogType::SINT16:
                _write((const char*) &sample.data, sizeof(int16_t));
                break;
            case DataLogType::SINT32:
                _write((const char*) &sample.data, sizeof(int32_t));
                break;
            case DataLogType::FLOAT:
                _write((const char*) &sample.data, sizeof(float));
                break;
            case DataLogType::DOUBLE:
                _write((const char*) &sample.data, sizeof(double));
                break;
        }

        _write((const char*) &sample.rel_timestamp_ms, sizeof(uint8_t));
    }
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
