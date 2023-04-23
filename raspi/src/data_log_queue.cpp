#include <data_log_queue.hpp>

DataLogSample DataLogQueue::pop()
{
    const std::lock_guard<std::mutex> lock(_mutex);

    DataLogSample front_sample = _samples.front();
    _samples.pop_front();

    return front_sample;
}

bool DataLogQueue::is_empty()
{
    const std::lock_guard<std::mutex> lock(_mutex);
    return _samples.empty();
}

uint32_t DataLogQueue::_get_rel_timestamp()
{
    uint32_t rel_timestamp_ms;
    uint32_t now_timestamp_ms = WallTime::millis();

    if (_first_sample)
    {
        _prev_timestamp_ms = now_timestamp_ms;
        _first_sample = false;
    }

    if ((now_timestamp_ms - _prev_timestamp_ms) < UINT8_MAX)
    {
        rel_timestamp_ms = now_timestamp_ms - _prev_timestamp_ms;
    }
    else
    {
        rel_timestamp_ms = UINT8_MAX;
        logger.error("Data log queue timstamp overflow. Called too seldom.");
    }

    _prev_timestamp_ms = now_timestamp_ms;

    return rel_timestamp_ms;
}

void DataLogQueue::_check_signal_type(DataLogSignal &signal, DataLogType &type)
{
    DataLogSignalInfo info = data_log::utils::get_data_log_signal_info(signal);

    if (info.type != type)
    {
        throw std::runtime_error("Data log signal type mismatch");
    }
}
