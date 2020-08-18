#ifndef DATA_LOG_QUEUE_H
#define DATA_LOG_QUEUE_H

#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <type_traits>
#include <wall_time.h>
#include <data_log_meta.h>

typedef struct {
    uint64_t data;
    uint8_t rel_timestamp_ms;
    DataLogType type;
    DataLogSignal signal;
} DataLogSample;

class DataLogQueue
{
public:
    template <typename T>
    void push(T data, DataLogSignal signal);

    DataLogSample pop();

    bool is_empty();
private:
    uint32_t _prev_timestamp_ms = wall_time.millis();
    std::deque<DataLogSample> _samples;
    std::mutex _mutex;

    template <typename T>
    DataLogType _get_data_type(T data);

    uint32_t _get_rel_timestamp();

    void _check_signal_type(DataLogSignal signal, DataLogType type);
};

template <typename T>
void DataLogQueue::push(T data, DataLogSignal signal)
{
    const std::lock_guard<std::mutex> lock(_mutex);

    DataLogSample sample;

    std::memcpy(&sample.data, &data, sizeof(T));
    sample.rel_timestamp_ms = _get_rel_timestamp();
    sample.type = _get_data_type(data);
    sample.signal = signal;

    _check_signal_type(signal, sample.type);

    _samples.push_back(sample);
}

template <typename T>
DataLogType DataLogQueue::_get_data_type(T data)
{
    DataLogType type;

    if (std::is_same<T, uint8_t>::value)
    {
        type = DataLogType::UINT8;
    }
    else if (std::is_same<T, uint16_t>::value)
    {
        type = DataLogType::UINT16;
    }
    else if (std::is_same<T, uint32_t>::value)
    {
        type = DataLogType::UINT32;
    }
    else if (std::is_same<T, int8_t>::value)
    {
        type = DataLogType::SINT8;
    }
    else if (std::is_same<T, int16_t>::value)
    {
        type = DataLogType::SINT16;
    }
    else if (std::is_same<T, int32_t>::value)
    {
        type = DataLogType::SINT32;
    }
    else if (std::is_same<T, float>::value)
    {
        type = DataLogType::FLOAT;
    }
    else if (std::is_same<T, double>::value)
    {
        type = DataLogType::DOUBLE;
    }
    else
    {
        throw std::runtime_error("Unsupported data type");
    }

    return type;
}

#endif /* DATA_LOG_QUEUE_H */
