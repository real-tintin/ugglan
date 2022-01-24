#ifndef DATA_LOG_QUEUE_H
#define DATA_LOG_QUEUE_H

#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <map>
#include <type_traits>
#include <wall_time.h>
#include <utils.h>
#include <logger.h>
#include <data_log_signals.h>

inline const uint32_t DATA_LOG_QUEUE_WARN_LARGE = 1e4;

struct DataLogSample {
    uint64_t data;
    uint8_t rel_timestamp_ms;
    DataLogType type;
    DataLogSignal signal;
};

class DataLogQueue
{
public:
    template <typename T>
    void push(T data, DataLogSignal signal);

    DataLogSample pop();

    template <typename T>
    void last_signal_data(T* data, DataLogSignal signal, T default_data = 0);

    bool is_empty();
private:
    bool _first_sample = true;
    uint32_t _prev_timestamp_ms;
    std::deque<DataLogSample> _samples;
    std::map<DataLogSignal, uint64_t> _last_signal_map;
    std::mutex _mutex;

    template <typename T>
    DataLogType _get_data_type(T data);

    template <typename T>
    void _update_last_signal_map(T data, DataLogSignal signal);

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
    _update_last_signal_map(data, signal);

    if (_samples.size() >= DATA_LOG_QUEUE_WARN_LARGE)
    {
        logger.warn("Data log queue contains " + std::to_string(_samples.size()) + \
                    " sample(s). Consider calling more often." );
    }
}

template <typename T>
void DataLogQueue::last_signal_data(T* data, DataLogSignal signal, T default_data)
{
    const std::lock_guard<std::mutex> lock(_mutex);

    auto it = _last_signal_map.find(signal);

    if (it != _last_signal_map.end())
    {
        std::memcpy(data, &it->second, sizeof(T));
    }
    else
    {
        *data = default_data;
    }
}

template <typename T>
DataLogType DataLogQueue::_get_data_type(T data)
{
    DataLogType type;

    if constexpr (std::is_same<T, bool>::value)
    {
        type = DataLogType::BOOL;
    }
    else if constexpr (std::is_same<T, uint8_t>::value)
    {
        type = DataLogType::UINT8;
    }
    else if constexpr (std::is_same<T, uint16_t>::value)
    {
        type = DataLogType::UINT16;
    }
    else if constexpr (std::is_same<T, uint32_t>::value)
    {
        type = DataLogType::UINT32;
    }
    else if constexpr (std::is_same<T, int8_t>::value)
    {
        type = DataLogType::SINT8;
    }
    else if constexpr (std::is_same<T, int16_t>::value)
    {
        type = DataLogType::SINT16;
    }
    else if constexpr (std::is_same<T, int32_t>::value)
    {
        type = DataLogType::SINT32;
    }
    else if constexpr (std::is_same<T, float>::value)
    {
        type = DataLogType::FLOAT;
    }
    else if constexpr (std::is_same<T, double>::value)
    {
        type = DataLogType::DOUBLE;
    }
    else
    {
        throw std::runtime_error("Unsupported data type");
    }

    return type;
}

template <typename T>
void DataLogQueue::_update_last_signal_map(T data, DataLogSignal signal)
{
    std::memcpy(&_last_signal_map[signal], &data, sizeof(T));
}

#endif /* DATA_LOG_QUEUE_H */
