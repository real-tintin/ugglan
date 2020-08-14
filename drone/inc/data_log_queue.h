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
};

template <typename T>
void DataLogQueue::push(T data, DataLogSignal signal)
{
    const std::lock_guard<std::mutex> lock(_mutex);

    DataLogSample sample;
    std::memcpy(&sample.data, &data, sizeof(T));
    sample.signal = signal;

    if ((wall_time.millis() - _prev_timestamp_ms) < UINT8_MAX)
    {
        sample.rel_timestamp_ms = wall_time.millis() - _prev_timestamp_ms;
    }
    else
    {
        sample.rel_timestamp_ms = UINT8_MAX;
        throw std::runtime_error("Data log queue timstamp overflow. Called too seldom");
    }

    if (std::is_same<T, uint8_t>::value)
    {
        sample.type = DataLogType::UINT8;
    }
    else if (std::is_same<T, uint16_t>::value)
    {
        sample.type = DataLogType::UINT16;
    }
    else if (std::is_same<T, uint32_t>::value)
    {
        sample.type = DataLogType::UINT32;
    }
    else if (std::is_same<T, int8_t>::value)
    {
        sample.type = DataLogType::SINT8;
    }
    else if (std::is_same<T, int16_t>::value)
    {
        sample.type = DataLogType::SINT16;
    }
    else if (std::is_same<T, int32_t>::value)
    {
        sample.type = DataLogType::SINT32;
    }
    else if (std::is_same<T, float>::value)
    {
        sample.type = DataLogType::FLOAT;
    }
    else if (std::is_same<T, double>::value)
    {
        sample.type = DataLogType::DOUBLE;
    }
    else
    {
        throw std::runtime_error("Unsupported data type.");
    }

    _samples.push_back(sample);
}

#endif /* DATA_LOG_QUEUE_H */
