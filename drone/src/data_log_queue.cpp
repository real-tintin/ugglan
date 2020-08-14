#include <data_log_queue.h>

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
    return _samples.size() == 0;
}
