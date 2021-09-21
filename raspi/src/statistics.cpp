#include <statistics.h>

namespace statistics
{
    RollingStats::RollingStats(uint32_t window_size) :
        _window_size(window_size)
    {
        _buf = new double[window_size];
    }

    void RollingStats::update(double x_new)
    {
        /* Using Welford's online algorithm. Inspired by:
        https://stackoverflow.com/questions/5147378/rolling-variance-algorithm */

        switch (_buf_status)
        {
            case BufStatus::Empty:
                _update_empty_buf(x_new);
                break;

            case BufStatus::NotEmptyNorFull:
                _update_not_empty_nor_full_buf(x_new);
                break;

            case BufStatus::Full:
                _update_full_buf(x_new);
                break;

            default:
                throw std::logic_error("Invalid case.");
                break;
        }
    }

    double RollingStats::get_mean()
    {
        _throw_error_if_buf_not_full();
        return _mean;
    }

    double RollingStats::get_variance()
    {
        _throw_error_if_buf_not_full();
        return _variance_sum / static_cast<double>(_window_size);
    }

    void RollingStats::_throw_error_if_buf_not_full()
    {
        if (_buf_status != BufStatus::Full)
        {
            throw std::logic_error("Estimates not available yet");
        }
    }

    void RollingStats::_update_full_buf(double x_new)
    {
        uint32_t oldest_index = (_latest_index + 1) % _window_size;
        double x_old = _buf[oldest_index];

        double new_mean = _mean + (x_new - x_old) / static_cast<double>(_window_size);

        double new_variance_sum_diff = (x_new - _mean) * (x_new - new_mean);
        double old_variance_sum_diff = (x_old - _mean) * (x_old - new_mean);

        _mean = new_mean;
        _variance_sum += new_variance_sum_diff - old_variance_sum_diff;

        _buf[oldest_index] = x_new;
        _latest_index = oldest_index;
    }

    void RollingStats::_update_not_empty_nor_full_buf(double x_new)
    {
        double new_mean = _mean + (x_new - _mean) / static_cast<double>(_latest_index + 2);

        _variance_sum += (x_new - _mean) * (x_new - new_mean);
        _mean = new_mean;

        _latest_index++;
        _buf[_latest_index] = x_new;

        if (_latest_index >= (_window_size - 1))
        {
            _buf_status = BufStatus::Full;
        }
    }

    void RollingStats::_update_empty_buf(double x_new)
    {
        _mean = x_new;
        _variance_sum = 0;

        _buf[_latest_index] = x_new;
        _buf_status = BufStatus::NotEmptyNorFull;
    }
}
