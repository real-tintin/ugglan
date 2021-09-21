#ifndef STATISTICS_H
#define STATISTICS_H

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace statistics
{
    class RollingStats
    {
    public:
        RollingStats(uint32_t window_size);

        void update(double x_new);

        double get_mean();
        double get_variance();

        bool estimates_available() { return _buf_status == BufStatus::Full; };
    private:
        enum class BufStatus {
            Empty,
            NotEmptyNorFull,
            Full
        };

        uint32_t _window_size;

        double _mean;
        double _variance_sum;

        double* _buf;
        uint32_t _latest_index = 0;
        BufStatus _buf_status = BufStatus::Empty;

        void _update_full_buf(double x_new);
        void _update_not_empty_nor_full_buf(double x_new);
        void _update_empty_buf(double x_new);

        void _throw_error_if_buf_not_full();
    };
}

#endif /* STATISTICS_H */
