#include <wall_time.hpp>

static auto _t_start = std::chrono::high_resolution_clock::now();

uint32_t WallTime::millis()
{
    auto t_now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(t_now - _t_start).count();
}

uint32_t WallTime::micros()
{
    auto t_now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::micro>(t_now - _t_start).count();
}
