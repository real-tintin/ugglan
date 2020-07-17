#include <wall_time.h>

WallTime wall_time;
static auto _t_start = std::chrono::steady_clock::now();

uint32_t WallTime::millis()
{
    auto t_now = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(t_now - _t_start).count();
}

uint32_t WallTime::micros()
{
    auto t_now = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::micro>(t_now - _t_start).count();
}
