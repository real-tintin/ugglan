#ifndef WALL_TIME_HPP
#define WALL_TIME_HPP

#include <chrono>
#include <cstdint>

class WallTime
{
  public:
    uint32_t millis();
    uint32_t micros();
};

extern WallTime wall_time;

#endif /* WALL_TIME_HPP */
