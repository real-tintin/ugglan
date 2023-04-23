#ifndef WALL_TIME_HPP
#define WALL_TIME_HPP

#include <chrono>
#include <cstdint>

class WallTime
{
public:
    static uint32_t millis();
    static uint32_t micros();
};

#endif /* WALL_TIME_HPP */
