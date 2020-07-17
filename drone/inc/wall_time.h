#ifndef WALL_TIME_H
#define WALL_TIME_H

#include <chrono>

class WallTime
{
public:
    double millis();
    double micros();
};

extern WallTime wall_time;

#endif /* WALL_TIME_H */
