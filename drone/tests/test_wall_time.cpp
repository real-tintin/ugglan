#include <catch.h>

#include <wall_time.h>

TEST_CASE("Wall time")
{
    double t_start_ms = wall_time.millis();
    double t_start_us = wall_time.micros();

    SECTION("close to zero")
    {
        REQUIRE(fabs(t_start_ms - 0.0) <= 100);
        REQUIRE(fabs(t_start_us - 0.0) <= (100 * 1000));
    }
    SECTION("increasing")
    {
        double t_now_ms = wall_time.millis();
        double t_now_us = wall_time.micros();

        REQUIRE(t_now_ms > t_start_ms);
        REQUIRE(t_now_us > t_start_us);
    }
}
