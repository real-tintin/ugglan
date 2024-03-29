#include <thread>

#include <catch/catch.hpp>

#include <wall_time.hpp>

TEST_CASE("wall time")
{
    uint32_t t_start_ms = WallTime::millis();
    uint32_t t_start_us = WallTime::micros();

    SECTION("init not too large")
    {
        REQUIRE((t_start_ms - 0.0) <= (60 * 1000));
        REQUIRE((t_start_us - 0.0) <= (60 * 1000 * 1000));
    }
    SECTION("monotonically increasing")
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1U));

        uint32_t t_now_ms = WallTime::millis();
        uint32_t t_now_us = WallTime::micros();

        REQUIRE(t_now_ms > t_start_ms);
        REQUIRE(t_now_us > t_start_us);
    }
}
