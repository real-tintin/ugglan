#include <catch.h>

#include <thread>
#include <wall_time.h>

TEST_CASE("wall time")
{
    uint32_t t_start_ms = wall_time.millis();
    uint32_t t_start_us = wall_time.micros();

    SECTION("init not too large")
    {
        REQUIRE((t_start_ms - 0.0) <= (60 * 1000));
        REQUIRE((t_start_us - 0.0) <= (60 * 1000 * 1000));
    }
    SECTION("monotonically increasing")
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1U));

        uint32_t t_now_ms = wall_time.millis();
        uint32_t t_now_us = wall_time.micros();

        REQUIRE(t_now_ms > t_start_ms);
        REQUIRE(t_now_us > t_start_us);
    }
}
