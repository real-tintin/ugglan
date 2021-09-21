#include <catch.h>

#include <array>
#include <statistics.h>

using namespace statistics;

static const double FLOAT_TOL = 1e-9;
static const uint32_t WINDOW_SIZE = 20;

void _fill_with_and_expect_mean_and_variance(
    RollingStats& rolling_stats,
    std::array<double, WINDOW_SIZE> samples,
    double exp_mean,
    double exp_variance)
{
    for (uint32_t i_sample = 0; i_sample < WINDOW_SIZE; i_sample++)
    {
        rolling_stats.update(samples[i_sample]);
    }

    REQUIRE(fabs(rolling_stats.get_mean() - exp_mean) < FLOAT_TOL);
    REQUIRE(fabs(rolling_stats.get_variance() - exp_variance) < FLOAT_TOL);
}

TEST_CASE("RollingMeanVariance")
{
    RollingStats rolling_stats(WINDOW_SIZE);

    SECTION("estimates available")
    {
        for (uint32_t i_call = 0; i_call < (WINDOW_SIZE - 1); i_call++)
        {
            rolling_stats.update(i_call);
            REQUIRE_FALSE(rolling_stats.estimates_available());
        }

        rolling_stats.update(3.14);
        REQUIRE(rolling_stats.estimates_available());
    }

    SECTION("mean & variance")
    {
        _fill_with_and_expect_mean_and_variance(
            rolling_stats,
            {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19},
            9.5,
            33.25);

        _fill_with_and_expect_mean_and_variance(
            rolling_stats,
            {-1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1},
            0.0,
            1.0);

        _fill_with_and_expect_mean_and_variance(
            rolling_stats,
            {0.1, 0.2, 1.1, 10, -0.5, 1, -1, 1e-5, -1, 1e-5, -1, 1e-5, -1, 1, -1, 1, -7, 1, -9, 3e-5},
            -0.3049970000000001,
            11.932476830051);
    }

    SECTION("get estimates if not available")
    {
        REQUIRE_THROWS_WITH(rolling_stats.get_mean(), "Estimates not available yet");
        REQUIRE_THROWS_WITH(rolling_stats.get_variance(), "Estimates not available yet");
    }
}
