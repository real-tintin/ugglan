#include <catch.h>

#include <string>
#include <math.h>
#include <catch_utils.h>
#include <utils.h>

using namespace utils;

static const double FLOAT_TOL = 1e-3;

TEST_CASE("get_env")
{
    SECTION("non existing env")
    {
        std::string default_val = "not_so_fancy";

        std::string env = get_env("NON_EXISTING_CATCH_ENV", default_val);
        REQUIRE(env.compare(default_val) == 0);
    }
    SECTION("string")
    {
        catchutils::set_env("CATCH_TEST_GETENV", "whats_up");

        std::string env = get_env("CATCH_TEST_GETENV", std::string(""));
        REQUIRE(env.compare("whats_up") == 0);
    }
    SECTION("double")
    {
        catchutils::set_env("CATCH_TEST_GETENV", "3.14");

        double env = get_env("CATCH_TEST_GETENV", double(0.0));
        REQUIRE(fabs(env - 3.14) <= FLOAT_TOL);
    }
}
