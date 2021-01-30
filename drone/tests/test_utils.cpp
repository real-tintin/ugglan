#include <catch.h>

#include <string>
#include <utils.h>

using namespace utils;

static const std::string VALID_ENV = "CATCH_TEST_GETENV";
static const std::string VALID_ENV_EXP = "test_getenv";

TEST_CASE("get_env_str")
{
    SECTION("invalid env")
    {
        std::string env = get_env_str("INVALID_ENV");
        REQUIRE(env.size() == 0);
    }
    SECTION("valid env")
    {
        std::string env = get_env_str(VALID_ENV);
        REQUIRE(env.compare(VALID_ENV_EXP) == 0);
    }
}
