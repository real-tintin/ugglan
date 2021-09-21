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

TEST_CASE("byte_to_hex_str")
{
    REQUIRE(byte_to_hex_str(0).compare("0x00") == 0);
    REQUIRE(byte_to_hex_str(10).compare("0x0A") == 0);
    REQUIRE(byte_to_hex_str(255).compare("0xFF") == 0);
}

TEST_CASE("byte_to_bit_str")
{
    REQUIRE(byte_to_bit_str(0).compare("0b00000000") == 0);
    REQUIRE(byte_to_bit_str(10).compare("0b00001010") == 0);
    REQUIRE(byte_to_bit_str(255).compare("0b11111111") == 0);
}
