#include <catch.h>

#include <string>
#include <math.h>
#include <catch_utils.h>
#include <utils.h>

using namespace utils;

static const double FLOAT_TOL = 1e-3;

TEST_CASE("get_env")
{
    SECTION("existing env")
    {
        catchutils::set_env("CATCH_TEST_GET_ENV", "whats_up");

        std::string value = get_env("CATCH_TEST_GET_ENV");
        REQUIRE(value.compare("whats_up") == 0);
    }
    SECTION("non existing env")
    {
        REQUIRE_THROWS_WITH(get_env("NON_EXISTING_GET_ENV"),
                            "Environmental variable doesn't exist: NON_EXISTING_GET_ENV");
    }
}

TEST_CASE("read_and_cast_env")
{
    SECTION("string")
    {
        std::string env;
        catchutils::set_env("CATCH_READ_AND_CAST_ENV", "whats_up");

        read_and_cast_env(env, "CATCH_READ_AND_CAST_ENV");
        REQUIRE(env.compare("whats_up") == 0);
    }
    SECTION("double")
    {
        double env;
        catchutils::set_env("CATCH_READ_AND_CAST_ENV", "3.14");

        read_and_cast_env(env, "CATCH_READ_AND_CAST_ENV");
        REQUIRE(fabs(env - 3.14) <= FLOAT_TOL);
    }
    SECTION("uint8_t")
    {
        uint8_t env;
        catchutils::set_env("CATCH_READ_AND_CAST_ENV", "255");

        read_and_cast_env(env, "CATCH_READ_AND_CAST_ENV");
        REQUIRE(env == 255);
    }
    SECTION("uint32_t")
    {
        uint32_t env;
        catchutils::set_env("CATCH_READ_AND_CAST_ENV", "4294967295");

        read_and_cast_env(env, "CATCH_READ_AND_CAST_ENV");
        REQUIRE(env == 4294967295);
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
