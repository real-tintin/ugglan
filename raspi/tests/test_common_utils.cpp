#include <string>
extern "C"
{
#include <math.h>
}

#include <catch/catch.hpp>

#include <catch_utils.hpp>
#include <common_utils.hpp>

using namespace common_utils;

static const double FLOAT_TOL = 1e-3;

TEST_CASE("get_env")
{
    SECTION("existing env")
    {
        catch_utils::set_env("CATCH_TEST_GET_ENV", "whats_up");

        std::string value = get_env("CATCH_TEST_GET_ENV");
        REQUIRE(value == "whats_up");
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
        catch_utils::set_env("CATCH_READ_AND_CAST_ENV", "whats_up");

        read_and_cast_env(env, "CATCH_READ_AND_CAST_ENV");
        REQUIRE(env == "whats_up");
    }
    SECTION("double")
    {
        double env;
        catch_utils::set_env("CATCH_READ_AND_CAST_ENV", "3.14");

        read_and_cast_env(env, "CATCH_READ_AND_CAST_ENV");
        REQUIRE(fabs(env - 3.14) <= FLOAT_TOL);
    }
    SECTION("uint8_t")
    {
        uint8_t env;
        catch_utils::set_env("CATCH_READ_AND_CAST_ENV", "255");

        read_and_cast_env(env, "CATCH_READ_AND_CAST_ENV");
        REQUIRE(env == 255);
    }
    SECTION("uint32_t")
    {
        uint32_t env;
        catch_utils::set_env("CATCH_READ_AND_CAST_ENV", "4294967295");

        read_and_cast_env(env, "CATCH_READ_AND_CAST_ENV");
        REQUIRE(env == 4294967295);
    }
}

TEST_CASE("byte_to_hex_str")
{
    REQUIRE(byte_to_hex_str(0) == "0x00");
    REQUIRE(byte_to_hex_str(10) == "0x0A");
    REQUIRE(byte_to_hex_str(255) == "0xFF");
}

TEST_CASE("byte_to_bit_str")
{
    REQUIRE(byte_to_bit_str(0) == "0b00000000");
    REQUIRE(byte_to_bit_str(10) == "0b00001010");
    REQUIRE(byte_to_bit_str(255) == "0b11111111");
}

TEST_CASE("pack_and_unpack_base64_gzip")
{
    std::string exp_unpacked = "please pack me";

    std::string packed = pack_gzip_base64(exp_unpacked);
    std::string act_unpacked = unpack_base64_gzip(packed);

    REQUIRE(exp_unpacked == act_unpacked);
}
