#include "catch_with_main.h"

#include "increment.h"

TEST_CASE("Increment", "uint8")
{
    REQUIRE(increment_uint8(0) == 1);
    REQUIRE(increment_uint8(255) == 0);
}
