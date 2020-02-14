#include "catch.h"

#include "add.h"

TEST_CASE("Add", "uint8")
{
    REQUIRE(add_uint8(0, 1) == 1);
    REQUIRE(add_uint8(1, 255) == 0);
}
