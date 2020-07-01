#include <catch.h>

#include <serial_conn_stub.h>
#include <tgyia6c.h>

TEST_CASE("tgyia6c status")
{
    SerialConn serial_conn = SerialConn();
    Tgyia6c rc_receiver = Tgyia6c(&serial_conn);

    SECTION("initialized")
    {
        REQUIRE(rc_receiver.get_status() == TGYIA6C_STATUS_OK);
    }
}

TEST_CASE("tgyia6c parse data")
{
    SECTION("valid")
    {
        // TODO
    }
    SECTION("invalid")
    {
        // TODO
    }
}
