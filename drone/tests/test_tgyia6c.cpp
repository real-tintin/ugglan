#include <catch.h>

#include <serial_conn_stub.h>
#include <tgyia6c.h>

static const double FLOAT_TOL = 1e-4;

TEST_CASE("tgyia6c initialized")
{
    SerialConn serial_conn = SerialConn();
    Tgyia6c rc_receiver = Tgyia6c(&serial_conn);

    SECTION("status")
    {
        REQUIRE(rc_receiver.get_status() == TGYIA6C_STATUS_OK);
    }
    SECTION("init values")
    {
        REQUIRE(fabs(rc_receiver.get_gimbal_left_x() - 0.5) <= FLOAT_TOL);
        REQUIRE(fabs(rc_receiver.get_gimbal_left_y() - 0.0) <= FLOAT_TOL);

        REQUIRE(fabs(rc_receiver.get_gimbal_right_x() - 0.5) <= FLOAT_TOL);
        REQUIRE(fabs(rc_receiver.get_gimbal_right_y() - 0.5) <= FLOAT_TOL);

        REQUIRE(fabs(rc_receiver.get_knob() - 0.0) <= FLOAT_TOL);

        REQUIRE(rc_receiver.get_switch_left() == SwitchLr::High);
        REQUIRE(rc_receiver.get_switch_right() == SwitchLr::High);
        REQUIRE(rc_receiver.get_switch_middle() == SwitchM::High);
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
