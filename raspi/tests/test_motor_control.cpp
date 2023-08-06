#include <catch/catch.hpp>

#include <motor_control.hpp>

void assert_motor_controls_are_positive(MotorControl &motor_controls)
{
    REQUIRE(motor_controls[0] > 0);
    REQUIRE(motor_controls[1] > 0);
    REQUIRE(motor_controls[2] > 0);
    REQUIRE(motor_controls[3] > 0);
}

TEST_CASE("body_to_motor_controls")
{
    BodyControl body_controls;
    MotorControl motor_controls;

    SECTION("all zero")
    {
        body_controls = BodyControl{0.0};
        motor_controls = body_to_motor_controls(body_controls);

        REQUIRE(motor_controls[0] == 0);
        REQUIRE(motor_controls[1] == 0);
        REQUIRE(motor_controls[2] == 0);
        REQUIRE(motor_controls[3] == 0);
    }
    SECTION("only positive fz")
    {
        body_controls = BodyControl{1.0, 0.0, 0.0, 0.0};
        motor_controls = body_to_motor_controls(body_controls);

        REQUIRE(motor_controls[0] == 0);
        REQUIRE(motor_controls[1] == 0);
        REQUIRE(motor_controls[2] == 0);
        REQUIRE(motor_controls[3] == 0);
    }
    SECTION("only negative fz")
    {
        body_controls = BodyControl{-10.0, 0.0, 0.0, 0.0};
        motor_controls = body_to_motor_controls(body_controls);

        assert_motor_controls_are_positive(motor_controls);

        REQUIRE(motor_controls[0] == motor_controls[1]);
        REQUIRE(motor_controls[1] == motor_controls[2]);
        REQUIRE(motor_controls[2] == motor_controls[3]);
    }
    SECTION("positive roll")
    {
        body_controls = BodyControl{-10.0, 0.1, 0.0, 0.0};
        motor_controls = body_to_motor_controls(body_controls);

        assert_motor_controls_are_positive(motor_controls);

        REQUIRE(motor_controls[0] == motor_controls[1]);
        REQUIRE(motor_controls[2] == motor_controls[3]);
        REQUIRE(motor_controls[2] > motor_controls[0]);
        REQUIRE(motor_controls[3] > motor_controls[1]);
    }
    SECTION("positive pitch")
    {
        body_controls = BodyControl{-10.0, 0.0, 0.1, 0.0};
        motor_controls = body_to_motor_controls(body_controls);

        assert_motor_controls_are_positive(motor_controls);

        REQUIRE(motor_controls[0] == motor_controls[3]);
        REQUIRE(motor_controls[1] == motor_controls[2]);
        REQUIRE(motor_controls[0] > motor_controls[1]);
        REQUIRE(motor_controls[3] > motor_controls[2]);
    }
    SECTION("positive yaw")
    {
        body_controls = BodyControl{-10.0, 0.0, 0.0, 0.1};
        motor_controls = body_to_motor_controls(body_controls);

        assert_motor_controls_are_positive(motor_controls);

        REQUIRE(motor_controls[0] == motor_controls[2]);
        REQUIRE(motor_controls[1] == motor_controls[3]);
        REQUIRE(motor_controls[1] > motor_controls[0]);
        REQUIRE(motor_controls[3] > motor_controls[2]);
    }
}
