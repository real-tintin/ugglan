#include <catch/catch.hpp>

#include <motor_control.hpp>

TEST_CASE("body_to_motor_controls")
{
    BodyControl body_controls;
    MotorControl exp_motor_controls;
    MotorControl act_motor_controls;

    SECTION("all zero")
    {
        body_controls = BodyControl{0.0};
        exp_motor_controls = MotorControl{0, 0, 0, 0};

        REQUIRE(exp_motor_controls == body_to_motor_controls(body_controls));
    }
    SECTION("only positive fz")
    {
        body_controls = BodyControl{1.0, 0.0, 0.0, 0.0};
        exp_motor_controls = MotorControl{0, 0, 0, 0};

        REQUIRE(exp_motor_controls == body_to_motor_controls(body_controls));
    }
    SECTION("only negative fz")
    {
        body_controls = BodyControl{-4.0, 0.0, 0.0, 0.0};
        exp_motor_controls = MotorControl{10027, 10027, 10027, 10027}; // sqrt(119474) * 57 - 9675 / 4

        REQUIRE(exp_motor_controls == body_to_motor_controls(body_controls));
    }
    SECTION("positive roll")
    {
        body_controls = BodyControl{-4.0, 0.1, 0.0, 0.0};
        act_motor_controls = body_to_motor_controls(body_controls);

        REQUIRE(act_motor_controls[0] == act_motor_controls[1]);
        REQUIRE(act_motor_controls[2] == act_motor_controls[3]);

        REQUIRE(act_motor_controls[2] > act_motor_controls[0]);
        REQUIRE(act_motor_controls[3] > act_motor_controls[1]);
    }
    SECTION("positive pitch")
    {
        body_controls = BodyControl{-4.0, 0.0, 0.1, 0.0};
        act_motor_controls = body_to_motor_controls(body_controls);

        REQUIRE(act_motor_controls[0] == act_motor_controls[3]);
        REQUIRE(act_motor_controls[1] == act_motor_controls[2]);

        REQUIRE(act_motor_controls[0] > act_motor_controls[1]);
        REQUIRE(act_motor_controls[3] > act_motor_controls[2]);
    }
    SECTION("positive yaw")
    {
        body_controls = BodyControl{-4.0, 0.0, 0.0, 0.1};
        act_motor_controls = body_to_motor_controls(body_controls);

        REQUIRE(act_motor_controls[0] == act_motor_controls[2]);
        REQUIRE(act_motor_controls[1] == act_motor_controls[3]);

        REQUIRE(act_motor_controls[1] > act_motor_controls[0]);
        REQUIRE(act_motor_controls[3] > act_motor_controls[2]);
    }
}
