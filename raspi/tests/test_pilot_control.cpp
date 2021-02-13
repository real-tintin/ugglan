#include <catch.h>

#include <math.h>
#include <pilot_control.h>

static const double FLOAT_TOL = 1e-3;
static const double SAMPLE_RATE_S = 0.02;

TEST_CASE("pilot control")
{
    PilotControl pilot_ctrl(SAMPLE_RATE_S);

    AttEstimate est = {0};
    PilotCtrlRef ref = {0};
    BodyControl ctrl = {0};

    SECTION("initialized")
    {
        pilot_ctrl.update(est, ref);
        ctrl = pilot_ctrl.get_ctrl();

        REQUIRE(fabs(ctrl.m_x - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(ctrl.m_y - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(ctrl.m_z - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(ctrl.f_z - 0.0) <= FLOAT_TOL);
    }
    SECTION("ctrl in fz")
    {
        ref.f_z = -PILOT_CTRL_ABS_MAX_REF_F_Z;

        pilot_ctrl.update(est, ref);
        ctrl = pilot_ctrl.get_ctrl();

        REQUIRE(fabs(ctrl.m_x - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(ctrl.m_y - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(ctrl.m_z - 0.0) <= FLOAT_TOL);
        REQUIRE(ctrl.f_z < FLOAT_TOL);
    }
    SECTION("positive ctrl in mx")
    {
        ref.roll = PILOT_CTRL_ABS_MAX_REF_ROLL;

        pilot_ctrl.update(est, ref);
        ctrl = pilot_ctrl.get_ctrl();

        REQUIRE(ctrl.m_x > FLOAT_TOL);
        REQUIRE(fabs(ctrl.m_y - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(ctrl.m_z - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(ctrl.f_z - 0.0) <= FLOAT_TOL);
    }
    SECTION("negative ctrl in mx")
    {
        ref.roll = -PILOT_CTRL_ABS_MAX_REF_ROLL;

        pilot_ctrl.update(est, ref);
        ctrl = pilot_ctrl.get_ctrl();

        REQUIRE(ctrl.m_x < FLOAT_TOL);
        REQUIRE(fabs(ctrl.m_y - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(ctrl.m_z - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(ctrl.f_z - 0.0) <= FLOAT_TOL);
    }
    SECTION("positive ctrl in my")
    {
        ref.pitch = PILOT_CTRL_ABS_MAX_REF_PITCH;

        pilot_ctrl.update(est, ref);
        ctrl = pilot_ctrl.get_ctrl();

        REQUIRE(fabs(ctrl.m_x - 0.0) <= FLOAT_TOL);
        REQUIRE(ctrl.m_y > FLOAT_TOL);
        REQUIRE(fabs(ctrl.m_z - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(ctrl.f_z - 0.0) <= FLOAT_TOL);
    }
    SECTION("negative ctrl in my")
    {
        ref.pitch = -PILOT_CTRL_ABS_MAX_REF_PITCH;

        pilot_ctrl.update(est, ref);
        ctrl = pilot_ctrl.get_ctrl();

        REQUIRE(fabs(ctrl.m_x - 0.0) <= FLOAT_TOL);
        REQUIRE(ctrl.m_y < FLOAT_TOL);
        REQUIRE(fabs(ctrl.m_z - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(ctrl.f_z - 0.0) <= FLOAT_TOL);
    }
    SECTION("positive ctrl in mz")
    {
        ref.yaw_rate = PILOT_CTRL_ABS_MAX_REF_YAW_RATE;

        pilot_ctrl.update(est, ref);
        ctrl = pilot_ctrl.get_ctrl();

        REQUIRE(fabs(ctrl.m_x - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(ctrl.m_y - 0.0) <= FLOAT_TOL);
        REQUIRE(ctrl.m_z > FLOAT_TOL);
        REQUIRE(fabs(ctrl.f_z - 0.0) <= FLOAT_TOL);
    }
    SECTION("negative ctrl in mz")
    {
        ref.yaw_rate = -PILOT_CTRL_ABS_MAX_REF_YAW_RATE;

        pilot_ctrl.update(est, ref);
        ctrl = pilot_ctrl.get_ctrl();

        REQUIRE(fabs(ctrl.m_x - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(ctrl.m_y - 0.0) <= FLOAT_TOL);
        REQUIRE(ctrl.m_z < FLOAT_TOL);
        REQUIRE(fabs(ctrl.f_z - 0.0) <= FLOAT_TOL);
    }
    SECTION("anti-windup")
    {
        uint16_t n_updates_to_sat = 1000;

        ref.roll = PILOT_CTRL_ABS_MAX_REF_ROLL;
        ref.pitch = PILOT_CTRL_ABS_MAX_REF_PITCH;
        ref.yaw_rate = PILOT_CTRL_ABS_MAX_REF_YAW_RATE;

        for (uint16_t x = 0; x < n_updates_to_sat; x++) { pilot_ctrl.update(est, ref); }
        BodyControl saturated_ctrl = pilot_ctrl.get_ctrl();

        for (uint16_t x = 0; x < n_updates_to_sat; x++) { pilot_ctrl.update(est, ref); }
        ctrl = pilot_ctrl.get_ctrl();

        REQUIRE(fabs(ctrl.m_x - saturated_ctrl.m_x) <= FLOAT_TOL);
        REQUIRE(fabs(ctrl.m_y - saturated_ctrl.m_y) <= FLOAT_TOL);
        REQUIRE(fabs(ctrl.m_z - saturated_ctrl.m_z) <= FLOAT_TOL);
    }
}

TEST_CASE("tgyia6c_to_pilot_ctrl_ref")
{
    SECTION("zero")
    {
        PilotCtrlRef ref = tgyia6c_to_pilot_ctrl_ref(0.5, 0.0, 0.5, 0.5);

        REQUIRE(fabs(ref.roll - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(ref.pitch - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(ref.yaw_rate - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(ref.f_z - 0.0) <= FLOAT_TOL);
    }
    SECTION("not zero")
    {
        PilotCtrlRef ref = tgyia6c_to_pilot_ctrl_ref(0.0, 1.0, 0.0, 1.0);

        REQUIRE(fabs(ref.roll + PILOT_CTRL_ABS_MAX_REF_ROLL) <= FLOAT_TOL);
        REQUIRE(fabs(ref.pitch + PILOT_CTRL_ABS_MAX_REF_PITCH) <= FLOAT_TOL);
        REQUIRE(fabs(ref.yaw_rate + PILOT_CTRL_ABS_MAX_REF_YAW_RATE) <= FLOAT_TOL);
        REQUIRE(fabs(ref.f_z + PILOT_CTRL_ABS_MAX_REF_F_Z) <= FLOAT_TOL);
    }
    SECTION("range limit")
    {
        PilotCtrlRef ref = tgyia6c_to_pilot_ctrl_ref(1.1, -1.1, 1.1, -1.1);

        REQUIRE(fabs(ref.roll - PILOT_CTRL_ABS_MAX_REF_ROLL) <= FLOAT_TOL);
        REQUIRE(fabs(ref.pitch - PILOT_CTRL_ABS_MAX_REF_PITCH) <= FLOAT_TOL);
        REQUIRE(fabs(ref.yaw_rate - PILOT_CTRL_ABS_MAX_REF_YAW_RATE) <= FLOAT_TOL);
        REQUIRE(fabs(ref.f_z - 0.0) <= FLOAT_TOL);
    }
}
