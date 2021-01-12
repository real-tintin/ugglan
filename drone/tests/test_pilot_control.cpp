#include <catch.h>

#include <math.h>
#include <pilot_control.h>
#include <logger.h>
static const double FLOAT_TOL = 1e-2;
static const double SAMPLE_RATE_S = 0.02;

void _update_n_times(PilotControl pilot_ctrl, AttEstimate& att_est,
                NormalizedRef norm_ref, BodyControl& body_ctrl,
                uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) { body_ctrl = pilot_ctrl.update(att_est, norm_ref); }
}

TEST_CASE("pilot control")
{
    PilotControl pilot_ctrl(SAMPLE_RATE_S);

    AttEstimate att_est = {0};
    NormalizedRef norm_ref = {0};
    BodyControl body_ctrl = {0};

    SECTION("initialized")
    {
        body_ctrl = pilot_ctrl.update(att_est, norm_ref);

        REQUIRE(fabs(body_ctrl.m_x - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(body_ctrl.m_y - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(body_ctrl.m_z - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(body_ctrl.f_z - 0.0) <= FLOAT_TOL);
    }
    SECTION("ctrl in fz")
    {
        norm_ref.f_z = 1.0;
        body_ctrl = pilot_ctrl.update(att_est, norm_ref);

        REQUIRE(fabs(body_ctrl.m_x - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(body_ctrl.m_y - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(body_ctrl.m_z - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(body_ctrl.f_z - PILOT_CTRL_SCALE_REF_F_Z) <= FLOAT_TOL);
    }
    SECTION("ctrl in mx")
    {
        norm_ref.roll = 1.0;
        _update_n_times(pilot_ctrl, att_est, norm_ref, body_ctrl, 10);

        REQUIRE(body_ctrl.m_x > 0.1);
        REQUIRE(fabs(body_ctrl.m_y - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(body_ctrl.m_z - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(body_ctrl.f_z - 0.0) <= FLOAT_TOL);
    }
    // TODO: Test anti-windup
}
