#include <catch.h>

#include <math.h>
#include <attitude_estimation.h>

static const double FLOAT_TOL = 1e-1;
static const double SAMPLE_RATE_S = 1.0;

static const AttEstConfig CONFIG = {
    .n_samples_gyro_offset_comp = 10,
    .rolling_var_window_size = 10,

    .kalman_q_scale = 1,

    .kalman_r_0_scale = 1,
    .kalman_r_1_scale = 1,

    .hard_iron_offset_x = 0.1,
    .hard_iron_offset_y = -0.1,
    .hard_iron_offset_z = 3.1
};

static const uint32_t EXEC_UNTIL_CALIB = CONFIG.n_samples_gyro_offset_comp + CONFIG.rolling_var_window_size;

static const uint32_t EXEC_UNTIL_CONVERGENCE = 10 * EXEC_UNTIL_CALIB;

void execute_n_samples(AttitudeEstimation &att, AttEstInput &input, uint32_t n_samples)
{
    for (uint32_t i_sample = 0; i_sample < n_samples; i_sample++) { att.update(input); }
}

void add_gyro_offset(AttEstInput &input)
{
    input.ang_rate_x = 1.123;
    input.ang_rate_y = -10.1;
    input.ang_rate_z = 2.45;
}

TEST_CASE("attitude estimation")
{
    AttitudeEstimation att(SAMPLE_RATE_S, CONFIG);
    AttEstInput input = {0};
    AttEstimate est;

    add_gyro_offset(input);

    SECTION("standstill calibration")
    {
        input.acc_z = -1;

        input.mag_field_x = CONFIG.hard_iron_offset_x;
        input.mag_field_y = CONFIG.hard_iron_offset_y;
        input.mag_field_z = CONFIG.hard_iron_offset_z;

        REQUIRE(att.is_calibrated() == false);
        REQUIRE(att.is_standstill() == false);

        execute_n_samples(att, input, EXEC_UNTIL_CALIB);

        REQUIRE(att.is_calibrated() == true);
        REQUIRE(att.is_standstill() == true);

        est = att.get_estimate();

        REQUIRE(fabs(est.roll.rate - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(est.pitch.rate - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(est.yaw.rate - 0.0) <= FLOAT_TOL);
    }

    SECTION("roll: 45 deg")
    {
        input.acc_y = -sin(M_PI / 4);
        input.acc_z = -cos(M_PI / 4);

        execute_n_samples(att, input, EXEC_UNTIL_CONVERGENCE);
        est = att.get_estimate();

        REQUIRE(fabs(est.roll.angle - (M_PI / 4)) <= FLOAT_TOL);
    }

    SECTION("roll: +/-180 deg")
    {
        input.acc_y = 0.0;
        input.acc_z = 1.0;

        execute_n_samples(att, input, EXEC_UNTIL_CONVERGENCE);
        est = att.get_estimate();

        REQUIRE(fabs(fabs(est.roll.angle) - (M_PI)) <= FLOAT_TOL);
    }

    SECTION("pitch: -45 deg")
    {
        input.acc_x = sin(-M_PI / 4);
        input.acc_z = -cos(-M_PI / 4);

        execute_n_samples(att, input, EXEC_UNTIL_CONVERGENCE);
        est = att.get_estimate();

        REQUIRE(fabs(est.pitch.angle - (-M_PI / 4)) <= FLOAT_TOL);
    }

    SECTION("pitch: +/-90 deg")
    {
        input.acc_x = 1.0;
        input.acc_z = 0.0;

        execute_n_samples(att, input, EXEC_UNTIL_CONVERGENCE);
        est = att.get_estimate();

        REQUIRE(fabs(fabs(est.pitch.angle) - (M_PI / 2)) <= FLOAT_TOL);
    }

    SECTION("yaw: 45 deg")
    {
        input.mag_field_x = 1.0 + CONFIG.hard_iron_offset_x;
        input.mag_field_y = 1.0 + CONFIG.hard_iron_offset_y;

        execute_n_samples(att, input, EXEC_UNTIL_CONVERGENCE);
        est = att.get_estimate();

        REQUIRE(fabs(est.yaw.angle - (M_PI / 4)) <= FLOAT_TOL);
    }

    SECTION("yaw: +/-180 deg")
    {
        input.mag_field_x = -1.0 + CONFIG.hard_iron_offset_x;
        input.mag_field_y = 0.0 + CONFIG.hard_iron_offset_y;

        execute_n_samples(att, input, EXEC_UNTIL_CONVERGENCE);
        est = att.get_estimate();

        REQUIRE(fabs(fabs(est.yaw.angle) - (M_PI)) <= FLOAT_TOL);
    }
}
