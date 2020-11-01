#include <catch.h>

#include <math.h>
#include <attitude_estimation.h>

static const double FLOAT_TOL = 1e-1;
static const double SAMPLE_RATE_S = 1.0;

static const uint8_t EXEC_UNTIL_CONVERGENCE = ATT_EST_N_SAMPLES_GYRO_OFFSET_COMP + 10;

void execute_n_samples(
    AttitudeEstimation &att_est,
    AttEstInput &input,
    uint8_t n_samples)
{
    for (uint8_t i_sample = 0; i_sample < n_samples; i_sample++) { att_est.update(input); }
}

void add_gyro_offset(AttEstInput &input)
{
    input.ang_rate_x = 1.123;
    input.ang_rate_y = -10.1;
    input.ang_rate_z = 2.45;
}

TEST_CASE("attitude estimation")
{
    AttitudeEstimation att_est(SAMPLE_RATE_S);
    AttEstInput input = {0};
    AttEstimates est;

    add_gyro_offset(input);

    SECTION("calibration")
    {
        REQUIRE(att_est.is_calibrated() == false);
        execute_n_samples(att_est, input, ATT_EST_N_SAMPLES_GYRO_OFFSET_COMP);
        REQUIRE(att_est.is_calibrated() == true);

        est = att_est.get_estimates();
        REQUIRE(fabs(est.ang_rate_x - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(est.ang_rate_y - 0.0) <= FLOAT_TOL);
        REQUIRE(fabs(est.ang_rate_z - 0.0) <= FLOAT_TOL);
    }

    SECTION("roll: 45 deg")
    {
        input.acc_y = -sin(M_PI / 4);
        input.acc_z = -cos(M_PI / 4);

        execute_n_samples(att_est, input, EXEC_UNTIL_CONVERGENCE);
        est = att_est.get_estimates();

        REQUIRE(fabs(est.roll - (M_PI / 4)) <= FLOAT_TOL);
    }

    SECTION("roll: +/-180 deg")
    {
        input.acc_y = 0.0;
        input.acc_z = 1.0;

        execute_n_samples(att_est, input, EXEC_UNTIL_CONVERGENCE);
        est = att_est.get_estimates();

        REQUIRE(fabs(fabs(est.roll) - (M_PI)) <= FLOAT_TOL);
    }

    SECTION("pitch: -45 deg")
    {
        input.acc_x = sin(-M_PI / 4);
        input.acc_z = -cos(-M_PI / 4);

        execute_n_samples(att_est, input, EXEC_UNTIL_CONVERGENCE);
        est = att_est.get_estimates();

        REQUIRE(fabs(est.pitch - (-M_PI / 4)) <= FLOAT_TOL);
    }

    SECTION("pitch: +/-90 deg")
    {
        input.acc_x = 1.0;
        input.acc_z = 0.0;

        execute_n_samples(att_est, input, EXEC_UNTIL_CONVERGENCE);
        est = att_est.get_estimates();

        REQUIRE(fabs(fabs(est.pitch) - (M_PI / 2)) <= FLOAT_TOL);
    }

    SECTION("yaw: 45 deg")
    {
        input.mag_field_x = 1.0;
        input.mag_field_y = -1.0;

        execute_n_samples(att_est, input, EXEC_UNTIL_CONVERGENCE);
        est = att_est.get_estimates();

        REQUIRE(fabs(est.yaw - (M_PI / 4)) <= FLOAT_TOL);
    }

    SECTION("yaw: +/-180 deg")
    {
        input.mag_field_x = -1.0;
        input.mag_field_y = 0.0;

        execute_n_samples(att_est, input, EXEC_UNTIL_CONVERGENCE);
        est = att_est.get_estimates();

        REQUIRE(fabs(fabs(est.yaw) - (M_PI)) <= FLOAT_TOL);
    }
}
