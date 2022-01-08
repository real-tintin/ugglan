#include <catch.h>

#include <math.h>
#include <attitude_estimation.h>

static const double FLOAT_TOL = 1e-1;
static const double SAMPLE_RATE_S = 0.01;

static const uint32_t EXEC_UNTIL_CALIB = ATT_EST_N_SAMPLES_GYRO_OFFSET_COMP + ATT_EST_ROLLING_WINDOW_SIZE;

static const uint32_t EXEC_UNTIL_CONVERGENCE = 10 * EXEC_UNTIL_CALIB;

void execute_n_samples(AttitudeEstimation &att, AttEstInput &input, uint32_t n_samples)
{
    for (uint32_t i_sample = 0; i_sample < n_samples; i_sample++) { att.update(input); }
}

void add_gyro_offset(AttEstInput &input)
{
    input.ang_rate.x = 1.123;
    input.ang_rate.y = -10.1;
    input.ang_rate.z = 2.45;
}

TEST_CASE("attitude estimation")
{
    AttitudeEstimation att(SAMPLE_RATE_S);
    AttEstInput input = {0};
    AttEstimate est;

    add_gyro_offset(input);

    SECTION("standstill calibration")
    {
        input.acc.z = -1;

        input.mag_field.x = ATT_EST_HARD_IRON_OFFSET_X;
        input.mag_field.y = ATT_EST_HARD_IRON_OFFSET_Y;
        input.mag_field.z = ATT_EST_HARD_IRON_OFFSET_Z;

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
        input.acc.y = -sin(M_PI / 4);
        input.acc.z = -cos(M_PI / 4);

        execute_n_samples(att, input, EXEC_UNTIL_CONVERGENCE);
        est = att.get_estimate();

        REQUIRE(fabs(est.roll.angle - (M_PI / 4)) <= FLOAT_TOL);
    }

    SECTION("roll: +/-180 deg")
    {
        input.acc.y = 0.0;
        input.acc.z = 1.0;

        execute_n_samples(att, input, EXEC_UNTIL_CONVERGENCE);
        est = att.get_estimate();

        REQUIRE(fabs(fabs(est.roll.angle) - (M_PI)) <= FLOAT_TOL);
    }

    SECTION("pitch: -45 deg")
    {
        input.acc.x = sin(-M_PI / 4);
        input.acc.z = -cos(-M_PI / 4);

        execute_n_samples(att, input, EXEC_UNTIL_CONVERGENCE);
        est = att.get_estimate();

        REQUIRE(fabs(est.pitch.angle - (-M_PI / 4)) <= FLOAT_TOL);
    }

    SECTION("pitch: +/-90 deg")
    {
        input.acc.x = 1.0;
        input.acc.z = 0.0;

        execute_n_samples(att, input, EXEC_UNTIL_CONVERGENCE);
        est = att.get_estimate();

        REQUIRE(fabs(fabs(est.pitch.angle) - (M_PI / 2)) <= FLOAT_TOL);
    }

    SECTION("yaw: 45 deg")
    {
        input.mag_field.x = 1.0 + ATT_EST_HARD_IRON_OFFSET_X;
        input.mag_field.y = 1.0 + ATT_EST_HARD_IRON_OFFSET_Y;

        execute_n_samples(att, input, EXEC_UNTIL_CONVERGENCE);
        est = att.get_estimate();

        REQUIRE(fabs(est.yaw.angle - (M_PI / 4)) <= FLOAT_TOL);
    }

    SECTION("yaw: +/-180 deg")
    {
        input.mag_field.x = -1.0 + ATT_EST_HARD_IRON_OFFSET_X;
        input.mag_field.y = 0.0 + ATT_EST_HARD_IRON_OFFSET_Y;

        execute_n_samples(att, input, EXEC_UNTIL_CONVERGENCE);
        est = att.get_estimate();

        REQUIRE(fabs(fabs(est.yaw.angle) - (M_PI)) <= FLOAT_TOL);
    }
}
