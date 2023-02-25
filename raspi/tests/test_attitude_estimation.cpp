#include <catch.h>

#include <math.h>
#include <attitude_estimation.h>

static const double ATTITUDE_TOL = 1e-5;
static const double INPUT_SAMPLE_RATE_S = 1.0;

static const att_est::Config CONFIG_ZERO_SENSOR_ERROR = {
    .n_samples_gyro_bias_compensation = 10,
    .rolling_var_window_size = 10,

    .kalman_q_scale = 1.0,

    .kalman_r_0 = 1.0,
    .kalman_r_1 = 1.0,

    .acc_error_s_x = 1.0,
    .acc_error_s_y = 1.0,
    .acc_error_s_z = 1.0,

    .acc_error_m_x_y = 0.0,
    .acc_error_m_x_z = 0.0,
    .acc_error_m_y_x = 0.0,
    .acc_error_m_y_z = 0.0,
    .acc_error_m_z_x = 0.0,
    .acc_error_m_z_y = 0.0,

    .acc_error_b_x = 0.0,
    .acc_error_b_y = 0.0,
    .acc_error_b_z = 0.0,

    .hard_iron_bias_x = 0.0,
    .hard_iron_bias_y = 0.0,
    .hard_iron_bias_z = 0.0
};

static const uint32_t EXEC_UNTIL_CALIB = CONFIG_ZERO_SENSOR_ERROR.n_samples_gyro_bias_compensation +
    CONFIG_ZERO_SENSOR_ERROR.rolling_var_window_size;

static const uint32_t EXEC_UNTIL_CONVERGENCE = 10 * EXEC_UNTIL_CALIB;

void execute_n_samples(att_est::Estimator &estimator, att_est::Imu &imu_uncompensated, uint32_t n_samples)
{
    for (uint32_t i_sample = 0; i_sample < n_samples; i_sample++) { estimator.update(imu_uncompensated); }
}

TEST_CASE("standstill calibration")
{
    att_est::Estimator estimator(INPUT_SAMPLE_RATE_S, CONFIG_ZERO_SENSOR_ERROR);
    att_est::Imu imu_uncompensated = {0};
    att_est::Attitude attitude;

    imu_uncompensated.acc_z = -1;

    imu_uncompensated.ang_rate_x = 1.123;
    imu_uncompensated.ang_rate_y = -10.1;
    imu_uncompensated.ang_rate_z = 2.45;

    REQUIRE(estimator.is_calibrated() == false);
    REQUIRE(estimator.is_standstill() == false);

    execute_n_samples(estimator, imu_uncompensated, EXEC_UNTIL_CALIB);

    REQUIRE(estimator.is_calibrated() == true);
    REQUIRE(estimator.is_standstill() == true);

    attitude = estimator.get_attitude();

    REQUIRE(fabs(attitude.roll.rate - 0.0) <= ATTITUDE_TOL);
    REQUIRE(fabs(attitude.pitch.rate - 0.0) <= ATTITUDE_TOL);
    REQUIRE(fabs(attitude.yaw.rate - 0.0) <= ATTITUDE_TOL);
}

TEST_CASE("attitude estimation")
{
    att_est::Estimator estimator(INPUT_SAMPLE_RATE_S, CONFIG_ZERO_SENSOR_ERROR);
    att_est::Imu imu_uncompensated = {0};
    att_est::Attitude attitude;

    SECTION("roll: 45 deg")
    {
        imu_uncompensated.acc_y = -sin(M_PI / 4);
        imu_uncompensated.acc_z = -cos(M_PI / 4);

        execute_n_samples(estimator, imu_uncompensated, EXEC_UNTIL_CONVERGENCE);
        attitude = estimator.get_attitude();

        REQUIRE(fabs(attitude.roll.angle - (M_PI / 4)) <= ATTITUDE_TOL);
    }

    SECTION("roll: +/-180 deg")
    {
        imu_uncompensated.acc_y = 0.0;
        imu_uncompensated.acc_z = 1.0;

        execute_n_samples(estimator, imu_uncompensated, EXEC_UNTIL_CONVERGENCE);
        attitude = estimator.get_attitude();

        REQUIRE(fabs(fabs(attitude.roll.angle) - (M_PI)) <= ATTITUDE_TOL);
    }

    SECTION("pitch: -45 deg")
    {
        imu_uncompensated.acc_x = sin(-M_PI / 4);
        imu_uncompensated.acc_z = -cos(-M_PI / 4);

        execute_n_samples(estimator, imu_uncompensated, EXEC_UNTIL_CONVERGENCE);
        attitude = estimator.get_attitude();

        REQUIRE(fabs(attitude.pitch.angle - (-M_PI / 4)) <= ATTITUDE_TOL);
    }

    SECTION("pitch: +/-90 deg")
    {
        imu_uncompensated.acc_x = 1.0;
        imu_uncompensated.acc_z = 0.0;

        execute_n_samples(estimator, imu_uncompensated, EXEC_UNTIL_CONVERGENCE);
        attitude = estimator.get_attitude();

        REQUIRE(fabs(fabs(attitude.pitch.angle) - (M_PI / 2)) <= ATTITUDE_TOL);
    }

    SECTION("yaw: 45 deg")
    {
        imu_uncompensated.mag_field_x = 1.0;
        imu_uncompensated.mag_field_y = 1.0;

        execute_n_samples(estimator, imu_uncompensated, EXEC_UNTIL_CONVERGENCE);
        attitude = estimator.get_attitude();

        REQUIRE(fabs(attitude.yaw.angle - (M_PI / 4)) <= ATTITUDE_TOL);
    }

    SECTION("yaw: +/-180 deg")
    {
        imu_uncompensated.mag_field_x = -1.0;
        imu_uncompensated.mag_field_y = 0.0;

        execute_n_samples(estimator, imu_uncompensated, EXEC_UNTIL_CONVERGENCE);
        attitude = estimator.get_attitude();

        REQUIRE(fabs(fabs(attitude.yaw.angle) - (M_PI)) <= ATTITUDE_TOL);
    }
}

TEST_CASE("static error compensation")
{
    WHEN("an estimator is created with a non zero static sensor error config")
    {
        att_est::Config config_non_zero_sensor_error = CONFIG_ZERO_SENSOR_ERROR;

        config_non_zero_sensor_error.acc_error_s_x = 1.1;
        config_non_zero_sensor_error.acc_error_s_y = 1.0;
        config_non_zero_sensor_error.acc_error_s_z = 0.9;

        // Intentionally omitted as complexity explodes.
        config_non_zero_sensor_error.acc_error_m_x_y = 0.0;
        config_non_zero_sensor_error.acc_error_m_x_z = 0.0;
        config_non_zero_sensor_error.acc_error_m_y_x = 0.0;
        config_non_zero_sensor_error.acc_error_m_y_z = 0.0;
        config_non_zero_sensor_error.acc_error_m_z_x = 0.0;
        config_non_zero_sensor_error.acc_error_m_z_y = 0.0;

        config_non_zero_sensor_error.acc_error_b_x = -3.1;
        config_non_zero_sensor_error.acc_error_b_y = 2.2;
        config_non_zero_sensor_error.acc_error_b_z = -1.3;

        config_non_zero_sensor_error.hard_iron_bias_x = 0.1;
        config_non_zero_sensor_error.hard_iron_bias_y = -0.1;
        config_non_zero_sensor_error.hard_iron_bias_z = 3.1;

        att_est::Estimator estimator(INPUT_SAMPLE_RATE_S, config_non_zero_sensor_error);

        GIVEN("an update with imu uncompensated state corresponding one plus the static sensor error")
        {
            att_est::Imu imu_uncompensated = {
                .acc_x = (1.0 - config_non_zero_sensor_error.acc_error_b_x) / config_non_zero_sensor_error.acc_error_s_x,
                .acc_y = (1.0 - config_non_zero_sensor_error.acc_error_b_y) / config_non_zero_sensor_error.acc_error_s_y,
                .acc_z = (1.0 - config_non_zero_sensor_error.acc_error_b_z) / config_non_zero_sensor_error.acc_error_s_z,

                .ang_rate_x = 0.0, // Not tested here
                .ang_rate_y = 0.0, // Not tested here
                .ang_rate_z = 0.0, // Not tested here

                .mag_field_x = 1.0 + config_non_zero_sensor_error.hard_iron_bias_x,
                .mag_field_y = 1.0 + config_non_zero_sensor_error.hard_iron_bias_y,
                .mag_field_z = 1.0 + config_non_zero_sensor_error.hard_iron_bias_z
            };

            estimator.update(imu_uncompensated);

            THEN("the compensated imu state should be one")
            {
                att_est::Imu imu_compensated = estimator.get_imu_compensated();

                REQUIRE(fabs(imu_compensated.acc_x - 1.0) <= ATTITUDE_TOL);
                REQUIRE(fabs(imu_compensated.acc_y - 1.0) <= ATTITUDE_TOL);
                REQUIRE(fabs(imu_compensated.acc_z - 1.0) <= ATTITUDE_TOL);

                REQUIRE(fabs(imu_compensated.mag_field_x - 1.0) <= ATTITUDE_TOL);
                REQUIRE(fabs(imu_compensated.mag_field_y - 1.0) <= ATTITUDE_TOL);
                REQUIRE(fabs(imu_compensated.mag_field_z - 1.0) <= ATTITUDE_TOL);
            }
        }
    }
}
