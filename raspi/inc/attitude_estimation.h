#ifndef ATTITUDE_ESTIMATION_H
#define ATTITUDE_ESTIMATION_H

#include <cstdint>
#include <algorithm>
#include <math.h>
#include <statistics.h>
#include <eigen/Eigen>

namespace att_est
{
struct Imu {
    double acc_x;       // [m/s^2]
    double acc_y;       // [m/s^2]
    double acc_z;       // [m/s^2]

    double ang_rate_x;  // [rad/s]
    double ang_rate_y;  // [rad/s]
    double ang_rate_z;  // [rad/s]

    double mag_field_x; // [gauss]
    double mag_field_y; // [gauss]
    double mag_field_z; // [gauss]
};

struct KalmanState {
    Eigen::Vector3d x {{0}, {0}, {0}};
    Eigen::Vector2d z {{0}, {0}};
    Eigen::Matrix3d P {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
};

struct AttState {
    double angle; // [rad]
    double rate;  // [rad/s]
    double acc;   // [rad/s^2]
};

struct Attitude {
    AttState roll;
    AttState pitch;
    AttState yaw;
};

struct Config {
    uint8_t n_samples_gyro_bias_compensation;
    uint32_t rolling_var_window_size;

    double kalman_q_scale;

    double kalman_r_0;
    double kalman_r_1;

    double acc_error_s_x;
    double acc_error_s_y;
    double acc_error_s_z;

    double acc_error_m_x_y;
    double acc_error_m_x_z;
    double acc_error_m_y_x;
    double acc_error_m_y_z;
    double acc_error_m_z_x;
    double acc_error_m_z_y;

    double acc_error_b_x;
    double acc_error_b_y;
    double acc_error_b_z;

    double hard_iron_bias_x;
    double hard_iron_bias_y;
    double hard_iron_bias_z;
};

class Estimator
{
public:
    Estimator(double input_sample_rate_s, Config config);

    void update(Imu imu_uncompensated);

    Attitude get_attitude();
    Imu get_imu_compensated();

    bool is_calibrated();
    bool is_standstill();
private:
    const double _dt;
    const Config _config;

    Imu _imu_uncompensated = {0};
    Imu _imu_compensated = {0};

    Attitude _attitude = {0};

    bool _is_standstill = false;

    double _gyro_bias_x = 0;
    double _gyro_bias_y = 0;
    double _gyro_bias_z = 0;

    double _imu_roll_angle = 0;
    double _imu_pitch_angle = 0;
    double _imu_yaw_angle = 0;

    bool _is_gyro_bias_compensated = false;
    uint8_t _n_samples_gyro_bias_compensated = 0;

    Eigen::Matrix<double, 2, 3> _H {{1, 0, 0}, {0, 1, 0}};
    Eigen::Matrix<double, 3, 2> _H_t = _H.transpose();

    Eigen::Matrix3d _I {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    Eigen::Matrix3d _Q;
    Eigen::Matrix2d _R;

    Eigen::Matrix3d _F;
    Eigen::Matrix3d _F_t;

    Eigen::Vector3d _x_pri;
    Eigen::Matrix3d _P_pri;

    Eigen::Matrix2d _S;
    Eigen::Matrix<double, 3, 2> _K;

    KalmanState _kalman_roll;
    KalmanState _kalman_pitch;
    KalmanState _kalman_yaw;

    statistics::RollingStats _rolling_stats_roll_angle;
    statistics::RollingStats _rolling_stats_pitch_angle;
    statistics::RollingStats _rolling_stats_yaw_angle;

    statistics::RollingStats _rolling_stats_roll_rate;
    statistics::RollingStats _rolling_stats_pitch_rate;
    statistics::RollingStats _rolling_stats_yaw_rate;

    void _update_imu_angles();
    void _update_rolling_stats();
    void _update_standstill_status();

    void _acc_static_compensation();
    void _gyro_dynamic_bias_compensation();
    void _hard_iron_bias_compensation();

    void _update_roll();
    void _update_pitch();
    void _update_yaw();

    void _update_att_state(double z_0, double z_1,
                           KalmanState& kalman_state,
                           AttState& att_state,
                           double modulo_lim);

    void _update_kalman_state(KalmanState& state);
    void _kalman_state_to_att_state(KalmanState& kalman, AttState& att);
    void _modulo_angle(double* angle, double limit);
};
} /* namespace att_est */

#endif /* ATTITUDE_ESTIMATION_H */
