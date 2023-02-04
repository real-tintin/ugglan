#ifndef ATTITUDE_ESTIMATION_H
#define ATTITUDE_ESTIMATION_H

#include <cstdint>
#include <algorithm>
#include <math.h>
#include <statistics.h>
#include <eigen/Eigen>

struct AttEstInput {
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

struct AttEstKalmanState {
    Eigen::Vector3d x {{0}, {0}, {0}};
    Eigen::Vector2d z {{0}, {0}};
    Eigen::Matrix3d P {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    Eigen::Matrix2d R {{0, 0}, {0, 0}};
};

struct AttEstState {
    double angle; // [rad]
    double rate;  // [rad/s]
    double acc;   // [rad/s^2]
};

struct AttEstimate {
    AttEstState roll;
    AttEstState pitch;
    AttEstState yaw;
};

struct AttEstConfig {
    uint8_t n_samples_gyro_offset_comp;
    uint32_t rolling_var_window_size;

    double kalman_q_scale;

    double kalman_r_0_scale;
    double kalman_r_1_scale;

    double hard_iron_offset_x;
    double hard_iron_offset_y;
    double hard_iron_offset_z;
};

class AttitudeEstimation
{
public:
    AttitudeEstimation(double input_sample_rate_s, AttEstConfig config);

    void update(AttEstInput input);

    AttEstimate get_estimate();

    bool is_calibrated();
    bool is_standstill();
private:
    const double _dt;
    const AttEstConfig _config;

    AttEstInput _in = {0};
    AttEstimate _est = {0};

    bool _is_standstill = false;

    double _gyro_offset_x = 0;
    double _gyro_offset_y = 0;
    double _gyro_offset_z = 0;

    double _imu_roll_angle = 0;
    double _imu_pitch_angle = 0;
    double _imu_yaw_angle = 0;

    bool _is_gyro_offset_comp = false;
    uint8_t _samples_gyro_offset_comp = 0;

    Eigen::Matrix<double, 2, 3> _H {{1, 0, 0}, {0, 1, 0}};
    Eigen::Matrix<double, 3, 2> _H_t = _H.transpose();

    Eigen::Matrix3d _I {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    Eigen::Matrix3d _Q;

    Eigen::Matrix3d _F;
    Eigen::Matrix3d _F_t;

    Eigen::Vector3d _x_pri;
    Eigen::Matrix3d _P_pri;

    Eigen::Matrix2d _S;
    Eigen::Matrix<double, 3, 2> _K;

    AttEstKalmanState _kalman_roll;
    AttEstKalmanState _kalman_pitch;
    AttEstKalmanState _kalman_yaw;

    statistics::RollingStats _rolling_stats_roll_angle;
    statistics::RollingStats _rolling_stats_pitch_angle;
    statistics::RollingStats _rolling_stats_yaw_angle;

    statistics::RollingStats _rolling_stats_roll_rate;
    statistics::RollingStats _rolling_stats_pitch_rate;
    statistics::RollingStats _rolling_stats_yaw_rate;

    void _update_imu_angles();
    void _update_rolling_stats();
    void _update_standstill_status();

    void _gyro_offset_comp();
    void _hard_iron_offset_comp();

    void _update_roll();
    void _update_pitch();
    void _update_yaw();

    void _update_est(double z_0, double z_1,
                     double r_0, double r_1,
                     AttEstKalmanState& kalman_state,
                     AttEstState& att_state,
                     double modulo_lim);

    void _update_kalman_state(AttEstKalmanState& state);
    void _kalman_state_to_att_state(AttEstKalmanState& kalman, AttEstState& att);
    void _modulo_angle(double* angle, double limit);
};

#endif /* ATTITUDE_ESTIMATION_H */
