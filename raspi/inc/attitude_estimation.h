#ifndef ATTITUDE_ESTIMATION_H
#define ATTITUDE_ESTIMATION_H

#include <cstdint>
#include <algorithm>
#include <math.h>
#include <utils.h>
#include <statistics.h>
#include <eigen/Eigen>
#include <iir1/Iir.h>

inline const double ATT_EST_KALMAN_Q_SCALE = utils::get_env("ATT_EST_KALMAN_Q_SCALE", 1e2);
inline const double ATT_EST_KALMAN_R_0_SCALE = utils::get_env("ATT_EST_KALMAN_R_0_SCALE", 1.0);
inline const double ATT_EST_KALMAN_R_1_SCALE = utils::get_env("ATT_EST_KALMAN_R_1_SCALE", 1.0);

inline const double ATT_EST_LP_BUTTER_ACC_CF = utils::get_env("ATT_EST_LP_BUTTER_ACC_CF", 2.0);
inline const int ATT_EST_LP_BUTTER_ACC_ORDER = 3;

inline const uint8_t ATT_EST_N_SAMPLES_GYRO_OFFSET_COMP = 100;

inline const uint32_t ATT_EST_ROLLING_WINDOW_SIZE = 20;

inline const double ATT_EST_HARD_IRON_OFFSET_X = 0.104;
inline const double ATT_EST_HARD_IRON_OFFSET_Y = 0.076;
inline const double ATT_EST_HARD_IRON_OFFSET_Z = 0.062;

struct AttEstPointSample {
    double x;
    double y;
    double z;
};

struct AttEstInput {
    AttEstPointSample acc;       // [m/s^2]
    AttEstPointSample ang_rate;  // [rad/s]
    AttEstPointSample mag_field; // [gauss]
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

class AttitudeEstimation
{
public:
    AttitudeEstimation(double input_sample_rate_s);

    void update(AttEstInput input);

    AttEstimate get_estimate();

    AttEstPointSample get_lp_filtered_acc();

    bool is_calibrated();
    bool is_standstill();
private:
    const double _dt;

    AttEstInput _in = {0};
    AttEstimate _est = {0};

    bool _is_standstill = false;

    AttEstPointSample _gyro_offset = {0, 0, 0};
    AttEstPointSample _lp_filtered_acc = {0, 0, 0};

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

    statistics::RollingStats _rolling_stats_roll_angle {ATT_EST_ROLLING_WINDOW_SIZE};
    statistics::RollingStats _rolling_stats_pitch_angle {ATT_EST_ROLLING_WINDOW_SIZE};
    statistics::RollingStats _rolling_stats_yaw_angle {ATT_EST_ROLLING_WINDOW_SIZE};

    statistics::RollingStats _rolling_stats_roll_rate {ATT_EST_ROLLING_WINDOW_SIZE};
    statistics::RollingStats _rolling_stats_pitch_rate {ATT_EST_ROLLING_WINDOW_SIZE};
    statistics::RollingStats _rolling_stats_yaw_rate {ATT_EST_ROLLING_WINDOW_SIZE};

    Iir::Butterworth::LowPass<ATT_EST_LP_BUTTER_ACC_ORDER> _lp_filter_acc_x;
    Iir::Butterworth::LowPass<ATT_EST_LP_BUTTER_ACC_ORDER> _lp_filter_acc_y;
    Iir::Butterworth::LowPass<ATT_EST_LP_BUTTER_ACC_ORDER> _lp_filter_acc_z;

    void _lp_filter_acc();

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
