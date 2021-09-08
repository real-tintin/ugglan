#ifndef ATTITUDE_ESTIMATION_H
#define ATTITUDE_ESTIMATION_H

#include <cstdint>
#include <math.h>
#include <utils.h>
#include <eigen/Eigen>

inline const double ATT_EST_KALMAN_Q_VARIANCE = utils::get_env("ATT_EST_KALMAN_Q_VARIANCE", 5e-1);
inline const double ATT_EST_KALMAN_R_VARIANCE = utils::get_env("ATT_EST_KALMAN_R_VARIANCE", 1e-3);

inline const uint8_t ATT_EST_N_SAMPLES_GYRO_OFFSET_COMP = 20;

inline const double ATT_EST_HARD_IRON_OFFSET_X = 0.131;
inline const double ATT_EST_HARD_IRON_OFFSET_Y = 0.143;
inline const double ATT_EST_HARD_IRON_OFFSET_Z = -0.144;

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

    bool is_calibrated();
private:
    const double _dt;

    AttEstInput _in = {0};
    AttEstimate _est = {0};

    double _gyro_offset_x = 0;
    double _gyro_offset_y = 0;
    double _gyro_offset_z = 0;

    bool _is_gyro_offset_comp = false;
    uint8_t _samples_gyro_offset_comp = 0;

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

    AttEstKalmanState _kalman_roll;
    AttEstKalmanState _kalman_pitch;
    AttEstKalmanState _kalman_yaw;

    void _gyro_offset_comp();
    void _hard_iron_offset_comp();

    void _update_roll();
    void _update_pitch();
    void _update_yaw();

    void _update_est(double z_0, double z_1,
                     AttEstKalmanState& kalman_state,
                     AttEstState& att_state,
                     double modulo_lim);

    void _update_kalman_state(AttEstKalmanState& state);
    void _kalman_state_to_att_state(AttEstKalmanState& kalman, AttEstState& att);
    void _modulo_angle(double* angle, double limit);
};

#endif /* ATTITUDE_ESTIMATION_H */
