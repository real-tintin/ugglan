#ifndef ATTITUDE_ESTIMATION_H
#define ATTITUDE_ESTIMATION_H

#include <cstdint>
#include <math.h>
#include <utils.h>
#include <matrix.h>

inline const double ATT_EST_KALMAN_Q_VARIANCE = utils::get_env("ATT_EST_KALMAN_Q_VARIANCE", 1e1);
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
    Matrix x = {{0}, {0}, {0}};
    Matrix z = {{0}, {0}};
    Matrix P = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
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

    Matrix _Q;
    Matrix _F;
    Matrix _R;

    Matrix _H = {{1, 0, 0}, {0, 1, 0}};
    Matrix _I = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

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
