#ifndef ATTITUDE_ESTIMATION_H
#define ATTITUDE_ESTIMATION_H

#include <cstdint>
#include <math.h>

inline const uint8_t ATT_EST_N_SAMPLES_GYRO_OFFSET_COMP = 20;

inline const double ATT_EST_TAU_ROLL = 1 / (2 * M_PI * 20); // 20 Hz cut-off freq.
inline const double ATT_EST_TAU_PITCH = 1 / (2 * M_PI * 20); // 20 Hz cut-off freq.
inline const double ATT_EST_TAU_YAW = 1 / (2 * M_PI * 20); // 20 Hz cut-off freq.

inline const double ATT_EST_MODULO_ROLL = M_PI;
inline const double ATT_EST_MODULO_PITCH = M_PI / 2;
inline const double ATT_EST_MODULO_YAW = M_PI;

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

struct AttEstimates {
    double roll;       // [rad]
    double pitch;      // [rad]
    double yaw;        // [rad]
    double ang_rate_x; // [rad/s]
    double ang_rate_y; // [rad/s]
    double ang_rate_z; // [rad/s]
};

class AttitudeEstimation
{
public:
    AttitudeEstimation(double input_sample_rate_ms);

    void update(AttEstInput input);

    AttEstimates get_estimates();

    bool is_calibrated();
private:
    const double _sample_rate_s;

    AttEstInput _in = {0};
    AttEstimates _est = {0};

    double _gyro_offset_x = 0;
    double _gyro_offset_y = 0;
    double _gyro_offset_z = 0;

    bool _is_gyro_offset_comp = false;
    uint8_t _samples_gyro_offset_comp = 0;

    void _gyro_offset_comp();

    void _update_roll();
    void _update_pitch();
    void _update_yaw();

    double _complementary_filter(double y_old, double u, double up, double tau);
    double _modulo_angle(double angle, double limit);
};

#endif /* ATTITUDE_ESTIMATION_H */
