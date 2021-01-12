#ifndef PILOT_CONTROL_H
#define PILOT_CONTROL_H

#include <cstdint>
#include <math.h>
#include <attitude_estimation.h>
#include <motor_control.h>
#include <physical_const.h>

inline const double PILOT_CTRL_SCALE_REF_ROLL = M_PI / 8; // [rad]
inline const double PILOT_CTRL_SCALE_REF_PITCH = M_PI / 8; // [rad]
inline const double PILOT_CTRL_SCALE_REF_YAW_RATE = M_PI; // [rad/s]
inline const double PILOT_CTRL_SCALE_REF_F_Z = -20.0; // [N]

inline const double PILOT_CTRL_L_ROLL[4] = {0.5, 4.0, 0.6, 1.5};  // State feedback matrice for roll.
inline const double PILOT_CTRL_L_PITCH[4] = {0.5, 4.0, 0.6, 1.5}; // State feedback matrice for pitch.
inline const double PILOT_CTRL_L_YAW_RATE[3] = {0.1, 0.2, 0.3};   // State feedback matrice for yaw-rate.

inline const double PILOT_CTRL_ALPHA_ROLL = 5.0;     // Reduced observer parameter for roll.
inline const double PILOT_CTRL_ALPHA_PITCH = 5.0;    // Reduced observer parameter for pitch.
inline const double PILOT_CTRL_ALPHA_YAW_RATE = 5.0; // Reduced observer parameter for yaw-rate.

struct NormalizedRef
{
    double roll;     // [-1.0, 1.0]
    double pitch;    // [-1.0, 1.0]
    double yaw_rate; // [-1.0, 1.0]
    double f_z;      // [0.0, 1.0]
};

typedef NormalizedRef AbsoluteRef;

class PilotControl
{
public:
    PilotControl(double input_sample_rate_s);

    BodyControl update(AttEstimate att_est, NormalizedRef norm_ref);

private:
    const double _sample_rate_s;

    AttEstimate _prev_att_est = {0};
    BodyControl _prev_body_ctrl = {0};

    double _intg_roll = 0;
    double _intg_pitch = 0;
    double _intg_yaw_rate = 0;

    AbsoluteRef _norm_to_abs_ref(NormalizedRef norm_ref);

    void _att_est_variable_change(AttEstimate& att_est, AbsoluteRef& abs_ref);
    void _update_intgrated_states(AttEstimate& att_est);

    void _update_body_ctrl_mx();
    void _update_body_ctrl_fz();
};

#endif /* PILOT_CONTROL_H */
