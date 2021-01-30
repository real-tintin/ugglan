#ifndef PILOT_CONTROL_H
#define PILOT_CONTROL_H

#include <cstdint>
#include <cstring>
#include <algorithm>
#include <math.h>
#include <attitude_estimation.h>
#include <motor_control.h>
#include <drone_props.h>

inline const uint8_t PILOT_CTRL_X_SIZE = 3;
inline const uint8_t PILOT_CTRL_L_SIZE = 4;

inline const double PILOT_CTRL_ABS_MAX_REF_ROLL = M_PI / 8; // [rad]
inline const double PILOT_CTRL_ABS_MAX_REF_PITCH = M_PI / 8; // [rad]
inline const double PILOT_CTRL_ABS_MAX_REF_YAW_RATE = M_PI; // [rad/s]
inline const double PILOT_CTRL_ABS_MAX_REF_F_Z = 20.0; // [N]

inline const double PILOT_CTRL_ANTI_WINDUP_SAT_PHI = 0.3; // About 75 % of max step size [rads]
inline const double PILOT_CTRL_ANTI_WINDUP_SAT_THETA = 0.3; // About 75 % of max step size [rads]
inline const double PILOT_CTRL_ANTI_WINDUP_SAT_PSI = 2.4; // About 75 % of max step size [rad]

inline const double PILOT_CTRL_L_ROLL[PILOT_CTRL_L_SIZE] = {1.5, 4.0, 0.6, 1.5};        // Feedback matrice for roll.
inline const double PILOT_CTRL_L_PITCH[PILOT_CTRL_L_SIZE] = {1.5, 4.0, 0.6, 1.5};       // Feedback matrice for pitch.
inline const double PILOT_CTRL_L_YAW_RATE[PILOT_CTRL_L_SIZE] = {0.0, 0.02, 0.04, 0.01}; // Feedback matrice for yaw-rate. Note, first state not used.

inline const double PILOT_CTRL_ALPHA_PHI = 5.0;   // Reduced observer parameter for phi.
inline const double PILOT_CTRL_ALPHA_THETA = 5.0; // Reduced observer parameter for theta.
inline const double PILOT_CTRL_ALPHA_PSI = 5.0;   // Reduced observer parameter for psi.

struct PilotCtrlRef
{
    double roll;     // [rad]
    double pitch;    // [rad]
    double yaw_rate; // [rad/s]
    double f_z;      // [N]
};

PilotCtrlRef tgyia6c_to_pilot_ctrl_ref(double gimbal_left_x, double gimbal_left_y,
                                       double gimbal_right_x, double gimbal_right_y);

class PilotControl
{
public:
    PilotControl(double input_sample_rate_s);

    void update(AttEstimate est, PilotCtrlRef ref);

    BodyControl get_ctrl();
private:
    const double _sample_rate_s;

    BodyControl _ctrl = {0};

    double _x_phi[PILOT_CTRL_X_SIZE] = {0};
    double _x_phi_prev[PILOT_CTRL_X_SIZE] = {0};

    double _x_theta[PILOT_CTRL_X_SIZE] = {0};
    double _x_theta_prev[PILOT_CTRL_X_SIZE] = {0};

    double _x_psi[PILOT_CTRL_X_SIZE] = {0};
    double _x_psi_prev[PILOT_CTRL_X_SIZE] = {0};

    void _store_old_states();
    void _change_of_variable(AttEstimate& att_est, PilotCtrlRef& ref);
    void _update_new_states(AttEstimate& att_est);
    void _integrate_states();

    void _update_ctrl_mx();
    void _update_ctrl_my();
    void _update_ctrl_mz();
    void _update_ctrl_fz(PilotCtrlRef& ref);

    double _feedback_ctrl(double x[PILOT_CTRL_X_SIZE],
                          double x_prev[PILOT_CTRL_X_SIZE],
                          double u_prev,
                          const double c,
                          const double alpha,
                          const double L[PILOT_CTRL_L_SIZE]);
};

#endif /* PILOT_CONTROL_H */
