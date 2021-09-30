#ifndef PILOT_CONTROL_H
#define PILOT_CONTROL_H

#include <cstdint>
#include <cstring>
#include <algorithm>
#include <math.h>
#include <attitude_estimation.h>
#include <motor_control.h>
#include <drone_props.h>
#include <utils.h>
#include <eigen/Eigen>

inline const double PILOT_CTRL_ABS_MAX_REF_ROLL = M_PI / 8; // [rad]
inline const double PILOT_CTRL_ABS_MAX_REF_PITCH = M_PI / 8; // [rad]
inline const double PILOT_CTRL_ABS_MAX_REF_YAW_RATE = M_PI; // [rad/s]
inline const double PILOT_CTRL_ABS_MAX_REF_F_Z = 20.0; // [N]

inline const double PILOT_CTRL_ANTI_WINDUP_SAT_PHI = utils::get_env("PILOT_CTRL_ANTI_WINDUP_SAT_PHI", 0.3);
inline const double PILOT_CTRL_ANTI_WINDUP_SAT_THETA = utils::get_env("PILOT_CTRL_ANTI_WINDUP_SAT_THETA", 0.3);
inline const double PILOT_CTRL_ANTI_WINDUP_SAT_PSI = utils::get_env("PILOT_CTRL_ANTI_WINDUP_SAT_PSI", 2.4);

inline const Eigen::RowVector4d PILOT_CTRL_L_ROLL
    {{
    utils::get_env("PILOT_CTRL_L_ROLL_0", 0.40),
    utils::get_env("PILOT_CTRL_L_ROLL_1", 3.85),
    utils::get_env("PILOT_CTRL_L_ROLL_2", 0.55),
    utils::get_env("PILOT_CTRL_L_ROLL_3", 0.02)
    }}; // Feedback matrice for roll.

inline const Eigen::RowVector4d PILOT_CTRL_L_PITCH
    {{
    utils::get_env("PILOT_CTRL_L_PITCH_0", 0.40),
    utils::get_env("PILOT_CTRL_L_PITCH_1", 3.85),
    utils::get_env("PILOT_CTRL_L_PITCH_2", 0.55),
    utils::get_env("PILOT_CTRL_L_PITCH_3", 0.02)
    }}; // Feedback matrice for pitch.

inline const Eigen::RowVector3d PILOT_CTRL_L_YAW_RATE
    {{
    utils::get_env("PILOT_CTRL_L_YAW_RATE_0", 0.02),
    utils::get_env("PILOT_CTRL_L_YAW_RATE_1", 0.04),
    utils::get_env("PILOT_CTRL_L_YAW_RATE_2", 0.00)
    }}; // Feedback matrice for yaw-rate.

enum class PilotCtrlState {
    Phi0,   // [rads]
    Phi1,   // [rad]
    Phi2,   // [rad/s]
    Phi3,   // [rad/s^2]

    Theta0, // [rads]
    Theta1, // [rad]
    Theta2, // [rad/s]
    Theta3, // [rad/s^2]

    Psi0,   // [rad]
    Psi1,   // [rad/s]
    Psi2,   // [rad/s^2]
};

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
    void reset();

    BodyControl get_ctrl();

    double get_state(PilotCtrlState state);
private:
    const double _sample_rate_s;

    BodyControl _ctrl = {0};

    Eigen::Vector4d _x_phi {{0}, {0}, {0}, {0}};
    Eigen::Vector4d _x_theta {{0}, {0}, {0}, {0}};
    Eigen::Vector3d _x_psi {{0}, {0}, {0}};

    Eigen::Vector<double, 1> _u;

    void _extract_states(AttEstimate& att_est);
    void _change_of_variable(PilotCtrlRef& ref);
    void _integrate_with_antiwindup();

    void _update_ctrl_mx();
    void _update_ctrl_my();
    void _update_ctrl_mz();
    void _update_ctrl_fz(PilotCtrlRef& ref);

    double _feedback_ctrl(Eigen::Vector4d x, Eigen::RowVector4d L);
    double _feedback_ctrl(Eigen::Vector3d x, Eigen::RowVector3d L);
};

#endif /* PILOT_CONTROL_H */
