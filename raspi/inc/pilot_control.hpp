#ifndef PILOT_CONTROL_HPP
#define PILOT_CONTROL_HPP

#include <algorithm>
#include <cstdint>
#include <cstring>
extern "C"
{
#include <math.h>
}

#include <eigen/Eigen>

#include <attitude_estimation.hpp>
#include <drone_props.hpp>
#include <motor_control.hpp>

inline const double PILOT_CTRL_ABS_MAX_REF_ROLL = M_PI / 8;  // [rad]
inline const double PILOT_CTRL_ABS_MAX_REF_PITCH = M_PI / 8; // [rad]
inline const double PILOT_CTRL_ABS_MAX_REF_YAW_RATE = M_PI;  // [rad/s]
inline const double PILOT_CTRL_ABS_MAX_REF_F_Z = 20.0;       // [N]

enum class PilotCtrlState
{
    Phi0, // [rads]
    Phi1, // [rad]
    Phi2, // [rad/s]
    Phi3, // [rad/s^2]

    Theta0, // [rads]
    Theta1, // [rad]
    Theta2, // [rad/s]
    Theta3, // [rad/s^2]

    Psi0, // [rad]
    Psi1, // [rad/s]
    Psi2, // [rad/s^2]
};

struct PilotCtrlRef
{
    double roll;     // [rad]
    double pitch;    // [rad]
    double yaw_rate; // [rad/s]
    double f_z;      // [N]
};

struct PilotCtrlConfig
{
    double anti_windup_sat_phi;
    double anti_windup_sat_theta;
    double anti_windup_sat_psi;

    Eigen::RowVector4d L_roll;     // Feedback matrice for roll.
    Eigen::RowVector4d L_pitch;    // Feedback matrice for pitch.
    Eigen::RowVector3d L_yaw_rate; // Feedback matrice for yaw-rate.
};

PilotCtrlRef tgyia6c_to_pilot_ctrl_ref(double gimbal_left_x,
                                       double gimbal_left_y,
                                       double gimbal_right_x,
                                       double gimbal_right_y);

class PilotControl
{
  public:
    PilotControl(double input_sample_rate_s, PilotCtrlConfig config);

    void update(att_est::Attitude attitude, PilotCtrlRef ref);
    void reset();

    BodyControl get_ctrl();

    double get_state(PilotCtrlState state);

  private:
    const double _sample_rate_s;
    const PilotCtrlConfig _config;

    BodyControl _ctrl = {0};

    Eigen::Vector4d _x_phi{{0}, {0}, {0}, {0}};
    Eigen::Vector4d _x_theta{{0}, {0}, {0}, {0}};
    Eigen::Vector3d _x_psi{{0}, {0}, {0}};

    Eigen::Vector<double, 1> _u;

    void _extract_states(att_est::Attitude &attitude);
    void _change_of_variable(PilotCtrlRef &ref);
    void _integrate_with_antiwindup();

    void _update_ctrl_mx();
    void _update_ctrl_my();
    void _update_ctrl_mz();
    void _update_ctrl_fz(PilotCtrlRef &ref);

    double _feedback_ctrl(Eigen::Vector4d x, Eigen::RowVector4d L);
    double _feedback_ctrl(Eigen::Vector3d x, Eigen::RowVector3d L);
};

#endif /* PILOT_CONTROL_HPP */
