#include <pilot_control.h>

static const double _C_PHI = 1 / DRONE_I_XX;
static const double _C_THETA = 1 / DRONE_I_YY;
static const double _C_PSI = 1 / DRONE_I_ZZ;

static const double _ROTATION_SCALE = 2.0;
static const double _ROTATION_OFFSET = -1.0;

static double _range_lim(double x, double x_min, double x_max)
{
    return std::max(x_min, std::min(x_max, x));
}

static double _range_ref(double x) { return _range_lim(x, 0.0, 1.0); }
static double _range_sat(double x, double v) { return _range_lim(x, -v, v); }

PilotCtrlRef tgyia6c_to_pilot_ctrl_ref(double gimbal_left_x,  /* [0.0-1.0] */
                                       double gimbal_left_y,  /* [0.0-1.0] */
                                       double gimbal_right_x, /* [0.0-1.0] */
                                       double gimbal_right_y  /* [0.0-1.0] */
                                       )
{
    PilotCtrlRef ref;

    ref.roll = (_ROTATION_SCALE * _range_ref(gimbal_right_x) + _ROTATION_OFFSET) * PILOT_CTRL_ABS_MAX_REF_ROLL;
    ref.pitch = - (_ROTATION_SCALE * _range_ref(gimbal_right_y) + _ROTATION_OFFSET) * PILOT_CTRL_ABS_MAX_REF_PITCH;
    ref.yaw_rate = (_ROTATION_SCALE * _range_ref(gimbal_left_x) + _ROTATION_OFFSET) * PILOT_CTRL_ABS_MAX_REF_YAW_RATE;
    ref.f_z = - _range_ref(gimbal_left_y) * PILOT_CTRL_ABS_MAX_REF_F_Z;

    return ref;
}

PilotControl::PilotControl(double input_sample_rate_s) :
    _sample_rate_s(input_sample_rate_s)
{}

void PilotControl::update(AttEstimate att_est, PilotCtrlRef ref)
{
    _store_old_states();
    _change_of_variable(att_est, ref);
    _update_new_states(att_est);
    _integrate_states();

    _update_ctrl_mx();
    _update_ctrl_my();
    _update_ctrl_mz();
    _update_ctrl_fz(ref);
}

BodyControl PilotControl::get_ctrl()
{
    return _ctrl;
}

void PilotControl::_store_old_states()
{
    std::memcpy(_x_phi_prev, _x_phi, sizeof(double) * PILOT_CTRL_X_SIZE);
    std::memcpy(_x_theta_prev, _x_theta, sizeof(double) * PILOT_CTRL_X_SIZE);
    std::memcpy(_x_psi_prev, _x_psi, sizeof(double) * PILOT_CTRL_X_SIZE);
}

void PilotControl::_change_of_variable(AttEstimate& att_est, PilotCtrlRef& ref)
{
    att_est.roll -= ref.roll;
    att_est.pitch -= ref.pitch;
    att_est.yaw_rate -= ref.yaw_rate;
}

void PilotControl::_update_new_states(AttEstimate& att_est)
{
    _x_phi[1] = att_est.roll;
    _x_phi[2] = att_est.roll_rate;

    _x_theta[1] = att_est.pitch;
    _x_theta[2] = att_est.pitch_rate;

    // _x_psi[0] not used i.e., _x_psi[1] is the integral of yaw-rate.
    _x_psi[2] = att_est.yaw_rate;
}

void PilotControl::_integrate_states()
{
    _x_phi[0] += _x_phi[1] * _sample_rate_s;
    _x_theta[0] += _x_theta[1] * _sample_rate_s;
    _x_psi[1] += _x_psi[2] * _sample_rate_s;

    _x_phi[0] = _range_sat(_x_phi[0], PILOT_CTRL_ANTI_WINDUP_SAT_PHI);
    _x_theta[0] = _range_sat(_x_theta[0], PILOT_CTRL_ANTI_WINDUP_SAT_THETA);
    _x_psi[1] = _range_sat(_x_psi[1], PILOT_CTRL_ANTI_WINDUP_SAT_PSI);
}

void PilotControl::_update_ctrl_mx()
{
    _ctrl.m_x = _feedback_ctrl(_x_phi, _x_phi_prev, _ctrl.m_x,
                          _C_PHI, PILOT_CTRL_ALPHA_PHI, PILOT_CTRL_L_ROLL);
}

void PilotControl::_update_ctrl_my()
{
    _ctrl.m_y = _feedback_ctrl(_x_theta, _x_theta_prev, _ctrl.m_y,
                          _C_THETA, PILOT_CTRL_ALPHA_THETA, PILOT_CTRL_L_PITCH);
}

void PilotControl::_update_ctrl_mz()
{
    _ctrl.m_z = _feedback_ctrl(_x_psi, _x_psi_prev, _ctrl.m_z,
                          _C_PSI, PILOT_CTRL_ALPHA_PSI, PILOT_CTRL_L_YAW_RATE);
}

void PilotControl::_update_ctrl_fz(PilotCtrlRef& ref)
{
    _ctrl.f_z = ref.f_z;
}

double PilotControl::_feedback_ctrl(double x[PILOT_CTRL_X_SIZE],
                                    double x_prev[PILOT_CTRL_X_SIZE],
                                    double u_prev,
                                    const double c,
                                    const double alpha,
                                    const double L[PILOT_CTRL_L_SIZE])
{
    /* See doc for details. */
    double beta_4 = MOTOR_TAU + _sample_rate_s * (1 + MOTOR_TAU * alpha * c);
    double beta_1 = MOTOR_TAU / beta_4;
    double beta_2 = -alpha * _sample_rate_s * (1 + alpha * MOTOR_TAU * c) / beta_4;
    double beta_3 = _sample_rate_s / beta_4;

    return (-(L[0] * x[0] +
              L[1] * x[1] +
             (L[2] + L[3] * (beta_2 + alpha)) * x[2] +
              L[3] * beta_1 * (u_prev - alpha * x_prev[2])) /
              (1 + L[3] * beta_3));
}
