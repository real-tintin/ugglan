#include <pilot_control.h>

static const double C_PHI = 1 / droneprops::I_XX;
static const double C_THETA = 1 / droneprops::I_YY;
static const double C_PSI = 1 / droneprops::I_ZZ;

static const double ROTATION_SCALE = 2.0;
static const double ROTATION_OFFSET = -1.0;

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

    ref.roll = (ROTATION_SCALE * _range_ref(gimbal_right_x) + ROTATION_OFFSET) * PILOT_CTRL_ABS_MAX_REF_ROLL;
    ref.pitch = - (ROTATION_SCALE * _range_ref(gimbal_right_y) + ROTATION_OFFSET) * PILOT_CTRL_ABS_MAX_REF_PITCH;
    ref.yaw_rate = (ROTATION_SCALE * _range_ref(gimbal_left_x) + ROTATION_OFFSET) * PILOT_CTRL_ABS_MAX_REF_YAW_RATE;
    ref.f_z = - _range_ref(gimbal_left_y) * PILOT_CTRL_ABS_MAX_REF_F_Z;

    return ref;
}

PilotControl::PilotControl(double input_sample_rate_s) :
    _sample_rate_s(input_sample_rate_s)
{}

void PilotControl::update(AttEstimate att_est, PilotCtrlRef ref)
{
    _extract_states(att_est);
    _change_of_variable(ref);
    _integrate_with_antiwindup();

    _update_ctrl_mx();
    _update_ctrl_my();
    _update_ctrl_mz();
    _update_ctrl_fz(ref);
}

BodyControl PilotControl::get_ctrl()
{
    return _ctrl;
}

double PilotControl::get_state(PilotCtrlState state)
{
    switch (state)
    {
        case PilotCtrlState::Phi0:
            return _x_phi[0][0];
        case PilotCtrlState::Phi1:
            return _x_phi[1][0];
        case PilotCtrlState::Phi2:
            return _x_phi[2][0];
        case PilotCtrlState::Phi3:
            return _x_phi[3][0];

        case PilotCtrlState::Theta0:
            return _x_theta[0][0];
        case PilotCtrlState::Theta1:
            return _x_theta[1][0];
        case PilotCtrlState::Theta2:
            return _x_theta[2][0];
        case PilotCtrlState::Theta3:
            return _x_theta[3][0];

        case PilotCtrlState::Psi0:
            return _x_psi[0][0];
        case PilotCtrlState::Psi1:
            return _x_psi[1][0];
        case PilotCtrlState::Psi2:
            return _x_psi[2][0];

        default:
            return 0; // Should never be reached.
    }
}

void PilotControl::_extract_states(AttEstimate& att_est)
{
    _x_phi[1][0] = att_est.roll.angle;
    _x_phi[2][0] = att_est.roll.rate;
    _x_phi[3][0] = att_est.roll.acc;

    _x_theta[1][0] = att_est.pitch.angle;
    _x_theta[2][0] = att_est.pitch.rate;
    _x_theta[3][0] = att_est.pitch.acc;

    _x_psi[1][0] = att_est.yaw.rate;
    _x_psi[2][0] = att_est.yaw.acc;
}

void PilotControl::_change_of_variable(PilotCtrlRef& ref)
{
    _x_phi[1][0] -= ref.roll;
    _x_theta[1][0] -= ref.pitch;
    _x_psi[1][0] -= ref.yaw_rate;
}

void PilotControl::_integrate_with_antiwindup()
{
    _x_phi[0][0] += _x_phi[1][0] * _sample_rate_s;
    _x_theta[0][0] += _x_theta[1][0] * _sample_rate_s;
    _x_psi[0][0] += _x_psi[1][0] * _sample_rate_s;

    _x_phi[0][0] = _range_sat(_x_phi[0][0], PILOT_CTRL_ANTI_WINDUP_SAT_PHI);
    _x_theta[0][0] = _range_sat(_x_theta[0][0], PILOT_CTRL_ANTI_WINDUP_SAT_THETA);
    _x_psi[0][0] = _range_sat(_x_psi[0][0], PILOT_CTRL_ANTI_WINDUP_SAT_PSI);
}

void PilotControl::_update_ctrl_mx()
{
    _ctrl.m_x = _feedback_ctrl(_x_phi, PILOT_CTRL_L_ROLL);
}

void PilotControl::_update_ctrl_my()
{
    _ctrl.m_y = _feedback_ctrl(_x_theta, PILOT_CTRL_L_PITCH);
}

void PilotControl::_update_ctrl_mz()
{
    _ctrl.m_z = _feedback_ctrl(_x_psi, PILOT_CTRL_L_YAW_RATE);
}

void PilotControl::_update_ctrl_fz(PilotCtrlRef& ref)
{
    _ctrl.f_z = ref.f_z;
}

double PilotControl::_feedback_ctrl(Matrix x, Matrix L)
{
    Matrix u = (-1 * L * x);
    return u[0][0];
}
