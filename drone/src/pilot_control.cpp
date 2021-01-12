#include <pilot_control.h>
#include <logger.h>

// TODO: PRIVATE BETA ETC.

PilotControl::PilotControl(double input_sample_rate_s) :
    _sample_rate_s(input_sample_rate_s)
{}

BodyControl PilotControl::update(AttEstimate att_est, NormalizedRef norm_ref)
{
    BodyControl body_ctrl = {0};
    AbsoluteRef abs_ref = _norm_to_abs_ref(norm_ref);

    _att_est_variable_change(att_est, abs_ref);
    _update_intgrated_states(att_est);

    //BETA
    //prev_body_ctrl
    //integrator + intg states
    // TODO: Change of variable.
    // TODO: _angle_ctrl(x_v, u, c, alpha)
    // TODO: _angular_rate_ctrl(x_v, u, c, alpha)
    // TODO: _reduced_observer(x_v, u, c, alpha)
    //_get_body_ctrl_roll_pitch();
    //_update_body_ctrl_my();
    //_update_body_ctrl_mz();

    // TODO: get_ctrl??? store body_ctrl

    body_ctrl.f_z = _update_body_ctrl_fz(_abs_ref.f_z);

    _prev_att_est = att_est;

    return body_ctrl;
}

void PilotControl::_att_est_variable_change(AttEstimate& att_est, AbsoluteRef& abs_ref)
{
    att_est.roll -= abs_ref.roll;
    att_est.pitch -= abs_ref.pitch;
    att_est.yaw_rate -= abs_ref.yaw_rate;
}

void PilotControl::_update_intgrated_states(AttEstimate& att_est)
{
    _intg_roll += att_est.roll * _sample_rate_s;
    _intg_pitch = att_est.pitch * _sample_rate_s;
    _intg_yaw_rate = att_est.yaw_rate * _sample_rate_s;
}

AbsoluteRef PilotControl::_norm_to_abs_ref(NormalizedRef norm_ref)
{
    AbsoluteRef abs_ref;

    abs_ref.roll = norm_ref.roll * PILOT_CTRL_SCALE_REF_ROLL;
    abs_ref.pitch = norm_ref.pitch * PILOT_CTRL_SCALE_REF_PITCH;
    abs_ref.yaw_rate = norm_ref.yaw_rate * PILOT_CTRL_SCALE_REF_YAW_RATE;
    abs_ref.f_z = norm_ref.f_z * PILOT_CTRL_SCALE_REF_F_Z;

    return abs_ref;
}

void PilotControl::_update_body_ctrl_mx()
{
    // TODO: change of variable
    // TODO: u = ....
    _body_ctrl.f_z = _abs_ref.f_z;
}

void PilotControl::_get_body_ctrl_fz(double ref) { return ref; }
