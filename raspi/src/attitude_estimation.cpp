#include <attitude_estimation.h>

static const double MODULO_ROLL = M_PI;
static const double MODULO_PITCH = M_PI / 2;
static const double MODULO_YAW = M_PI;

AttitudeEstimation::AttitudeEstimation(double input_sample_rate_s) :
    _dt(input_sample_rate_s)
{
    _Q = ATT_EST_KALMAN_Q_VARIANCE * Matrix({
        {0.25 * pow(_dt, 4), 0.5 * pow(_dt, 3), 0.5 * pow(_dt, 2)},
        {0.5 * pow(_dt, 3), pow(_dt, 2), _dt},
        {0.5 * pow(_dt, 2), _dt, 1}
        });

    _F = Matrix({
        {1, _dt, 0.5 * pow(_dt, 2)},
        {0, 1, _dt},
        {0, 0, 1}
        });

    _R = ATT_EST_KALMAN_R_VARIANCE * Matrix({{1, 0}, {0, 1}});
}

void AttitudeEstimation::update(AttEstInput input)
{
    _in = input;

    _gyro_offset_comp();
    _hard_iron_offset_comp();

    if (is_calibrated())
    {
        _update_roll();
        _update_pitch();
        _update_yaw();
    }
}

AttEstimate AttitudeEstimation::get_estimate()
{
    return _est;
}

bool AttitudeEstimation::is_calibrated()
{
    return _is_gyro_offset_comp;
}

void AttitudeEstimation::_update_roll()
{
    double roll_acc = atan2(-_in.acc_y, -_in.acc_z);

    _update_est(roll_acc, _in.ang_rate_x, _kalman_roll, _est.roll, MODULO_ROLL);
}

void AttitudeEstimation::_update_pitch()
{
    double pitch_acc = atan2(_in.acc_x, sqrt(pow(_in.acc_y, 2) + pow(_in.acc_z, 2)));

    _update_est(pitch_acc, _in.ang_rate_y, _kalman_pitch, _est.pitch, MODULO_PITCH);
}

void AttitudeEstimation::_update_yaw()
{
    double b_x = _in.mag_field_x * cos(_est.pitch.angle) +
                  _in.mag_field_y * sin(_est.roll.angle) * sin(_est.pitch.angle) +
                  _in.mag_field_z * sin(_est.pitch.angle) * cos(_est.roll.angle);
    double b_y = _in.mag_field_y * cos(_est.roll.angle) - _in.mag_field_z * sin(_est.roll.angle);

    double yaw_mag = atan2(-b_y, b_x);

    _update_est(yaw_mag, _in.ang_rate_z, _kalman_yaw, _est.yaw, MODULO_YAW);
}

void AttitudeEstimation::_update_est(double z_0, double z_1,
                                     AttEstKalmanState& kalman_state,
                                     AttEstState& att_state,
                                     double modulo_lim)
{
    kalman_state.z[0][0] = z_0;
    kalman_state.z[1][0] = z_1;

    _update_kalman_state(kalman_state);
    _kalman_state_to_att_state(kalman_state, att_state);
    _modulo_angle(&att_state.angle, modulo_lim);
}

void AttitudeEstimation::_update_kalman_state(AttEstKalmanState& state)
{
    Matrix x_pri = _F * state.x;
    Matrix P_pri = _F * state.P * _F.transpose() + _Q;

    Matrix S = _H * P_pri * _H.transpose() + _R;
    Matrix K = P_pri * _H.transpose() * S.inverse();

    state.x = x_pri + K * (state.z - _H * x_pri);
    state.P = (_I - K * _H) * P_pri;
}

void AttitudeEstimation::_kalman_state_to_att_state(AttEstKalmanState& kalman, AttEstState& att)
{
    att.angle = kalman.x[0][0];
    att.rate = kalman.x[1][0];
    att.acc = kalman.x[2][0];
}

void AttitudeEstimation::_modulo_angle(double* angle, double limit)
{
    *angle = fmod(*angle + limit, 2.0 * limit) - limit;
}

void AttitudeEstimation::_gyro_offset_comp()
{
    if (!_is_gyro_offset_comp &&
        _samples_gyro_offset_comp < ATT_EST_N_SAMPLES_GYRO_OFFSET_COMP)
    {
        _gyro_offset_x += _in.ang_rate_x;
        _gyro_offset_y += _in.ang_rate_y;
        _gyro_offset_z += _in.ang_rate_z;

        _samples_gyro_offset_comp++;
    }

    if (!_is_gyro_offset_comp &&
        _samples_gyro_offset_comp == ATT_EST_N_SAMPLES_GYRO_OFFSET_COMP)
    {
        _gyro_offset_x /= ATT_EST_N_SAMPLES_GYRO_OFFSET_COMP;
        _gyro_offset_y /= ATT_EST_N_SAMPLES_GYRO_OFFSET_COMP;
        _gyro_offset_z /= ATT_EST_N_SAMPLES_GYRO_OFFSET_COMP;

        _is_gyro_offset_comp = true;
    }

    if (_is_gyro_offset_comp)
    {
        _in.ang_rate_x -= _gyro_offset_x;
        _in.ang_rate_y -= _gyro_offset_y;
        _in.ang_rate_z -= _gyro_offset_z;
    }
}

void AttitudeEstimation::_hard_iron_offset_comp()
{
    _in.mag_field_x -= ATT_EST_HARD_IRON_OFFSET_X;
    _in.mag_field_y -= ATT_EST_HARD_IRON_OFFSET_Y;
    _in.mag_field_z -= ATT_EST_HARD_IRON_OFFSET_Z;
}
