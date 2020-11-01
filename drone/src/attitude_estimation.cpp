#include <attitude_estimation.h>

AttitudeEstimation::AttitudeEstimation(double input_sample_rate_ms) :
    _sample_rate_s(input_sample_rate_ms)
{}

void AttitudeEstimation::update(AttEstInput input)
{
    _in = input;

    _gyro_offset_comp();

    if (is_calibrated())
    {
        _update_roll();
        _update_pitch();
        _update_yaw();
    }
}

AttEstimates AttitudeEstimation::get_estimates()
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

    _est.roll = _complementary_filter(_est.roll, roll_acc, _est.ang_rate_x, ATT_EST_TAU_ROLL);
    _est.roll = _modulo_angle(_est.roll, ATT_EST_MODULO_ROLL);
}

void AttitudeEstimation::_update_pitch()
{
    double pitch_acc = atan2(_in.acc_x, sqrt(pow(_in.acc_y, 2) + pow(_in.acc_z, 2)));

    _est.pitch = _complementary_filter(_est.pitch, pitch_acc, _est.ang_rate_y, ATT_EST_TAU_ROLL);
    _est.pitch = _modulo_angle(_est.pitch, ATT_EST_MODULO_PITCH);
}

void AttitudeEstimation::_update_yaw()
{
    double m_x = _in.mag_field_x * cos(_est.pitch) +
                  _in.mag_field_y * sin(_est.roll) * sin(_est.pitch) +
                  _in.mag_field_z * sin(_est.pitch) * cos(_est.roll);
    double m_y = _in.mag_field_y * cos(_est.roll) - _in.mag_field_z * sin(_est.roll);

    double yaw_mag = atan2(m_y, m_x);

    _est.yaw = _complementary_filter(_est.yaw, yaw_mag, _est.ang_rate_z, ATT_EST_TAU_YAW);
    _est.yaw = _modulo_angle(_est.yaw, ATT_EST_MODULO_YAW);
}

double AttitudeEstimation::_modulo_angle(double angle, double limit)
{
    return fmod(angle + limit, 2.0 * limit) - limit;
}

double AttitudeEstimation::_complementary_filter(double y_old, double u, double up, double tau)
{
    double alpha = tau / (tau + _sample_rate_s);
    double y_new = alpha * (y_old + up * _sample_rate_s) + (1 - alpha) * u;

    return y_new;
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
        _est.ang_rate_x = _in.ang_rate_x - _gyro_offset_x;
        _est.ang_rate_y = _in.ang_rate_y - _gyro_offset_y;
        _est.ang_rate_z = _in.ang_rate_z - _gyro_offset_z;
    }
}
