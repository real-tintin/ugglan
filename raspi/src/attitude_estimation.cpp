#include <attitude_estimation.hpp>

namespace att_est
{
namespace
{
static const double MODULO_ROLL = M_PI;
static const double MODULO_PITCH = M_PI / 2;
static const double MODULO_YAW = M_PI;

static const double STANDSTILL_ANG_VAR_LIM = 0.02;   // [rad]
static const double STANDSTILL_RATE_VAR_LIM = 0.005; // [rad/s]
} // namespace

Estimator::Estimator(double input_sample_rate_s, Config config)
    : _dt(input_sample_rate_s), _config(config),

      _rolling_stats_roll_angle(config.rolling_var_window_size),
      _rolling_stats_pitch_angle(config.rolling_var_window_size),
      _rolling_stats_yaw_angle(config.rolling_var_window_size),

      _rolling_stats_roll_rate(config.rolling_var_window_size),
      _rolling_stats_pitch_rate(config.rolling_var_window_size), _rolling_stats_yaw_rate(config.rolling_var_window_size)
{
    _Q = _config.kalman_q_scale * Eigen::Matrix3d({{0.25 * pow(_dt, 4), 0.5 * pow(_dt, 3), 0.5 * pow(_dt, 2)},
                                                   {0.5 * pow(_dt, 3), pow(_dt, 2), _dt},
                                                   {0.5 * pow(_dt, 2), _dt, 1}});

    _R = Eigen::Matrix2d({{_config.kalman_r_0, 0.0}, {0.0, _config.kalman_r_1}});

    _F = Eigen::Matrix3d({{1.0, _dt, 0.5 * pow(_dt, 2)}, {0.0, 1.0, _dt}, {0.0, 0.0, 1.0}});
    _F_t = _F.transpose();
}

void Estimator::update(Imu imu_uncompensated)
{
    _imu_uncompensated = imu_uncompensated;

    _acc_static_compensation();
    _gyro_dynamic_bias_compensation();
    _hard_iron_bias_compensation();

    _update_imu_angles();
    _update_rolling_stats();
    _update_standstill_status();

    if (is_calibrated())
    {
        _update_roll();
        _update_pitch();
        _update_yaw();
    }
}

Attitude Estimator::get_attitude()
{
    return _attitude;
}

Imu Estimator::get_imu_compensated()
{
    return _imu_compensated;
}

bool Estimator::is_calibrated()
{
    return _is_gyro_bias_compensated;
}

bool Estimator::is_standstill()
{
    return _is_standstill;
}

void Estimator::_update_imu_angles()
{
    _imu_roll_angle = atan2(-_imu_compensated.acc_y, -_imu_compensated.acc_z);
    _imu_pitch_angle =
        atan2(_imu_compensated.acc_x, sqrt(pow(_imu_compensated.acc_y, 2) + pow(_imu_compensated.acc_z, 2)));

    double b_x = _imu_compensated.mag_field_x * cos(_attitude.pitch.angle) +
                 _imu_compensated.mag_field_y * sin(_attitude.roll.angle) * sin(_attitude.pitch.angle) +
                 _imu_compensated.mag_field_z * sin(_attitude.pitch.angle) * cos(_attitude.roll.angle);
    double b_y = _imu_compensated.mag_field_y * cos(_attitude.roll.angle) -
                 _imu_compensated.mag_field_z * sin(_attitude.roll.angle);

    _imu_yaw_angle = atan2(-b_y, b_x);
}

void Estimator::_update_rolling_stats()
{
    _rolling_stats_roll_angle.update(_imu_roll_angle);
    _rolling_stats_pitch_angle.update(_imu_pitch_angle);
    _rolling_stats_yaw_angle.update(_imu_yaw_angle);

    _rolling_stats_roll_rate.update(_imu_compensated.ang_rate_x);
    _rolling_stats_pitch_rate.update(_imu_compensated.ang_rate_y);
    _rolling_stats_yaw_rate.update(_imu_compensated.ang_rate_z);
}

void Estimator::_update_standstill_status()
{
    if ((_rolling_stats_roll_angle.estimates_available() && _rolling_stats_pitch_angle.estimates_available() &&
         _rolling_stats_yaw_angle.estimates_available() &&

         _rolling_stats_roll_rate.estimates_available() && _rolling_stats_pitch_rate.estimates_available() &&
         _rolling_stats_yaw_rate.estimates_available()) &&
        ((_rolling_stats_roll_angle.get_variance() < STANDSTILL_ANG_VAR_LIM) &&
         (_rolling_stats_pitch_angle.get_variance() < STANDSTILL_ANG_VAR_LIM) &&
         (_rolling_stats_yaw_angle.get_variance() < STANDSTILL_ANG_VAR_LIM) &&

         (_rolling_stats_roll_rate.get_variance() < STANDSTILL_RATE_VAR_LIM) &&
         (_rolling_stats_pitch_rate.get_variance() < STANDSTILL_RATE_VAR_LIM) &&
         (_rolling_stats_yaw_rate.get_variance() < STANDSTILL_RATE_VAR_LIM)))
    {
        _is_standstill = true;
    }
}

void Estimator::_update_roll()
{
    _update_att_state(_imu_roll_angle, _imu_compensated.ang_rate_x, _kalman_roll, _attitude.roll, MODULO_ROLL);
}

void Estimator::_update_pitch()
{
    _update_att_state(_imu_pitch_angle, _imu_compensated.ang_rate_y, _kalman_pitch, _attitude.pitch, MODULO_PITCH);
}

void Estimator::_update_yaw()
{
    _update_att_state(_imu_yaw_angle, _imu_compensated.ang_rate_z, _kalman_yaw, _attitude.yaw, MODULO_YAW);
}

void Estimator::_update_att_state(
    double z_0, double z_1, KalmanState &kalman_state, AttState &att_state, double modulo_lim)
{
    kalman_state.z(0) = z_0;
    kalman_state.z(1) = z_1;

    _update_kalman_state(kalman_state);
    _kalman_state_to_att_state(kalman_state, att_state);
    _modulo_angle(&att_state.angle, modulo_lim);
}

void Estimator::_update_kalman_state(KalmanState &state)
{
    _x_pri.noalias() = _F * state.x;
    _P_pri.noalias() = _F * state.P * _F_t + _Q;

    _S.noalias() = _H * _P_pri * _H_t + _R;
    _K.noalias() = _P_pri * _H_t * _S.inverse();

    state.x.noalias() = _x_pri + _K * (state.z - _H * _x_pri);
    state.P.noalias() = (_I - _K * _H) * _P_pri;
}

void Estimator::_kalman_state_to_att_state(KalmanState &kalman, AttState &att)
{
    att.angle = kalman.x(0);
    att.rate = kalman.x(1);
    att.acc = kalman.x(2);
}

void Estimator::_modulo_angle(double *angle, double limit)
{
    *angle = fmod(*angle + limit, 2.0 * limit) - limit;
}

void Estimator::_acc_static_compensation()
{
    _imu_compensated.acc_x = _config.acc_error_s_x * _imu_uncompensated.acc_x +
                             _config.acc_error_m_x_y * _imu_uncompensated.acc_y +
                             _config.acc_error_m_x_z * _imu_uncompensated.acc_z + _config.acc_error_b_x;

    _imu_compensated.acc_y = _config.acc_error_m_y_x * _imu_uncompensated.acc_x +
                             _config.acc_error_s_y * _imu_uncompensated.acc_y +
                             _config.acc_error_m_y_z * _imu_uncompensated.acc_z + _config.acc_error_b_y;

    _imu_compensated.acc_z = _config.acc_error_m_z_x * _imu_uncompensated.acc_x +
                             _config.acc_error_m_z_y * _imu_uncompensated.acc_y +
                             _config.acc_error_s_z * _imu_uncompensated.acc_z + _config.acc_error_b_z;
}

void Estimator::_gyro_dynamic_bias_compensation()
{
    if (!_is_gyro_bias_compensated && _n_samples_gyro_bias_compensated < _config.n_samples_gyro_bias_compensation &&
        _is_standstill)
    {
        _gyro_bias_x += _imu_uncompensated.ang_rate_x;
        _gyro_bias_y += _imu_uncompensated.ang_rate_y;
        _gyro_bias_z += _imu_uncompensated.ang_rate_z;

        _n_samples_gyro_bias_compensated++;
    }

    if (!_is_gyro_bias_compensated && _n_samples_gyro_bias_compensated == _config.n_samples_gyro_bias_compensation)
    {
        _gyro_bias_x /= _config.n_samples_gyro_bias_compensation;
        _gyro_bias_y /= _config.n_samples_gyro_bias_compensation;
        _gyro_bias_z /= _config.n_samples_gyro_bias_compensation;

        _is_gyro_bias_compensated = true;
    }

    if (_is_gyro_bias_compensated)
    {
        _imu_compensated.ang_rate_x = _imu_uncompensated.ang_rate_x - _gyro_bias_x;
        _imu_compensated.ang_rate_y = _imu_uncompensated.ang_rate_y - _gyro_bias_y;
        _imu_compensated.ang_rate_z = _imu_uncompensated.ang_rate_z - _gyro_bias_z;
    }
}

void Estimator::_hard_iron_bias_compensation()
{
    _imu_compensated.mag_field_x = _imu_uncompensated.mag_field_x - _config.hard_iron_bias_x;
    _imu_compensated.mag_field_y = _imu_uncompensated.mag_field_y - _config.hard_iron_bias_y;
    _imu_compensated.mag_field_z = _imu_uncompensated.mag_field_z - _config.hard_iron_bias_z;
}
} /* namespace att_est */
