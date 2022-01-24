#include <attitude_estimation.h>

static const double MODULO_ROLL = M_PI;
static const double MODULO_PITCH = M_PI / 2;
static const double MODULO_YAW = M_PI;

static const double STANDSTILL_ANG_VAR_LIM = 0.02; // [rad]
static const double STANDSTILL_RATE_VAR_LIM = 0.005; // [rad/s]

static const double KALMAN_R_ALMOST_ZERO = 1e-6;

AttitudeEstimation::AttitudeEstimation(double input_sample_rate_s, AttEstConfig config) :
    _dt(input_sample_rate_s),
    _config(config),

    _rolling_stats_roll_angle(config.rolling_var_window_size),
    _rolling_stats_pitch_angle(config.rolling_var_window_size),
    _rolling_stats_yaw_angle(config.rolling_var_window_size),

    _rolling_stats_roll_rate(config.rolling_var_window_size),
    _rolling_stats_pitch_rate(config.rolling_var_window_size),
    _rolling_stats_yaw_rate(config.rolling_var_window_size)
{
    _Q = _config.kalman_q_scale * Eigen::Matrix3d({
        {0.25 * pow(_dt, 4), 0.5 * pow(_dt, 3), 0.5 * pow(_dt, 2)},
        {0.5 * pow(_dt, 3), pow(_dt, 2), _dt},
        {0.5 * pow(_dt, 2), _dt, 1}
        });

    _F = Eigen::Matrix3d({
        {1, _dt, 0.5 * pow(_dt, 2)},
        {0, 1, _dt},
        {0, 0, 1}
        });
    _F_t = _F.transpose();
}

void AttitudeEstimation::update(AttEstInput input)
{
    _in = input;

    _hard_iron_offset_comp();

    _update_imu_angles();
    _update_rolling_stats();
    _update_standstill_status();

    _gyro_offset_comp();

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

bool AttitudeEstimation::is_standstill()
{
    return _is_standstill;
}

void AttitudeEstimation::_update_imu_angles()
{
    _imu_roll_angle = atan2(-_in.acc_y, -_in.acc_z);
    _imu_pitch_angle = atan2(_in.acc_x, sqrt(pow(_in.acc_y, 2) + pow(_in.acc_z, 2)));

    double b_x = _in.mag_field_x * cos(_est.pitch.angle) +
                  _in.mag_field_y * sin(_est.roll.angle) * sin(_est.pitch.angle) +
                  _in.mag_field_z * sin(_est.pitch.angle) * cos(_est.roll.angle);
    double b_y = _in.mag_field_y * cos(_est.roll.angle) - _in.mag_field_z * sin(_est.roll.angle);

    _imu_yaw_angle = atan2(-b_y, b_x);
}

void AttitudeEstimation::_update_rolling_stats()
{
    _rolling_stats_roll_angle.update(_imu_roll_angle);
    _rolling_stats_pitch_angle.update(_imu_pitch_angle);
    _rolling_stats_yaw_angle.update(_imu_yaw_angle);

    _rolling_stats_roll_rate.update(_in.ang_rate_x);
    _rolling_stats_pitch_rate.update(_in.ang_rate_y);
    _rolling_stats_yaw_rate.update(_in.ang_rate_z);
}

void AttitudeEstimation::_update_standstill_status()
{
    if
    (
        (
        _rolling_stats_roll_angle.estimates_available() &&
        _rolling_stats_pitch_angle.estimates_available() &&
        _rolling_stats_yaw_angle.estimates_available() &&

        _rolling_stats_roll_rate.estimates_available() &&
        _rolling_stats_pitch_rate.estimates_available() &&
        _rolling_stats_yaw_rate.estimates_available()
        ) &&
        (
        (_rolling_stats_roll_angle.get_variance() < STANDSTILL_ANG_VAR_LIM) &&
        (_rolling_stats_pitch_angle.get_variance() < STANDSTILL_ANG_VAR_LIM) &&
        (_rolling_stats_yaw_angle.get_variance() < STANDSTILL_ANG_VAR_LIM) &&

        (_rolling_stats_roll_rate.get_variance() < STANDSTILL_RATE_VAR_LIM) &&
        (_rolling_stats_pitch_rate.get_variance() < STANDSTILL_RATE_VAR_LIM) &&
        (_rolling_stats_yaw_rate.get_variance() < STANDSTILL_RATE_VAR_LIM)
        )
    )
    {
        _is_standstill = true;
    }
}

void AttitudeEstimation::_update_roll()
{
    _update_est(
        _imu_roll_angle,
        _in.ang_rate_x,
        _rolling_stats_roll_angle.get_variance(),
        _rolling_stats_roll_rate.get_variance(),
        _kalman_roll,
        _est.roll,
        MODULO_ROLL
        );
}

void AttitudeEstimation::_update_pitch()
{
    _update_est(
        _imu_pitch_angle,
        _in.ang_rate_y,
        _rolling_stats_pitch_angle.get_variance(),
        _rolling_stats_pitch_rate.get_variance(),
        _kalman_pitch,
        _est.pitch,
        MODULO_PITCH
        );
}

void AttitudeEstimation::_update_yaw()
{
    _update_est(
        _imu_yaw_angle,
        _in.ang_rate_z,
        _rolling_stats_yaw_angle.get_variance(),
        _rolling_stats_yaw_rate.get_variance(),
        _kalman_yaw,
        _est.yaw,
        MODULO_YAW
        );
}

void AttitudeEstimation::_update_est(double z_0, double z_1,
                                     double r_0, double r_1,
                                     AttEstKalmanState& kalman_state,
                                     AttEstState& att_state,
                                     double modulo_lim)
{
    kalman_state.z(0) = z_0;
    kalman_state.z(1) = z_1;

    kalman_state.R(0, 0) = _config.kalman_r_0_scale * std::max(r_0, KALMAN_R_ALMOST_ZERO);
    kalman_state.R(1, 1) = _config.kalman_r_1_scale * std::max(r_1, KALMAN_R_ALMOST_ZERO);

    _update_kalman_state(kalman_state);
    _kalman_state_to_att_state(kalman_state, att_state);
    _modulo_angle(&att_state.angle, modulo_lim);
}

void AttitudeEstimation::_update_kalman_state(AttEstKalmanState& state)
{
    _x_pri.noalias() = _F * state.x;
    _P_pri.noalias() = _F * state.P * _F_t + _Q;

    _S.noalias() = _H * _P_pri * _H_t + state.R;
    _K.noalias() = _P_pri * _H_t * _S.inverse();

    state.x.noalias() = _x_pri + _K * (state.z - _H * _x_pri);
    state.P.noalias() = (_I - _K * _H) * _P_pri;
}

void AttitudeEstimation::_kalman_state_to_att_state(AttEstKalmanState& kalman, AttEstState& att)
{
    att.angle = kalman.x(0);
    att.rate = kalman.x(1);
    att.acc = kalman.x(2);
}

void AttitudeEstimation::_modulo_angle(double* angle, double limit)
{
    *angle = fmod(*angle + limit, 2.0 * limit) - limit;
}

void AttitudeEstimation::_gyro_offset_comp()
{
    if (!_is_gyro_offset_comp &&
        _samples_gyro_offset_comp < _config.n_samples_gyro_offset_comp &&
        _is_standstill)
    {
        _gyro_offset_x += _in.ang_rate_x;
        _gyro_offset_y += _in.ang_rate_y;
        _gyro_offset_z += _in.ang_rate_z;

        _samples_gyro_offset_comp++;
    }

    if (!_is_gyro_offset_comp &&
        _samples_gyro_offset_comp == _config.n_samples_gyro_offset_comp)
    {
        _gyro_offset_x /= _config.n_samples_gyro_offset_comp;
        _gyro_offset_y /= _config.n_samples_gyro_offset_comp;
        _gyro_offset_z /= _config.n_samples_gyro_offset_comp;

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
    _in.mag_field_x -= _config.hard_iron_offset_x;
    _in.mag_field_y -= _config.hard_iron_offset_y;
    _in.mag_field_z -= _config.hard_iron_offset_z;
}
