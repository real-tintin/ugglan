#include <attitude_estimation.h>

static const double MODULO_ROLL = M_PI;
static const double MODULO_PITCH = M_PI / 2;
static const double MODULO_YAW = M_PI;

static const double STANDSTILL_ANG_VAR_LIM = 0.02; // [rad]
static const double STANDSTILL_RATE_VAR_LIM = 0.005; // [rad/s]

static const double KALMAN_R_ALMOST_ZERO = 1e-6;

AttitudeEstimation::AttitudeEstimation(double input_sample_rate_s) :
    _dt(input_sample_rate_s)
{
    _Q = ATT_EST_KALMAN_Q_SCALE * Eigen::Matrix3d({
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

    _lp_filter_acc_x.setup(1 / _dt, ATT_EST_LP_BUTTER_ACC_CF);
    _lp_filter_acc_y.setup(1 / _dt, ATT_EST_LP_BUTTER_ACC_CF);
    _lp_filter_acc_z.setup(1 / _dt, ATT_EST_LP_BUTTER_ACC_CF);
}

void AttitudeEstimation::update(AttEstInput input)
{
    _in = input;

    _hard_iron_offset_comp();
    _lp_filter_acc();

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

AttEstPointSample AttitudeEstimation::get_lp_filtered_acc()
{
    return _lp_filtered_acc;
}

bool AttitudeEstimation::is_calibrated()
{
    return _is_gyro_offset_comp;
}

bool AttitudeEstimation::is_standstill()
{
    return _is_standstill;
}

void AttitudeEstimation::_lp_filter_acc()
{
    _lp_filtered_acc.x = _lp_filter_acc_x.filter(_in.acc.x);
    _lp_filtered_acc.y = _lp_filter_acc_y.filter(_in.acc.y);
    _lp_filtered_acc.z = _lp_filter_acc_z.filter(_in.acc.z);
}

void AttitudeEstimation::_update_imu_angles()
{
    _imu_roll_angle = atan2(-_lp_filtered_acc.y, -_lp_filtered_acc.z);
    _imu_pitch_angle = atan2(_lp_filtered_acc.x, sqrt(pow(_lp_filtered_acc.y, 2) + pow(_lp_filtered_acc.z, 2)));

    double b_x = _in.mag_field.x * cos(_est.pitch.angle) +
                  _in.mag_field.y * sin(_est.roll.angle) * sin(_est.pitch.angle) +
                  _in.mag_field.z * sin(_est.pitch.angle) * cos(_est.roll.angle);
    double b_y = _in.mag_field.y * cos(_est.roll.angle) - _in.mag_field.z * sin(_est.roll.angle);

    _imu_yaw_angle = atan2(-b_y, b_x);
}

void AttitudeEstimation::_update_rolling_stats()
{
    _rolling_stats_roll_angle.update(_imu_roll_angle);
    _rolling_stats_pitch_angle.update(_imu_pitch_angle);
    _rolling_stats_yaw_angle.update(_imu_yaw_angle);

    _rolling_stats_roll_rate.update(_in.ang_rate.x);
    _rolling_stats_pitch_rate.update(_in.ang_rate.y);
    _rolling_stats_yaw_rate.update(_in.ang_rate.z);
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
        _in.ang_rate.x,
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
        _in.ang_rate.y,
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
        _in.ang_rate.z,
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

    kalman_state.R(0, 0) = ATT_EST_KALMAN_R_0_SCALE * std::max(r_0, KALMAN_R_ALMOST_ZERO);
    kalman_state.R(1, 1) = ATT_EST_KALMAN_R_1_SCALE * std::max(r_1, KALMAN_R_ALMOST_ZERO);

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
        _samples_gyro_offset_comp < ATT_EST_N_SAMPLES_GYRO_OFFSET_COMP &&
        _is_standstill)
    {
        _gyro_offset.x += _in.ang_rate.x;
        _gyro_offset.y += _in.ang_rate.y;
        _gyro_offset.z += _in.ang_rate.z;

        _samples_gyro_offset_comp++;
    }

    if (!_is_gyro_offset_comp &&
        _samples_gyro_offset_comp == ATT_EST_N_SAMPLES_GYRO_OFFSET_COMP)
    {
        _gyro_offset.x /= ATT_EST_N_SAMPLES_GYRO_OFFSET_COMP;
        _gyro_offset.y /= ATT_EST_N_SAMPLES_GYRO_OFFSET_COMP;
        _gyro_offset.z /= ATT_EST_N_SAMPLES_GYRO_OFFSET_COMP;

        _is_gyro_offset_comp = true;
    }

    if (_is_gyro_offset_comp)
    {
        _in.ang_rate.x -= _gyro_offset.x;
        _in.ang_rate.y -= _gyro_offset.y;
        _in.ang_rate.z -= _gyro_offset.z;
    }
}

void AttitudeEstimation::_hard_iron_offset_comp()
{
    _in.mag_field.x -= ATT_EST_HARD_IRON_OFFSET_X;
    _in.mag_field.y -= ATT_EST_HARD_IRON_OFFSET_Y;
    _in.mag_field.z -= ATT_EST_HARD_IRON_OFFSET_Z;
}
