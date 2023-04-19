#include <config.hpp>

namespace config
{
void Config::load()
{
    _env_lines.clear();

    _load_data_log_root();
    _load_logger_level();
    _load_bus_devices();
    _load_zmq_addresses();
    _load_att_est();
    _load_pilot_ctrl();
}

void Config::print_to_debug()
{
    for (auto env_line : _env_lines)
    {
        logger.debug(env_line);
    }
}

void Config::_load_data_log_root()
{
    std::string as_str;
    _read_env(as_str, "DATA_LOG_ROOT");

    _data_log_root = std::filesystem::path(as_str);
}

void Config::_load_logger_level()
{
    std::string as_str;
    _read_env(as_str, "LOGGER_LEVEL");

    _logger_level = _log_str_to_level(as_str);
}

void Config::_load_bus_devices()
{
    _read_env(_serial_device, "SERIAL_DEV_GPIO");

    _read_env(_i2c_device_imu, "I2C_DEV_HW_FAST_MODE");
    _read_env(_i2c_device_esc, "I2C_DEV_SW_NORMAL_MODE");
}

void Config::_load_zmq_addresses()
{
    _read_env(_zmq_address_request, "ZMQ_ADDRESS_REQUEST");
    _read_env(_zmq_address_stream, "ZMQ_ADDRESS_STREAM");
}

void Config::_load_att_est()
{
    _read_env(_att_est.n_samples_gyro_bias_compensation, "ATT_EST_N_SAMPLES_GYRO_BIAS_COMPENSATION");
    _read_env(_att_est.rolling_var_window_size, "ATT_EST_ROLLING_WINDOW_SIZE");

    _read_env(_att_est.kalman_q_scale, "ATT_EST_KALMAN_Q_SCALE");
    _read_env(_att_est.kalman_r_0, "ATT_EST_KALMAN_R_0");
    _read_env(_att_est.kalman_r_1, "ATT_EST_KALMAN_R_1");

    _read_env(_att_est.acc_error_s_x, "ATT_EST_ACC_ERROR_S_X");
    _read_env(_att_est.acc_error_s_y, "ATT_EST_ACC_ERROR_S_Y");
    _read_env(_att_est.acc_error_s_z, "ATT_EST_ACC_ERROR_S_Z");

    _read_env(_att_est.acc_error_m_x_y, "ATT_EST_ACC_ERROR_M_X_Y");
    _read_env(_att_est.acc_error_m_x_z, "ATT_EST_ACC_ERROR_M_X_Z");
    _read_env(_att_est.acc_error_m_y_x, "ATT_EST_ACC_ERROR_M_Y_X");
    _read_env(_att_est.acc_error_m_y_z, "ATT_EST_ACC_ERROR_M_Y_Z");
    _read_env(_att_est.acc_error_m_z_x, "ATT_EST_ACC_ERROR_M_Z_X");
    _read_env(_att_est.acc_error_m_z_y, "ATT_EST_ACC_ERROR_M_Z_Y");

    _read_env(_att_est.acc_error_b_x, "ATT_EST_ACC_ERROR_B_X");
    _read_env(_att_est.acc_error_b_y, "ATT_EST_ACC_ERROR_B_Y");
    _read_env(_att_est.acc_error_b_z, "ATT_EST_ACC_ERROR_B_Z");

    _read_env(_att_est.hard_iron_bias_x, "ATT_EST_HARD_IRON_BIAS_X");
    _read_env(_att_est.hard_iron_bias_y, "ATT_EST_HARD_IRON_BIAS_Y");
    _read_env(_att_est.hard_iron_bias_z, "ATT_EST_HARD_IRON_BIAS_Z");
}

void Config::_load_pilot_ctrl()
{
    _read_env(_pilot_ctrl.anti_windup_sat_phi, "PILOT_CTRL_ANTI_WINDUP_SAT_PHI");
    _read_env(_pilot_ctrl.anti_windup_sat_theta, "PILOT_CTRL_ANTI_WINDUP_SAT_THETA");
    _read_env(_pilot_ctrl.anti_windup_sat_phi, "PILOT_CTRL_ANTI_WINDUP_SAT_PSI");

    _read_env(_pilot_ctrl.L_roll[0], "PILOT_CTRL_L_ROLL_0");
    _read_env(_pilot_ctrl.L_roll[1], "PILOT_CTRL_L_ROLL_1");
    _read_env(_pilot_ctrl.L_roll[2], "PILOT_CTRL_L_ROLL_2");
    _read_env(_pilot_ctrl.L_roll[3], "PILOT_CTRL_L_ROLL_3");

    _read_env(_pilot_ctrl.L_pitch[0], "PILOT_CTRL_L_PITCH_0");
    _read_env(_pilot_ctrl.L_pitch[1], "PILOT_CTRL_L_PITCH_1");
    _read_env(_pilot_ctrl.L_pitch[2], "PILOT_CTRL_L_PITCH_2");
    _read_env(_pilot_ctrl.L_pitch[3], "PILOT_CTRL_L_PITCH_3");

    _read_env(_pilot_ctrl.L_yaw_rate[0], "PILOT_CTRL_L_YAW_RATE_0");
    _read_env(_pilot_ctrl.L_yaw_rate[1], "PILOT_CTRL_L_YAW_RATE_1");
    _read_env(_pilot_ctrl.L_yaw_rate[2], "PILOT_CTRL_L_YAW_RATE_2");
}

LogLevel Config::_log_str_to_level(std::string log_level)
{
    if (log_level == "DEBUG")
    {
        return LogLevel::debug;
    }
    else if (log_level == "INFO")
    {
        return LogLevel::info;
    }
    else if (log_level == "WARN")
    {
        return LogLevel::warn;
    }
    else if (log_level == "ERROR")
    {
        return LogLevel::error;
    }
    else if (log_level == "OFF")
    {
        return LogLevel::off;
    }
    else
    {
        return logger.get_level();
    }
}
} /* namespace config */
