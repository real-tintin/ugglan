#include <afro_esc.h>

AfroEsc::AfroEsc(I2cConn& i2c_conn) :
    _i2c_conn(i2c_conn)
{
    _open_i2c_conn();
    _arm();
}

void AfroEsc::read()
{
    _reset_is_alive_byte();

    bool did_read = _i2c_conn.read_block_data(AFRO_READ_REV_H, AFRO_READ_BUF_SIZE, _buf_read);

    if (did_read)
    {
        _update_rpm_timer();
        _status &= ~AFRO_STATUS_ERR_READ;
    }
    else
    {
        _status |= AFRO_STATUS_ERR_READ;
    }
}

void AfroEsc::write(int16_t command)
{
    _buf_write[0] = command >> 8;
    _buf_write[1] = command;

    bool did_write = _i2c_conn.write_block_data(AFRO_WRITE_THROTTLE_H, AFRO_WRITE_BUF_SIZE, _buf_write);

    if (did_write)
    {
        _status &= ~AFRO_STATUS_ERR_WRITE;
    }
    else
    {
        _status |= AFRO_STATUS_ERR_WRITE;
    }
}

bool AfroEsc::get_is_alive()
{
    return (_buf_read[AFRO_READ_BUF_ID] == AFRO_IF_ALIVE_BYTE);
}

// Returns voltage [V]
double AfroEsc::get_voltage()
{
    uint16_t raw_voltage = (_buf_read[AFRO_READ_BUF_VBAT_H] << 8) | _buf_read[AFRO_READ_BUF_VBAT_L];
    return double(raw_voltage) * AFRO_VOLTAGE_SCALE / AFRO_VOLTAGE_RESOLUTION;
}

// Returns current [A]
double AfroEsc::get_current()
{
    uint16_t raw_current = (_buf_read[AFRO_READ_BUF_CURRENT_H] << 8) | _buf_read[AFRO_READ_BUF_CURRENT_L];
    return (double(raw_current) + AFRO_CURRENT_OFFSET) * AFRO_CURRENT_SCALE / AFRO_CURRENT_RESOLUTION;
}

// Returns temperature [C]
double AfroEsc::get_temperature()
{
    uint16_t raw_temperature = (_buf_read[AFRO_READ_BUF_TEMP_H] << 8) | _buf_read[AFRO_READ_BUF_TEMP_L];
    double resistance = AFRO_SERIESRESISTOR / (65535 / double(raw_temperature) - 1);

    double steinhart;
    steinhart = resistance / AFRO_THERMISTORNOMINAL;       // (R/Ro)
    steinhart = log(steinhart);                            // ln(R/Ro)
    steinhart /= AFRO_BCOEFFICIENT;                        // 1/B * ln(R/Ro)
    steinhart += 1.0 / (AFRO_TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                           // Invert
    steinhart -= 273.15;                                   // Convert to [C]

    return steinhart;
}

// Returns RPM [rpm]
double AfroEsc::get_rpm()
{
    int16_t raw_rpm = (_buf_read[AFRO_READ_BUF_REV_H] << 8) | _buf_read[AFRO_READ_BUF_REV_L];
    double rpm = double(raw_rpm) / (double(_rpm_dt_ms) / AFRO_MS_IN_MIN) / double(AFRO_MOTOR_POLES);

    return rpm;
}

uint8_t AfroEsc::get_status()
{
    return _status;
}

void AfroEsc::_open_i2c_conn()
{
    if (!_i2c_conn.open())
    {
        _status = AFRO_STATUS_ERR_INIT;
    }
    else
    {
        _status = AFRO_STATUS_NOT_ARMED;
    }
}

void AfroEsc::_arm()
{
    // First arm for AFRO_ARM_TIME_MS.
    uint32_t arm_t_ms = wall_time.millis();
    while (AFRO_ARM_TIME_MS > (wall_time.millis() - arm_t_ms)) { write(0U); }

    // Then try to turn motor and check if alive for AFRO_TURN_TIME_MS.
    uint32_t turn_t_ms = wall_time.millis();
    while ((AFRO_TURN_TIME_MS > (wall_time.millis() - turn_t_ms)) && !get_is_alive())
    {
        write(1U);
        read();
    }
    write(0U);

    if (get_is_alive())
    {
        logger.debug("Arming ESC successful.");
        _status = AFRO_STATUS_OK;
    }
    else
    {
        logger.error("Arming ESC failed.");
        _status = AFRO_STATUS_ERR_ARM;
    }
}

void AfroEsc::_update_rpm_timer()
{
    double t_now_ms = wall_time.millis();

    _rpm_dt_ms = t_now_ms - _rpm_t_ms;
    _rpm_t_ms = t_now_ms;
}

void AfroEsc::_reset_is_alive_byte()
{
    _buf_read[AFRO_READ_BUF_ID] = 0x00;
}
