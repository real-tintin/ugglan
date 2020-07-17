#include "afro_esc.h"

AfroEsc::AfroEsc(I2cConn* i2c_conn)
{
    _open_i2c_conn();
    _arm();
}

void AfroEsc::read()
{
    bool did_read = false;
    _buf_read[AFRO_READ_BUF_ID] = 0x00; // Reset AFRO_READ_BUF_ID (to check for alive) TODO: Break-out?

    did_read |= _i2c_conn->read_block_data(AFRO_READ_REV_H, AFRO_READ_BUF_SIZE, _buf_read);

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
    bool did_write = false;

    _buf_write[0] = command >> 8;
    _buf_write[1] = command;

    did_write |= _i2c_conn->write_block_data(AFRO_WRITE_THROTTLE_H, AFRO_WRITE_BUF_SIZE, _buf_write);

    if (did_write)
    {
        _status &= ~AFRO_STATUS_ERR_WRITE;
    }
    else
    {
        _status |= AFRO_STATUS_ERR_WRITE;
    }
}

// Returns voltage [V]
double AfroEsc::get_voltage()
{
    uint16_t raw_voltage = (_buf_read[AFRO_READ_BUF_VBAT_H] << 8) | _buf_read[AFRO_READ_BUF_VBAT_L];
    return double(raw_voltage) / 65536.0l * 5.0l * 6.45l;
}

// Returns current [A]
double AfroEsc::get_current()
{
    uint16_t raw_current = (_buf_read[AFRO_READ_BUF_CURRENT_H] << 8) | _buf_read[AFRO_READ_BUF_CURRENT_L];
    return (double(raw_current) - 32767) / 65535.0l * 5.0l * 14.706l;
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
    steinhart -= 273.15;                                   // convert to C

    return steinhart;
}

// Returns RPM [rpm]
int16_t AfroEsc::get_rpm()
{
    int16_t raw_rpm = (_buf_read[AFRO_READ_BUF_REV_H] << 8) | _buf_read[AFRO_READ_BUF_REV_L];
    double rpm = double(raw_rpm) / (double(_rpm_dt_ms) / AFRO_MS_IN_MIN) / double(AFRO_MOTOR_POLES);

    return int16_t(rpm);
}

uint8_t AfroEsc::get_status()
{
    return _status;
}

void AfroEsc::_open_i2c_conn()
{
    if (!_i2c_conn->open())
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
    bool was_alive = false;

    // First arm ESC for AFRO_ARM_TIME_MS.
    double arm_t_ms = wall_time.millis();
    while (AFRO_ARM_TIME_MS > (wall_time.millis() - arm_t_ms)) { write(0); }

    // Then try to turn motor and check if alive.
    double turn_t_ms = wall_time.millis();
    while ((AFRO_TURN_TIME_MS > (wall_time.millis() - turn_t_ms)) && !was_alive)
    {
        write(1);
        read();

        if (_is_alive())
        {
            was_alive = true;
        }
    }
    write(0);

    // Finally check if successful.
    if (was_alive)
    {
        _status = AFRO_STATUS_OK;
    }
    else
    {
        _status = AFRO_STATUS_ERR_ARM;
    }
}

uint8_t AfroEsc::_is_alive()
{
    return (_buf_read[AFRO_READ_BUF_ID] == AFRO_IF_ALIVE_BYTE);
}

void AfroEsc::_update_rpm_timer()
{
    double t_now_ms = wall_time.millis();

    _rpm_dt_ms = t_now_ms - _rpm_t_ms;
    _rpm_t_ms = t_now_ms;
}
