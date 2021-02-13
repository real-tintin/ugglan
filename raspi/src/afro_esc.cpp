#include <afro_esc.h>

inline const double THERMISTOR_NOMINAL  = 10000; // Resistance at 25 degrees [C]
inline const double TEMPERATURE_NOMINAL = 25;    // Temperature for nominal resistance [C]
inline const double B_COEFFICIENT       = 3900;  // The beta coefficient of the thermistor (usually 3000-4000)
inline const double SERIES_RESISTOR     = 3300;  // The value of the 'other' resistor

static const uint16_t WAKE_UP_TIME_MS = 50;
static const uint16_t NUDGE_TIME_MS   = 100;

static const double MS_IN_S = 1000.0;

AfroEsc::AfroEsc(I2cConn& i2c_conn) :
    _i2c_conn(i2c_conn)
{
    _open_i2c_conn();
}

void AfroEsc::arm()
{
    _arm_wake_up();
    _arm_nudge();

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

void AfroEsc::arm_fast() { write(0U); }

void AfroEsc::read()
{
    _reset_is_alive_byte();

    bool did_read = _i2c_conn.read_block_data(AFRO_REG_READ_REV_H, AFRO_READ_BUF_SIZE, _buf_read);

    if (did_read)
    {
        _update_rev_timer();
        _status &= ~AFRO_STATUS_ERR_READ;
    }
    else
    {
        _status |= AFRO_STATUS_ERR_READ;
    }
}

void AfroEsc::write(int16_t motor_cmd)
{
    _buf_write[0] = motor_cmd >> 8;
    _buf_write[1] = motor_cmd;

    bool did_write = _i2c_conn.write_block_data(AFRO_REG_WRITE_THROTTLE_H, AFRO_WRITE_BUF_SIZE, _buf_write);

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
    double resistance = SERIES_RESISTOR / (65535 / double(raw_temperature) - 1);

    double steinhart;
    steinhart = resistance / THERMISTOR_NOMINAL;       // (R/Ro)
    steinhart = log(steinhart);                        // ln(R/Ro)
    steinhart /= B_COEFFICIENT;                        // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                       // Invert
    steinhart -= 273.15;                               // Convert to [C]

    return steinhart;
}

// Returns angular rate [rad/s]
double AfroEsc::get_angular_rate()
{
    int16_t raw_rev = (_buf_read[AFRO_READ_BUF_REV_H] << 8) | _buf_read[AFRO_READ_BUF_REV_L];
    double angular_rate = 0;

    if (_rev_dt_ms > 0)
    {
        angular_rate = (double(raw_rev) * MS_IN_S * 2.0 * M_PI) / (double(_rev_dt_ms) * double(AFRO_MOTOR_POLES));
    }

    return angular_rate;
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

void AfroEsc::_arm_wake_up()
{
    uint32_t wake_up_t_ms = wall_time.millis();

    while (WAKE_UP_TIME_MS > (wall_time.millis() - wake_up_t_ms))
    {
        write(0);
    }
}

void AfroEsc::_arm_nudge()
{
    uint32_t nudge_t_ms = wall_time.millis();

    while ((NUDGE_TIME_MS > (wall_time.millis() - nudge_t_ms)) && !get_is_alive())
    {
        write(1U);
        read();
    }

    write(0U);
}

void AfroEsc::_update_rev_timer()
{
    uint32_t t_now_ms = wall_time.millis();

    if (!_rev_first_sample) { _rev_dt_ms = t_now_ms - _rev_t_ms; }
    else { _rev_first_sample = false; }

    _rev_t_ms = t_now_ms;
}

void AfroEsc::_reset_is_alive_byte()
{
    _buf_read[AFRO_READ_BUF_ID] = 0x00;
}
