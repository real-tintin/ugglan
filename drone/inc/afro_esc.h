#ifndef AFRO_ESC_H
#define AFRO_ESC_H

#include <math.h>
#include <wall_time.h>

#if defined(UNIT_TEST)
#include <i2c_conn_stub.h>
#else
#include <i2c_conn.h>
#endif

static const uint8_t AFRO_WRITE_THROTTLE_H = 0x00;
static const uint8_t AFRO_WRITE_THROTTLE_L = 0x01;
static const uint8_t AFRO_READ_REV_H       = 0x02;
static const uint8_t AFRO_READ_REV_L       = 0x03;
static const uint8_t AFRO_READ_VBAT_H      = 0x04;
static const uint8_t AFRO_READ_VBAT_L      = 0x05;
static const uint8_t AFRO_READ_TEMP_H      = 0x06;
static const uint8_t AFRO_READ_TEMP_L      = 0x07;
static const uint8_t AFRO_READ_CURRENT_H   = 0x08;
static const uint8_t AFRO_READ_CURRENT_L   = 0x09;
static const uint8_t AFRO_READ_ID          = 0x0A;

static const uint8_t AFRO_IF_ALIVE_BYTE = 0xab;

static const uint8_t AFRO_MOTOR_POLES = 7;

static const double AFRO_THERMISTORNOMINAL  = 10000; // Resistance at 25 degrees C.
static const double AFRO_TEMPERATURENOMINAL = 25;    // Temperature for nominal resistance [C].
static const double AFRO_BCOEFFICIENT       = 3900;  // The beta coefficient of the thermistor (usually 3000-4000).
static const double AFRO_SERIESRESISTOR     = 3300;  // The value of the 'other' resistor.

static const uint8_t AFRO_READ_BUF_SIZE      = 9;
static const uint8_t AFRO_READ_BUF_REV_H     = 0x00;
static const uint8_t AFRO_READ_BUF_REV_L     = 0x01;
static const uint8_t AFRO_READ_BUF_VBAT_H    = 0x02;
static const uint8_t AFRO_READ_BUF_VBAT_L    = 0x03;
static const uint8_t AFRO_READ_BUF_TEMP_H    = 0x04;
static const uint8_t AFRO_READ_BUF_TEMP_L    = 0x05;
static const uint8_t AFRO_READ_BUF_CURRENT_H = 0x06;
static const uint8_t AFRO_READ_BUF_CURRENT_L = 0x07;
static const uint8_t AFRO_READ_BUF_ID        = 0x08;

static const uint8_t AFRO_WRITE_BUF_SIZE  = 2;
static const uint8_t AFRO_WRITE_BUF_REV_H = 0x00;
static const uint8_t AFRO_WRITE_BUF_REV_L = 0x01;

static const uint8_t AFRO_STATUS_OK        = 0x00;
static const uint8_t AFRO_STATUS_ERR_INIT  = 0x01;
static const uint8_t AFRO_STATUS_NOT_ARMED = 0x02;
static const uint8_t AFRO_STATUS_ERR_ARM   = 0x04;
static const uint8_t AFRO_STATUS_ERR_WRITE = 0x08;
static const uint8_t AFRO_STATUS_ERR_READ  = 0x10;

static const uint16_t AFRO_ARM_TIME_MS  = 50;
static const uint16_t AFRO_TURN_TIME_MS = 100;

static const double AFRO_MS_IN_MIN = 60000.0;

class AfroEsc
{
public:
    AfroEsc(I2cConn* i2c_conn);

    void read();
    void write(int16_t command);

    int16_t get_rpm();
    double get_voltage();
    double get_current();
    double get_temperature();

    uint8_t get_status();
private:
    void _arm();
    void _open_i2c_conn();
    void _update_rpm_timer();

    uint8_t _is_alive();

    I2cConn* _i2c_conn;

    uint8_t _buf_read[AFRO_READ_BUF_SIZE] = {0};
    uint8_t _buf_write[AFRO_WRITE_BUF_SIZE] = {0};

    double _rpm_t_ms = wall_time.millis();
    double _rpm_dt_ms = 0.0;

    uint8_t _status = AFRO_STATUS_OK;
};

#endif /* AFRO_ESC_H */
