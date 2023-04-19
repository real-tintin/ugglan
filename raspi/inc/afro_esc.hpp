/*
Afro ESC (Electronic Speed Controller), see data sheet
https://rcexplorer.se/wp-content/uploads/2016/03/AFRO-Esc-1.pdf.

Heavily inspired by https://github.com/bluerobotics/Arduino_I2C_ESC.
*/

#ifndef AFRO_ESC_HPP
#define AFRO_ESC_HPP

#include <atomic>
extern "C"
{
#include <math.h>
}

#include <i2c_conn.hpp>
#include <logger.hpp>
#include <wall_time.hpp>

inline const uint8_t AFRO_REG_WRITE_THROTTLE_H = 0x00;
inline const uint8_t AFRO_REG_WRITE_THROTTLE_L = 0x01;
inline const uint8_t AFRO_REG_READ_REV_H = 0x02;
inline const uint8_t AFRO_REG_READ_REV_L = 0x03;
inline const uint8_t AFRO_REG_READ_VBAT_H = 0x04;
inline const uint8_t AFRO_REG_READ_VBAT_L = 0x05;
inline const uint8_t AFRO_REG_READ_TEMP_H = 0x06;
inline const uint8_t AFRO_REG_READ_TEMP_L = 0x07;
inline const uint8_t AFRO_REG_READ_CURRENT_H = 0x08;
inline const uint8_t AFRO_REG_READ_CURRENT_L = 0x09;
inline const uint8_t AFRO_REG_READ_ID = 0x0A;

inline const uint8_t AFRO_IF_ALIVE_BYTE = 0xAB;

inline const uint8_t AFRO_MOTOR_POLES = 7;

inline const double AFRO_VOLTAGE_RESOLUTION = 65535;
inline const double AFRO_CURRENT_RESOLUTION = 65535;

inline const double AFRO_VOLTAGE_SCALE = 32.25; // Derived from (5 * 6.45)
inline const double AFRO_CURRENT_SCALE = 73.53; // Derived from (5 * 14.706)

inline const double AFRO_CURRENT_OFFSET = -32767;

inline const uint8_t AFRO_READ_BUF_SIZE = 9;
inline const uint8_t AFRO_READ_BUF_REV_H = 0x00;
inline const uint8_t AFRO_READ_BUF_REV_L = 0x01;
inline const uint8_t AFRO_READ_BUF_VBAT_H = 0x02;
inline const uint8_t AFRO_READ_BUF_VBAT_L = 0x03;
inline const uint8_t AFRO_READ_BUF_TEMP_H = 0x04;
inline const uint8_t AFRO_READ_BUF_TEMP_L = 0x05;
inline const uint8_t AFRO_READ_BUF_CURRENT_H = 0x06;
inline const uint8_t AFRO_READ_BUF_CURRENT_L = 0x07;
inline const uint8_t AFRO_READ_BUF_ID = 0x08;

inline const uint8_t AFRO_WRITE_BUF_SIZE = 2;
inline const uint8_t AFRO_WRITE_BUF_REV_H = 0x00;
inline const uint8_t AFRO_WRITE_BUF_REV_L = 0x01;

inline const uint8_t AFRO_STATUS_OK = 0x00;
inline const uint8_t AFRO_STATUS_ERR_INIT = 0x01;
inline const uint8_t AFRO_STATUS_NOT_ARMED = 0x02;
inline const uint8_t AFRO_STATUS_ERR_ARM = 0x04;
inline const uint8_t AFRO_STATUS_ERR_WRITE = 0x08;
inline const uint8_t AFRO_STATUS_ERR_READ = 0x10;

class AfroEsc {
  public:
    AfroEsc(I2cConn &i2c_conn);

    void arm();
    void arm_fast();

    void halt();

    void read();
    void write(int16_t motor_cmd);

    bool get_is_alive();

    double get_angular_rate();
    double get_voltage();
    double get_current();
    double get_temperature();

    uint8_t get_status();

  private:
    void _open_i2c_conn();
    void _arm_wake_up();
    void _arm_nudge();
    void _update_rev_timer();
    void _reset_is_alive_byte();

    I2cConn &_i2c_conn;

    uint8_t _buf_read[AFRO_READ_BUF_SIZE] = {0};
    uint8_t _buf_write[AFRO_WRITE_BUF_SIZE] = {0};

    uint32_t _rev_t_ms;
    uint32_t _rev_dt_ms = 0;
    bool _rev_first_sample = true;

    std::atomic<uint8_t> _status = AFRO_STATUS_OK;
};

#endif /* AFRO_ESC_HPP */
