/*
LPS25H pressure sensor, see data sheet
https://www.pololu.com/file/download/LPS25H.pdf?file_id=0J761
*/

#ifndef LPS25H_H
#define LPS25H_H

#include <cstdint>
#include <pololu_alt_imu.h>

static const uint8_t LPS25H_I2C_ADDRESS = 0x5D;

// Register addresses
static const uint8_t LPS25H_REF_P_XL     = 0x08;
static const uint8_t LPS25H_REF_P_L      = 0x09;
static const uint8_t LPS25H_REF_P_H      = 0x0A;
static const uint8_t LPS25H_WHO_AM_I     = 0x0F;
static const uint8_t LPS25H_RES_CONF     = 0x10;
static const uint8_t LPS25H_CTRL_REG1    = 0x20;
static const uint8_t LPS25H_CTRL_REG2    = 0x21;
static const uint8_t LPS25H_CTRL_REG3    = 0x22;
static const uint8_t LPS25H_CTRL_REG4    = 0x23;
static const uint8_t LPS25H_INT_CFG      = 0x24;
static const uint8_t LPS25H_INT_SOURCE   = 0x25;
static const uint8_t LPS25H_STATUS_REG   = 0x27;
static const uint8_t LPS25H_PRESS_OUT_XL = 0x28;
static const uint8_t LPS25H_PRESS_OUT_L  = 0x29;
static const uint8_t LPS25H_PRESS_OUT_H  = 0x2A;
static const uint8_t LPS25H_TEMP_OUT_P_L = 0x2B;
static const uint8_t LPS25H_TEMP_OUT_P_H = 0x2C;
static const uint8_t LPS25H_FIFO_CTRL    = 0x2E;
static const uint8_t LPS25H_FIFO_STATUS  = 0x2F;
static const uint8_t LPS25H_THS_P_L      = 0x30;
static const uint8_t LPS25H_THS_P_H      = 0x31;
static const uint8_t LPS25H_RPDS_L       = 0x39;
static const uint8_t LPS25H_RPDS_H       = 0x3A;

// Buffer indices and size
static const uint8_t LPS25H_BUF_PRES_SIZE    = 3;
static const uint8_t LPS25H_BUF_PRESS_OUT_XL = 0x00;
static const uint8_t LPS25H_BUF_PRESS_OUT_L  = 0x01;
static const uint8_t LPS25H_BUF_PRESS_OUT_H  = 0x02;

static const uint8_t LPS25H_BUF_TEMP_SIZE    = 2;
static const uint8_t LPS25H_BUF_TEMP_OUT_P_L = 0x00;
static const uint8_t LPS25H_BUF_TEMP_OUT_P_H = 0x01;

// Scaling, resolution and offset
static const double LPS25H_PRES_SCALE = 100; // Convert from hPa to Pa
static const double LPS25H_PRES_RESOLUTION = 4096;

static const double LPS25H_TEMP_RESOLUTION = 480;
static const double LPS25H_TEMP_OFFSET = 42.5; // [C]

// Config and read map
static const ConfigMap LPS25H_CONFIG_MAP = {
    {LPS25H_CTRL_REG1, 0b11000000}, // Enable pressure and temperature for continues output at 25 Hz
    {LPS25H_RES_CONF, 0b00001111}  // Set highest resolution on pressure and temperature
    };

static const ReadMap LPS25H_READ_MAP = {
    {LPS25H_PRESS_OUT_XL, LPS25H_BUF_PRES_SIZE},
    {LPS25H_TEMP_OUT_P_L, LPS25H_BUF_TEMP_SIZE},
    };

class Lps25h : public PololuAltImu
{
public:
    Lps25h(I2cConn* i2c_conn);

    double get_pressure();
    double get_temperature();
};

#endif
