/* L3GD20H gyro sensor, see data sheet
https://www.pololu.com/file/download/L3GD20H.pdf?file_id=0J731
*/

#ifndef L3GD20H_H
#define L3GD20H_H

#include <pololu_alt_imu.h>

static const uint8_t L3GD20H_I2C_ADDRESS = 0x6B;

// Register addresses
static const uint8_t L3GD20H_WHO_AM_I    = 0x0F;
static const uint8_t L3GD20H_CTRL1       = 0x20;
static const uint8_t L3GD20H_CTRL2       = 0x21;
static const uint8_t L3GD20H_CTRL3       = 0x22;
static const uint8_t L3GD20H_CTRL4       = 0x23;
static const uint8_t L3GD20H_CTRL5       = 0x24;
static const uint8_t L3GD20H_REFERENCE   = 0x25;
static const uint8_t L3GD20H_OUT_TEMP    = 0x26;
static const uint8_t L3GD20H_STATUS      = 0x27;
static const uint8_t L3GD20H_OUT_X_L     = 0x28;
static const uint8_t L3GD20H_OUT_X_H     = 0x29;
static const uint8_t L3GD20H_OUT_Y_L     = 0x2A;
static const uint8_t L3GD20H_OUT_Y_H     = 0x2B;
static const uint8_t L3GD20H_OUT_Z_L     = 0x2C;
static const uint8_t L3GD20H_OUT_Z_H     = 0x2D;
static const uint8_t L3GD20H_FIFO_CTRL   = 0x2E;
static const uint8_t L3GD20H_FIFO_SRC    = 0x2F;
static const uint8_t L3GD20H_IG_CFG      = 0x30;
static const uint8_t L3GD20H_IG_SRC      = 0x31;
static const uint8_t L3GD20H_IG_THS_XH   = 0x32;
static const uint8_t L3GD20H_IG_THS_XL   = 0x33;
static const uint8_t L3GD20H_IG_THS_YH   = 0x34;
static const uint8_t L3GD20H_IG_THS_YL   = 0x35;
static const uint8_t L3GD20H_IG_THS_ZH   = 0x36;
static const uint8_t L3GD20H_IG_THS_ZL   = 0x37;
static const uint8_t L3GD20H_IG_DURATION = 0x38;
static const uint8_t L3GD20H_LOW_ODR     = 0x39;

// Buffer indices and size
static const uint8_t L3GD20H_BUFFER_SIZE    = 6;
static const uint8_t L3GD20H_BUFFER_OUT_X_L = 0x00;
static const uint8_t L3GD20H_BUFFER_OUT_X_H = 0x01;
static const uint8_t L3GD20H_BUFFER_OUT_Y_L = 0x02;
static const uint8_t L3GD20H_BUFFER_OUT_Y_H = 0x03;
static const uint8_t L3GD20H_BUFFER_OUT_Z_L = 0x04;
static const uint8_t L3GD20H_BUFFER_OUT_Z_H = 0x05;

// Scaling and resolution
static const double L3GD20H_GYRO_SCALE = 500; // [deg/s]
static const double L3GD20H_GYRO_RESOLUTION = 32767; // 2^15-1 (16-bit signed integer)

// Config and read map
static const ConfigMap L3GD20H_CONFIG_MAP = {
    {L3GD20H_CTRL1, 0b01011111}, // Enable XYZ gyro axis at 200 Hz, normal mode
    {L3GD20H_CTRL4, 0b00010000}  // Continues data with scaling 500 deg/s
    };

static const ReadMap L3GD20H_READ_MAP = {
    {L3GD20H_OUT_X_L, L3GD20H_BUFFER_SIZE}
    };

class L3gd20h : public PololuAltImu
{
public:
    L3gd20h(I2cConn* i2c_conn);

    double get_angular_rate_x();
    double get_angular_rate_y();
    double get_angular_rate_z();
};

#endif
