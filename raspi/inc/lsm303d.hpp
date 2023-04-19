/*
LSM303D accelerometer and magnetometer, see data sheet
https://www.pololu.com/file/download/LSM303D.pdf?file_id=0J703
*/

#ifndef LSM303D_HPP
#define LSM303D_HPP

#include <cstdint>

#include <pololu_alt_imu.hpp>
#include <science_const.hpp>

inline const uint8_t LSM303D_I2C_ADDRESS = 0x1D;

inline const uint8_t LSM303D_REG_OUT_X_L_M = 0x08;
inline const uint8_t LSM303D_REG_OUT_X_H_M = 0x09;
inline const uint8_t LSM303D_REG_OUT_Y_L_M = 0x0A;
inline const uint8_t LSM303D_REG_OUT_Y_H_M = 0x0B;
inline const uint8_t LSM303D_REG_OUT_Z_L_M = 0x0C;
inline const uint8_t LSM303D_REG_OUT_Z_H_M = 0x0D;

inline const uint8_t LSM303D_REG_INT_CTRL_M = 0x12;

inline const uint8_t LSM303D_REG_OFFSET_X_L_M = 0x16;
inline const uint8_t LSM303D_REG_OFFSET_X_H_M = 0x17;
inline const uint8_t LSM303D_REG_OFFSET_Y_L_M = 0x18;
inline const uint8_t LSM303D_REG_OFFSET_Y_H_M = 0x19;
inline const uint8_t LSM303D_REG_OFFSET_Z_L_M = 0x1A;
inline const uint8_t LSM303D_REG_OFFSET_Z_H_M = 0x1B;

inline const uint8_t LSM303D_REG_REFERENCE_X = 0x1C;
inline const uint8_t LSM303D_REG_REFERENCE_Y = 0x1D;
inline const uint8_t LSM303D_REG_REFERENCE_Z = 0x1E;

inline const uint8_t LSM303D_REG_CTRL0 = 0x1F;
inline const uint8_t LSM303D_REG_CTRL1 = 0x20;
inline const uint8_t LSM303D_REG_CTRL2 = 0x21;
inline const uint8_t LSM303D_REG_CTRL5 = 0x24;
inline const uint8_t LSM303D_REG_CTRL6 = 0x25;
inline const uint8_t LSM303D_REG_CTRL7 = 0x26;

inline const uint8_t LSM303D_REG_OUT_X_L_A = 0x28;
inline const uint8_t LSM303D_REG_OUT_X_H_A = 0x29;
inline const uint8_t LSM303D_REG_OUT_Y_L_A = 0x2A;
inline const uint8_t LSM303D_REG_OUT_Y_H_A = 0x2B;
inline const uint8_t LSM303D_REG_OUT_Z_L_A = 0x2C;
inline const uint8_t LSM303D_REG_OUT_Z_H_A = 0x2D;

inline const uint8_t LSM303D_REG_FIFO_CTRL = 0x2E;

inline const uint8_t LSM303D_REG_IG_CFG1 = 0x30;
inline const uint8_t LSM303D_REG_IG_CFG2 = 0x34;

inline const uint8_t LSM303D_REG_CLICK_CFG = 0x38;

inline const uint8_t LSM303D_BUF_ACC_SIZE = 6;
inline const uint8_t LSM303D_BUF_OUT_X_L_A = 0x00;
inline const uint8_t LSM303D_BUF_OUT_X_H_A = 0x01;
inline const uint8_t LSM303D_BUF_OUT_Y_L_A = 0x02;
inline const uint8_t LSM303D_BUF_OUT_Y_H_A = 0x03;
inline const uint8_t LSM303D_BUF_OUT_Z_L_A = 0x04;
inline const uint8_t LSM303D_BUF_OUT_Z_H_A = 0x05;

inline const uint8_t LSM303D_BUF_MAG_SIZE = 6;
inline const uint8_t LSM303D_BUF_OUT_X_L_M = 0x00;
inline const uint8_t LSM303D_BUF_OUT_X_H_M = 0x01;
inline const uint8_t LSM303D_BUF_OUT_Y_L_M = 0x02;
inline const uint8_t LSM303D_BUF_OUT_Y_H_M = 0x03;
inline const uint8_t LSM303D_BUF_OUT_Z_L_M = 0x04;
inline const uint8_t LSM303D_BUF_OUT_Z_H_M = 0x05;

inline const double LSM303D_ACC_SCALE = 4 * GRAVITY_EARTH; // [m/s]
inline const double LSM303D_ACC_RESOLUTION = 32767;        // 2^15-1 (16-bit signed integer)

inline const double LSM303D_MAG_SCALE = 4;          // [gauss]
inline const double LSM303D_MAG_RESOLUTION = 32767; // 2^15-1 (16-bit signed integer)

inline const ConfigMap LSM303D_CONFIG_MAP = {
    {LSM303D_REG_INT_CTRL_M, 0b00000000}, // Default (see detailed list not the overview table)

    {LSM303D_REG_OFFSET_X_L_M, 0b00000000}, // Default
    {LSM303D_REG_OFFSET_X_H_M, 0b00000000}, // Default
    {LSM303D_REG_OFFSET_Y_L_M, 0b00000000}, // Default
    {LSM303D_REG_OFFSET_Y_H_M, 0b00000000}, // Default
    {LSM303D_REG_OFFSET_Z_L_M, 0b00000000}, // Default
    {LSM303D_REG_OFFSET_Z_H_M, 0b00000000}, // Default

    {LSM303D_REG_CTRL0, 0b00000000}, // Default
    {LSM303D_REG_CTRL1, 0b01100111}, // Enable XYZ accelerometer axis for continues output at 100 Hz
    {LSM303D_REG_CTRL2, 0b00001000}, // Accelerometer scale selection, ± 4 g
    {LSM303D_REG_CTRL5, 0b01110100}, // Enable magnetometer at 100 Hz in high resolution mode
    {LSM303D_REG_CTRL6, 0b00100000}, // Magnetometer scale selection, ± 4 gauss
    {LSM303D_REG_CTRL7, 0b00000000}, // Magnetometer normal, continuous-conversion mode

    {LSM303D_REG_FIFO_CTRL, 0b00000000}, // Default

    {LSM303D_REG_IG_CFG1, 0b00000000}, // Default
    {LSM303D_REG_IG_CFG2, 0b00000000}, // Default

    {LSM303D_REG_CLICK_CFG, 0b00000000} // Default
};

inline const ReadMap LSM303D_READ_MAP = {
    {LSM303D_REG_OUT_X_L_A, LSM303D_BUF_ACC_SIZE},
    {LSM303D_REG_OUT_X_L_M, LSM303D_BUF_MAG_SIZE},
};

class Lsm303d : public PololuAltImu {
  public:
    Lsm303d(I2cConn &i2c_conn);

    double get_acceleration_x();
    double get_acceleration_y();
    double get_acceleration_z();

    double get_magnetic_field_x();
    double get_magnetic_field_y();
    double get_magnetic_field_z();
};

#endif /* LSM303D_HPP */
