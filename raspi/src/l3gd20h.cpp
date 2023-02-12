#include <l3gd20h.h>

L3gd20h::L3gd20h(I2cConn& i2c_conn) : PololuAltImu(i2c_conn, "L3gd20h")
{
    _setup(L3GD20H_CONFIG_MAP, L3GD20H_READ_MAP);
}

// Returns angular rate about x [rad/s]
double L3gd20h::get_angular_rate_x()
{
    uint8_t* buf = _get_buffer(L3GD20H_REG_OUT_X_L);

    int16_t raw_ang_rate_x = (buf[L3GD20H_BUF_OUT_X_H] << 8) | buf[L3GD20H_BUF_OUT_X_L];
    double ang_rate_x = static_cast<double>(raw_ang_rate_x) * L3GD20H_GYRO_SCALE / L3GD20H_GYRO_RESOLUTION;

    return ang_rate_x;
}

// Returns angular rate about y [rad/s]
double L3gd20h::get_angular_rate_y()
{
    uint8_t* buf = _get_buffer(L3GD20H_REG_OUT_X_L);

    int16_t raw_ang_rate_y = (buf[L3GD20H_BUF_OUT_Y_H] << 8) | buf[L3GD20H_BUF_OUT_Y_L];
    double ang_rate_y = static_cast<double>(raw_ang_rate_y) * L3GD20H_GYRO_SCALE / L3GD20H_GYRO_RESOLUTION;

    return ang_rate_y;
}

// Returns angular rate about z [rad/s]
double L3gd20h::get_angular_rate_z()
{
    uint8_t* buf = _get_buffer(L3GD20H_REG_OUT_X_L);

    int16_t raw_ang_rate_z = (buf[L3GD20H_BUF_OUT_Z_H] << 8) | buf[L3GD20H_BUF_OUT_Z_L];
    double ang_rate_z = static_cast<double>(raw_ang_rate_z) * L3GD20H_GYRO_SCALE / L3GD20H_GYRO_RESOLUTION;

    return ang_rate_z;
}
