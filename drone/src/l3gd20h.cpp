#include <l3gd20h.h>

L3gd20h::L3gd20h(I2cConn* i2c_conn) : PololuAltImu(i2c_conn)
{
    _setup(L3GD20H_CONFIG_MAP, L3GD20H_READ_MAP);
}

// Returns angular rate about x [deg/s]
double L3gd20h::get_angular_rate_x()
{
    uint8_t* buffer = _get_buffer(L3GD20H_OUT_X_L);

    int16_t raw_ang_rate_x = (buffer[L3GD20H_BUFFER_OUT_X_H] << 8) | buffer[L3GD20H_BUFFER_OUT_X_L];
    double ang_rate_x = double(raw_ang_rate_x) * double(L3GD20H_GYRO_SCALE) / double(L3GD20H_GYRO_RESOLUTION);

    return ang_rate_x;
}

// Returns angular rate about y [deg/s]
double L3gd20h::get_angular_rate_y()
{
    uint8_t* buffer = _get_buffer(L3GD20H_OUT_X_L);

    int16_t raw_ang_rate_y = (buffer[L3GD20H_BUFFER_OUT_Y_H] << 8) | buffer[L3GD20H_BUFFER_OUT_Y_L];
    double ang_rate_y = double(raw_ang_rate_y) * double(L3GD20H_GYRO_SCALE) / double(L3GD20H_GYRO_RESOLUTION);

    return ang_rate_y;
}

// Returns angular rate about z [deg/s]
double L3gd20h::get_angular_rate_z()
{
    uint8_t* buffer = _get_buffer(L3GD20H_OUT_X_L);

    int16_t raw_ang_rate_z = (buffer[L3GD20H_BUFFER_OUT_Z_H] << 8) | buffer[L3GD20H_BUFFER_OUT_Z_L];
    double ang_rate_z = double(raw_ang_rate_z) * double(L3GD20H_GYRO_SCALE) / double(L3GD20H_GYRO_RESOLUTION);

    return ang_rate_z;
}
