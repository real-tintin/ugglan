#include <lsm303d.h>

Lsm303d::Lsm303d(I2cConn& i2c_conn) : PololuAltImu(i2c_conn)
{
    _setup(LSM303D_CONFIG_MAP, LSM303D_READ_MAP);
}

// Returns acceleration component in x [m/s]
double Lsm303d::get_acceleration_x()
{
    uint8_t* buf = _get_buffer(LSM303D_OUT_X_L_A);

    int16_t raw_acc_x = (buf[LSM303D_BUF_OUT_X_H_A] << 8) | buf[LSM303D_BUF_OUT_X_L_A];
    double acc_x = double(raw_acc_x) * LSM303D_ACC_SCALE / LSM303D_ACC_RESOLUTION;

    return acc_x;
}

// Returns acceleration component in y [m/s]
double Lsm303d::get_acceleration_y()
{
    uint8_t* buf = _get_buffer(LSM303D_OUT_X_L_A);

    int16_t raw_acc_y = (buf[LSM303D_BUF_OUT_Y_H_A] << 8) | buf[LSM303D_BUF_OUT_Y_L_A];
    double acc_y = double(raw_acc_y) * LSM303D_ACC_SCALE / LSM303D_ACC_RESOLUTION;

    return acc_y;
}

// Returns acceleration component in z [m/s]
double Lsm303d::get_acceleration_z()
{
    uint8_t* buf = _get_buffer(LSM303D_OUT_X_L_A);

    int16_t raw_acc_z = (buf[LSM303D_BUF_OUT_Z_H_A] << 8) | buf[LSM303D_BUF_OUT_Z_L_A];
    double acc_z = double(raw_acc_z) * LSM303D_ACC_SCALE / LSM303D_ACC_RESOLUTION;

    return acc_z;
}

// Returns magnetic field component in x [gauss]
double Lsm303d::get_magnetic_field_x()
{
    uint8_t* buf = _get_buffer(LSM303D_OUT_X_L_M);

    int16_t raw_mag_x = (buf[LSM303D_BUF_OUT_X_H_M] << 8) | buf[LSM303D_BUF_OUT_X_L_M];
    double mag_x = double(raw_mag_x) * LSM303D_MAG_SCALE / LSM303D_MAG_RESOLUTION;

    return mag_x;
}

// Returns magnetic field component in y [gauss]
double Lsm303d::get_magnetic_field_y()
{
    uint8_t* buf = _get_buffer(LSM303D_OUT_X_L_M);

    int16_t raw_mag_y = (buf[LSM303D_BUF_OUT_Y_H_M] << 8) | buf[LSM303D_BUF_OUT_Y_L_M];
    double mag_y = double(raw_mag_y) * LSM303D_MAG_SCALE / LSM303D_MAG_RESOLUTION;

    return mag_y;
}

// Returns magnetic field component in z [gauss]
double Lsm303d::get_magnetic_field_z()
{
    uint8_t* buf = _get_buffer(LSM303D_OUT_X_L_M);

    int16_t raw_mag_z = (buf[LSM303D_BUF_OUT_Z_H_M] << 8) | buf[LSM303D_BUF_OUT_Z_L_M];
    double mag_z = double(raw_mag_z) * LSM303D_MAG_SCALE / LSM303D_MAG_RESOLUTION;

    return mag_z;
}
