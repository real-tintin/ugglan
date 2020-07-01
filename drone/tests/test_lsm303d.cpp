#include <catch.h>

#include <i2c_conn_stub.h>
#include <pololu_alt_imu.h>
#include <lsm303d.h>

static const double FLOAT_TOL = 1e-4;

static I2cWriteMap WRITE_MAP = {LSM303D_CTRL1, LSM303D_CTRL2,
    LSM303D_CTRL5, LSM303D_CTRL6, LSM303D_CTRL7};

void set_lsm303d_stub_data(I2cConn* i2c_conn, uint8_t acc[LSM303D_BUF_ACC_SIZE],
                           uint8_t mag[LSM303D_BUF_MAG_SIZE])
{
    I2cReadBlockMap read_map = {
        {LSM303D_OUT_X_L_A | POLOLU_AUTO_INCREMENT, acc},
        {LSM303D_OUT_X_L_M | POLOLU_AUTO_INCREMENT, mag}
    };

    i2c_conn->set_read_block_map(read_map);
}

TEST_CASE("lsm303d interpretation")
{
    I2cConn i2c_conn = I2cConn();
    i2c_conn.set_write_map(WRITE_MAP);

    Lsm303d acc_mag = Lsm303d(&i2c_conn);

    SECTION("zero")
    {
        uint8_t acc_data[LSM303D_BUF_ACC_SIZE] = {0};
        uint8_t mag_data[LSM303D_BUF_MAG_SIZE] = {0};

        set_lsm303d_stub_data(&i2c_conn, acc_data, mag_data);
        acc_mag.update();

        REQUIRE(acc_mag.get_acceleration_x() == 0.0);
        REQUIRE(acc_mag.get_acceleration_y() == 0.0);
        REQUIRE(acc_mag.get_acceleration_z() == 0.0);

        REQUIRE(acc_mag.get_magnetic_field_x() == 0.0);
        REQUIRE(acc_mag.get_magnetic_field_y() == 0.0);
        REQUIRE(acc_mag.get_magnetic_field_z() == 0.0);
    }
    SECTION("non-zero")
    {
        uint8_t acc_data[LSM303D_BUF_ACC_SIZE] = {
            123, 0,   // 123 * 4 * 9.82 / (2^15-1) = 0.14745
            133, 255, // -123 * 4 * 9.82 / (2^15-1) = -0.14745
            0, 32};   // 8192 * 4 * 9.82 / (2^15-1) = 9.8203
        uint8_t mag_data[LSM303D_BUF_MAG_SIZE] = {
            225, 16, // 4321 * 4 / (2^15-1) = 0.52748
            31, 239, // -4321 * 4 / (2^15-1) = -0.52748
            0, 80};  // 20480 * 4 / (2^15-1) = 2.5001

        set_lsm303d_stub_data(&i2c_conn, acc_data, mag_data);
        acc_mag.update();

        REQUIRE(fabs(acc_mag.get_acceleration_x() - 0.14745) <= FLOAT_TOL);
        REQUIRE(fabs(acc_mag.get_acceleration_y() + 0.14745) <= FLOAT_TOL);
        REQUIRE(fabs(acc_mag.get_acceleration_z() - 9.8203) <= FLOAT_TOL);

        REQUIRE(fabs(acc_mag.get_magnetic_field_x() - 0.52748) <= FLOAT_TOL);
        REQUIRE(fabs(acc_mag.get_magnetic_field_y() + 0.52748) <= FLOAT_TOL);
        REQUIRE(fabs(acc_mag.get_magnetic_field_z() - 2.5001) <= FLOAT_TOL);
    }
}
