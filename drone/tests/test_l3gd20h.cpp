#include <catch.h>

#include <i2c_conn_stub.h>
#include <pololu_alt_imu.h>
#include <l3gd20h.h>

static const double FLOAT_TOL = 1e-4;

static I2cWriteMap WRITE_MAP = {L3GD20H_CTRL1, L3GD20H_CTRL4};

void update_i2c_stub_data(I2cConn* i2c_conn, uint8_t data[L3GD20H_BUFFER_SIZE])
{
    I2cReadBlockMap read_map = {{L3GD20H_OUT_X_L | POLOLU_AUTO_INCREMENT, data}};
    i2c_conn->set_read_block_map(read_map);
}

TEST_CASE("l3gd20h interpretation")
{
    I2cConn i2c_conn = I2cConn();
    i2c_conn.set_write_map(WRITE_MAP);

    L3gd20h gyro = L3gd20h(&i2c_conn);

    SECTION("zero")
    {
        uint8_t i2c_stub_data[L3GD20H_BUFFER_SIZE] = {0};
        update_i2c_stub_data(&i2c_conn, i2c_stub_data);

        gyro.update();

        REQUIRE(gyro.get_angular_rate_x() == 0.0);
        REQUIRE(gyro.get_angular_rate_y() == 0.0);
        REQUIRE(gyro.get_angular_rate_z() == 0.0);
    }
    SECTION("non-zero")
    {
        uint8_t i2c_stub_data[L3GD20H_BUFFER_SIZE] = {
            66, 0,  // 66 * 8.7267 / 32767 approx 1.757e-2
            2, 128, // -32766 * 8.7267 / 32767 approx -8.7264
            13, 0}; // 13 * 8.7267 / 32767 approx 3.462e-3
        update_i2c_stub_data(&i2c_conn, i2c_stub_data);

        gyro.update();

        REQUIRE(fabs(gyro.get_angular_rate_x() - 1.757e-2) <= FLOAT_TOL);
        REQUIRE(fabs(gyro.get_angular_rate_y() + 8.7264) <= FLOAT_TOL);
        REQUIRE(fabs(gyro.get_angular_rate_z() - 3.462e-3) <= FLOAT_TOL);
    }
}
