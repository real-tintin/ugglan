#include <catch.h>

#include <i2c_conn_stub.h>
#include <pololu_alt_imu.h>

#include <lps25h.h>

static const double FLOAT_TOL = 1e-4;

static I2cWriteMap WRITE_MAP = {LPS25H_CTRL_REG1, LPS25H_RES_CONF};

void update_i2c_stub_data(I2cConn* i2c_conn, uint8_t pres[LPS25H_BUF_PRES_SIZE],
                          uint8_t temp[LPS25H_BUF_TEMP_SIZE])
{
    I2cReadBlockMap read_map = {
        {LPS25H_PRESS_OUT_XL | POLOLU_AUTO_INCREMENT, pres},
        {LPS25H_TEMP_OUT_P_L | POLOLU_AUTO_INCREMENT, temp}
    };

    i2c_conn->set_read_block_map(read_map);
}

TEST_CASE("lps25h interpretation")
{
    I2cConn i2c_conn = I2cConn();
    i2c_conn.set_write_map(WRITE_MAP);

    Lps25h barometer = Lps25h(&i2c_conn);

    SECTION("zero")
    {
        uint8_t pres_data[LPS25H_BUF_PRES_SIZE] = {0};
        uint8_t temp_data[LPS25H_BUF_TEMP_SIZE] = {
            80, 176}; // Compensate for offset on temperature -42.5 * 480 = -20400

        update_i2c_stub_data(&i2c_conn, pres_data, temp_data);
        barometer.update();

        REQUIRE(barometer.get_pressure() == 0.0);
        REQUIRE(barometer.get_temperature() == 0.0);
    }
    SECTION("non-zero")
    {
        uint8_t pres_data[LPS25H_BUF_PRES_SIZE] = {
            0, 84, 63}; // 101325 / 100 * 4096 = 4150272
        uint8_t temp_data[LPS25H_BUF_TEMP_SIZE] = {
            176, 245}; // (37 - 42.5) * 480 = -2640

        update_i2c_stub_data(&i2c_conn, pres_data, temp_data);
        barometer.update();

        REQUIRE(fabs(barometer.get_pressure() - 101325.0) <= FLOAT_TOL);
        REQUIRE(fabs(barometer.get_temperature() - 37.0) <= FLOAT_TOL);
    }
}
