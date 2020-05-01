#include <catch.h>
#include <cstring>

#include <pololu_alt_imu.h>

static const uint8_t TEST_CONFIG_REG = 0x03;
static const uint8_t TEST_CONFIG_DATA = 123;

static const uint8_t TEST_READ_REG = 0x05;
static const uint8_t TEST_READ_REG_SIZE = 4;

static uint8_t TEST_READ_DATA[TEST_READ_REG_SIZE] = {0, 1, 2, 3};

static ConfigMap CONFIG_MAP = {{TEST_CONFIG_REG, TEST_CONFIG_DATA}};
static ReadMap READ_MAP = {{TEST_READ_REG, TEST_READ_REG_SIZE}};

static I2cReadBlockMap VALID_READ_MAP = {{TEST_READ_REG, TEST_READ_DATA}};
static I2cWriteMap VALID_WRITE_MAP = {TEST_CONFIG_REG};

static I2cReadBlockMap INVALID_READ_MAP = {{TEST_READ_REG + 1, TEST_READ_DATA}};
static I2cWriteMap INVALID_WRITE_MAP = {TEST_CONFIG_REG - 1};

I2cConn get_i2c_conn(I2cReadBlockMap read_map, I2cWriteMap write_map)
{
    I2cConn i2c_conn = I2cConn();

    i2c_conn.set_read_block_map(read_map);
    i2c_conn.set_write_map(write_map);

    return i2c_conn;
}

TEST_CASE("pololu_alt_imu: valid")
{
    I2cConn valid_i2c_conn = get_i2c_conn(VALID_READ_MAP, VALID_WRITE_MAP);
    PololuAltImu alt_imu = PololuAltImu(&valid_i2c_conn, CONFIG_MAP, READ_MAP);

    SECTION("config")
    {
        REQUIRE(alt_imu.get_status() == POLOLU_STATUS_OK);
    }
    SECTION("update and check buffer")
    {
        alt_imu.update();

        REQUIRE(alt_imu.get_status() == POLOLU_STATUS_OK);
        REQUIRE(std::memcmp(alt_imu.get_buffer(TEST_READ_REG), TEST_READ_DATA, TEST_READ_REG_SIZE) == 0);
    }
}

TEST_CASE("pololu_alt_imu: invalid")
{
    I2cConn invalid_i2c_conn = get_i2c_conn(INVALID_READ_MAP, INVALID_WRITE_MAP);
    PololuAltImu alt_imu = PololuAltImu(&invalid_i2c_conn, CONFIG_MAP, READ_MAP);

    SECTION("config")
    {
        REQUIRE(alt_imu.get_status() == POLOLU_STATUS_ERR_CONF);
    }
    SECTION("update and check buffer")
    {
        alt_imu.update();

        REQUIRE(alt_imu.get_status() == (POLOLU_STATUS_ERR_CONF | POLOLU_STATUS_ERR_READ));
        REQUIRE(std::memcmp(alt_imu.get_buffer(TEST_READ_REG), TEST_READ_DATA, TEST_READ_REG_SIZE) != 0);
    }
}

TEST_CASE("pololu_alt_imu: recover error read status")
{
    I2cConn i2c_conn = get_i2c_conn(VALID_READ_MAP, VALID_WRITE_MAP);
    PololuAltImu alt_imu = PololuAltImu(&i2c_conn, CONFIG_MAP, READ_MAP);

    alt_imu.update();
    REQUIRE(alt_imu.get_status() == POLOLU_STATUS_OK);

    i2c_conn.set_read_block_map(INVALID_READ_MAP);
    alt_imu.update();
    REQUIRE(alt_imu.get_status() == POLOLU_STATUS_ERR_READ);

    i2c_conn.set_read_block_map(VALID_READ_MAP);
    alt_imu.update();
    REQUIRE(alt_imu.get_status() == POLOLU_STATUS_OK);
}
