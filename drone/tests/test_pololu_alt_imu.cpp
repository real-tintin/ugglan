#include <catch.h>
#include <cstring>

#include <i2c_conn_stub.h>
#include <pololu_alt_imu.h>

static const uint8_t TEST_CONFIG_REG_0 = 0x03;
static const uint8_t TEST_CONFIG_REG_1 = 0x30;
static const uint8_t TEST_CONFIG_DATA = 123;

static const uint8_t TEST_READ_REG_0 = 0x05;
static const uint8_t TEST_READ_REG_1 = 0x50;
static const uint8_t TEST_READ_REG_SIZE = 4;

static uint8_t TEST_READ_DATA_0[TEST_READ_REG_SIZE] = {0, 1, 2, 3};
static uint8_t TEST_READ_DATA_1[TEST_READ_REG_SIZE] = {4, 5, 6, 7};

static ConfigMap CONFIG_MAP = {
    {TEST_CONFIG_REG_0, TEST_CONFIG_DATA},
    {TEST_CONFIG_REG_1, TEST_CONFIG_DATA}
};
static ReadMap READ_MAP = {
    {TEST_READ_REG_0, TEST_READ_REG_SIZE},
    {TEST_READ_REG_1, TEST_READ_REG_SIZE}
};

static I2cReadBlockMap VALID_READ_MAP = {
    {TEST_READ_REG_0 | POLOLU_AUTO_INCREMENT, TEST_READ_DATA_0},
    {TEST_READ_REG_1 | POLOLU_AUTO_INCREMENT, TEST_READ_DATA_1}
};
static I2cWriteMap VALID_WRITE_MAP = {
    TEST_CONFIG_REG_0,
    TEST_CONFIG_REG_1
};

static I2cReadBlockMap INVALID_READ_MAP = {
    {TEST_READ_REG_0 + 1, TEST_READ_DATA_0},
    {TEST_READ_REG_1 | POLOLU_AUTO_INCREMENT, TEST_READ_DATA_1}
};
static I2cWriteMap INVALID_WRITE_MAP = {
    TEST_CONFIG_REG_0 + 1,
    TEST_CONFIG_REG_1,
};

class TestPololuAltImu : public PololuAltImu
{
public:
    TestPololuAltImu(I2cConn* i2c_conn) : PololuAltImu(i2c_conn)
    {
        _setup(CONFIG_MAP, READ_MAP);
    }

    uint8_t* get_reg_0_bytes() { return _get_buffer(TEST_READ_REG_0); }
    uint8_t* get_reg_1_bytes() { return _get_buffer(TEST_READ_REG_1); }
};

I2cConn get_i2c_conn_stub(I2cReadBlockMap read_map, I2cWriteMap write_map)
{
    I2cConn i2c_conn = I2cConn();

    i2c_conn.set_read_block_map(read_map);
    i2c_conn.set_write_map(write_map);

    return i2c_conn;
}

TEST_CASE("pololu_alt_imu: valid")
{
    I2cConn valid_i2c_conn = get_i2c_conn_stub(VALID_READ_MAP, VALID_WRITE_MAP);
    TestPololuAltImu alt_imu = TestPololuAltImu(&valid_i2c_conn);

    SECTION("config")
    {
        REQUIRE(alt_imu.get_status() == POLOLU_STATUS_OK);
    }
    SECTION("update and check buffer")
    {
        alt_imu.update();

        REQUIRE(alt_imu.get_status() == POLOLU_STATUS_OK);
        REQUIRE(std::memcmp(alt_imu.get_reg_0_bytes(), TEST_READ_DATA_0, TEST_READ_REG_SIZE) == 0);
        REQUIRE(std::memcmp(alt_imu.get_reg_1_bytes(), TEST_READ_DATA_1, TEST_READ_REG_SIZE) == 0);
    }
}

TEST_CASE("pololu_alt_imu: invalid")
{
    I2cConn invalid_i2c_conn = get_i2c_conn_stub(INVALID_READ_MAP, INVALID_WRITE_MAP);
    TestPololuAltImu alt_imu = TestPololuAltImu(&invalid_i2c_conn);

    SECTION("config")
    {
        REQUIRE(alt_imu.get_status() == POLOLU_STATUS_ERR_CONF);
    }
    SECTION("update and check buffer")
    {
        alt_imu.update();

        REQUIRE(alt_imu.get_status() == (POLOLU_STATUS_ERR_CONF | POLOLU_STATUS_ERR_READ));
        REQUIRE(std::memcmp(alt_imu.get_reg_0_bytes(), TEST_READ_DATA_0, TEST_READ_REG_SIZE) != 0);
        REQUIRE(std::memcmp(alt_imu.get_reg_1_bytes(), TEST_READ_DATA_1, TEST_READ_REG_SIZE) == 0);
    }
}

TEST_CASE("pololu_alt_imu: recover error read status")
{
    I2cConn i2c_conn = get_i2c_conn_stub(VALID_READ_MAP, VALID_WRITE_MAP);
    TestPololuAltImu alt_imu = TestPololuAltImu(&i2c_conn);

    alt_imu.update();
    REQUIRE(alt_imu.get_status() == POLOLU_STATUS_OK);

    i2c_conn.set_read_block_map(INVALID_READ_MAP);
    alt_imu.update();
    REQUIRE(alt_imu.get_status() == POLOLU_STATUS_ERR_READ);

    i2c_conn.set_read_block_map(VALID_READ_MAP);
    alt_imu.update();
    REQUIRE(alt_imu.get_status() == POLOLU_STATUS_OK);
}
