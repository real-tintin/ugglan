#include <catch.h>

#include <i2c_conn_stub.h>
#include <pololu_alt_imu.h>

static const uint8_t TEST_CONFIG_REG_0 = 0x03;
static const uint8_t TEST_CONFIG_REG_1 = 0x30;
static const uint8_t TEST_CONFIG_DATA_0 = 123;
static const uint8_t TEST_CONFIG_DATA_1 = 001;
static const uint8_t TEST_N_CONFIGS = 2;

static const uint8_t TEST_READ_REG_0 = 0x05;
static const uint8_t TEST_READ_REG_1 = 0x50;
static const uint8_t TEST_READ_REG_SIZE = 4;

static uint8_t TEST_READ_DATA_0[TEST_READ_REG_SIZE] = {0, 1, 2, 3};
static uint8_t TEST_READ_DATA_1[TEST_READ_REG_SIZE] = {4, 5, 6, 7};

static ConfigMap CONFIG_MAP = {
    {TEST_CONFIG_REG_0, TEST_CONFIG_DATA_0},
    {TEST_CONFIG_REG_1, TEST_CONFIG_DATA_1}
};
static ReadMap READ_MAP = {
    {TEST_READ_REG_0, TEST_READ_REG_SIZE},
    {TEST_READ_REG_1, TEST_READ_REG_SIZE}
};

static I2cReadBlockMap VALID_READ_BLOCK_MAP = {
    {TEST_READ_REG_0 | POLOLU_AUTO_INCREMENT, TEST_READ_DATA_0},
    {TEST_READ_REG_1 | POLOLU_AUTO_INCREMENT, TEST_READ_DATA_1}
};
static I2cWriteMap VALID_WRITE_MAP = {
    TEST_CONFIG_REG_0,
    TEST_CONFIG_REG_1
};

static I2cReadBlockMap INVALID_READ_BLOCK_MAP = {
    {TEST_READ_REG_0 + 1, TEST_READ_DATA_0},
    {TEST_READ_REG_1 | POLOLU_AUTO_INCREMENT, TEST_READ_DATA_1}
};
static I2cWriteMap INVALID_WRITE_MAP = {
    TEST_CONFIG_REG_0 + 1,
    TEST_CONFIG_REG_1,
};

static I2cReadByteMap MATCHING_READ_BYTE_MAP = {
    {TEST_CONFIG_REG_0, TEST_CONFIG_DATA_0},
    {TEST_CONFIG_REG_1, TEST_CONFIG_DATA_1}
};
static I2cReadByteMap MISMATCHING_READ_BYTE_MAP = {
    {TEST_CONFIG_REG_0, TEST_CONFIG_DATA_0 + 1},
    {TEST_CONFIG_REG_1, TEST_CONFIG_DATA_1 + 1}
};

class TestPololuAltImu : public PololuAltImu
{
public:
    TestPololuAltImu(I2cConn& i2c_conn) : PololuAltImu(i2c_conn, "TestPololuAltImu")
    {
        _setup(CONFIG_MAP, READ_MAP);
    }

    uint8_t* get_reg_0_bytes() { return _get_buffer(TEST_READ_REG_0); }
    uint8_t* get_reg_1_bytes() { return _get_buffer(TEST_READ_REG_1); }
};

I2cConnStub get_i2c_conn_stub(I2cReadByteMap read_byte_map,
                              I2cReadBlockMap read_block_map,
                              I2cWriteMap write_map)
{
    I2cConnStub i2c_conn;

    i2c_conn.set_read_byte_map(read_byte_map);
    i2c_conn.set_read_block_map(read_block_map);
    i2c_conn.set_write_map(write_map);

    return i2c_conn;
}

TEST_CASE("valid i2c registry")
{
    I2cConnStub valid_i2c_conn = get_i2c_conn_stub(MATCHING_READ_BYTE_MAP, VALID_READ_BLOCK_MAP, VALID_WRITE_MAP);
    TestPololuAltImu alt_imu(valid_i2c_conn);

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

TEST_CASE("invalid i2c registry")
{
    I2cConnStub invalid_i2c_conn = get_i2c_conn_stub(MISMATCHING_READ_BYTE_MAP, INVALID_READ_BLOCK_MAP, INVALID_WRITE_MAP);
    TestPololuAltImu alt_imu(invalid_i2c_conn);

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

TEST_CASE("recover error read status")
{
    I2cConnStub i2c_conn = get_i2c_conn_stub(MATCHING_READ_BYTE_MAP, VALID_READ_BLOCK_MAP, VALID_WRITE_MAP);
    TestPololuAltImu alt_imu(i2c_conn);

    alt_imu.update();
    REQUIRE(alt_imu.get_status() == POLOLU_STATUS_OK);

    i2c_conn.set_read_block_map(INVALID_READ_BLOCK_MAP);
    alt_imu.update();
    REQUIRE(alt_imu.get_status() == POLOLU_STATUS_ERR_READ);

    i2c_conn.set_read_block_map(VALID_READ_BLOCK_MAP);
    alt_imu.update();
    REQUIRE(alt_imu.get_status() == POLOLU_STATUS_OK);
}

TEST_CASE("update of config")
{
    SECTION("matching")
    {
        I2cConnStub i2c_conn = get_i2c_conn_stub(MATCHING_READ_BYTE_MAP, VALID_READ_BLOCK_MAP, VALID_WRITE_MAP);

        uint32_t n_calls_read_byte_data = i2c_conn.get_n_calls_read_byte_data();
        uint32_t n_calls_write_byte_data = i2c_conn.get_n_calls_write_byte_data();

        TestPololuAltImu alt_imu(i2c_conn);

        REQUIRE((n_calls_read_byte_data + TEST_N_CONFIGS) == i2c_conn.get_n_calls_read_byte_data());
        REQUIRE(n_calls_write_byte_data == i2c_conn.get_n_calls_write_byte_data());
    }
    SECTION("mismatching")
    {
        I2cConnStub i2c_conn = get_i2c_conn_stub(MISMATCHING_READ_BYTE_MAP, VALID_READ_BLOCK_MAP, VALID_WRITE_MAP);

        uint32_t n_calls_read_byte_data = i2c_conn.get_n_calls_read_byte_data();
        uint32_t n_calls_write_byte_data = i2c_conn.get_n_calls_write_byte_data();

        TestPololuAltImu alt_imu(i2c_conn);

        REQUIRE((n_calls_read_byte_data + TEST_N_CONFIGS) == i2c_conn.get_n_calls_read_byte_data());
        REQUIRE((n_calls_write_byte_data + TEST_N_CONFIGS) == i2c_conn.get_n_calls_write_byte_data());
    }
}
