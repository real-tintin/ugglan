#include <catch.h>

#include <i2c_conn_stub.h>
#include <afro_esc.h>

/*
Expected read buffer:
    Byte 0-1 : Rev counter set to 1000 [-]
    Byte 2-3 : Voltage set to 1000 [V]
    Byte 4-5 : Temperature set to 1000 [C]
    Byte 6-7 : Current set to 1000 [A]
    Byte 8   : Alive byte set to true
*/
uint8_t READ_BUF[AFRO_READ_BUF_SIZE] =
{

    0b00000011,
    0b11101000,

    0b00, // TODO
    0b00, // TODO

    0b00, // TODO
    0b00, // TODO

    0b00, // TODO
    0b00, // TODO

    AFRO_IF_ALIVE_BYTE
};

static I2cWriteMap WRITE_MAP = {AFRO_WRITE_THROTTLE_H};
static I2cReadBlockMap READ_MAP = { {AFRO_READ_REV_H, READ_BUF} };

TEST_CASE("afro esc")
{
    I2cConn i2c_conn = I2cConn();
    i2c_conn.set_write_map(WRITE_MAP);
    i2c_conn.set_read_block_map(READ_MAP);

    AfroEsc esc = AfroEsc(&i2c_conn);

    SECTION("initialized")
    {
        REQUIRE(esc.get_status() == AFRO_STATUS_OK);
    }

    SECTION("read")
    {
        esc.read();
        // TODO: Verify get functions.
    }

    SECTION("write")
    {
        esc.write(1U);
        REQUIRE(esc.get_status() == AFRO_STATUS_OK);
    }
}
