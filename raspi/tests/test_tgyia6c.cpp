#include <catch.hpp>

#include <serial_conn_stub.hpp>
#include <tgyia6c.hpp>

#define TWOS_COMP_1000_LSB 0b11101000
#define TWOS_COMP_1000_MSB 0b00000011

#define TWOS_COMP_1500_LSB 0b11011100
#define TWOS_COMP_1500_MSB 0b00000101

#define TWOS_COMP_2000_LSB 0b11010000
#define TWOS_COMP_2000_MSB 0b00000111

static const double FLOAT_TOL = 1e-4;

static const uint8_t INVALID_BUF_SIZE = TGYIA6C_BUF_SIZE;
uint8_t INVALID_BUF[INVALID_BUF_SIZE] = {0};

/*
Shall contain the following
    Byte 0      : Package size in bytes
    Byte 1      : TGYIA6C_PROTOCOL_COMMAND40
    Byte 2-3    : Channel 0 set to 1000
    Byte 4-5    : Channel 1 set to 1500
    Byte 6-7    : Channel 2 set to 2000
    Byte 8-9    : Channel 3 set to 1000
    Byte 10-11  : Channel 4 set to 1500
    Byte 12-13  : Channel 5 set to 2000
    Byte 14-15  : Channel 6 set to 1000
    Byte 16-17  : Channel 7 set to 1500
    Byte 18-19  : Channel 8 set to 0 (not used)
    Byte 20-21  : Channel 9 set to 0 (not used)
    Byte 22     : LSB checksum
    Byte 23     : MSB checksum

Note that as two's complement
    1000: (0b00000011 << 8 | 0b11101000‬)
    1500: (0b00000101 << 8 | 0b11011100)
    2000: (0b00000111 << 8 | 0b11010000)

Where checksum = 0xFFFF - bytes =
    0xFFFF - 0x18 - 0x40 - (0x3 +  0xE8) * 3 -
        (0x5 + 0xDC) * 3 - (0x7 + 0xD0) * 2 =
    0xFFFF - 0x2C1 - 0x2A3 - 0x1AE = F895
i.e.,
    LSB checksum: 0b11111000
    MSB checksum: 0b10010101
*/
static const uint8_t VALID_BUF_SIZE = 24;
uint8_t VALID_BUF[VALID_BUF_SIZE] = {VALID_BUF_SIZE,
                                     TGYIA6C_PROTOCOL_COMMAND40,
                                     TWOS_COMP_1000_LSB,
                                     TWOS_COMP_1000_MSB,
                                     TWOS_COMP_1500_LSB,
                                     TWOS_COMP_1500_MSB,
                                     TWOS_COMP_2000_LSB,
                                     TWOS_COMP_2000_MSB,
                                     TWOS_COMP_1000_LSB,
                                     TWOS_COMP_1000_MSB,
                                     TWOS_COMP_1500_LSB,
                                     TWOS_COMP_1500_MSB,
                                     TWOS_COMP_2000_LSB,
                                     TWOS_COMP_2000_MSB,
                                     TWOS_COMP_1000_LSB,
                                     TWOS_COMP_1000_MSB,
                                     TWOS_COMP_1500_LSB,
                                     TWOS_COMP_1500_MSB,
                                     0U,
                                     0U,
                                     0U,
                                     0U,
                                     0b10010101,
                                     0b11111000};

void assert_init_values(Tgyia6c &rc_receiver)
{
    REQUIRE(fabs(rc_receiver.get_gimbal_left_x() - 0.5) <= FLOAT_TOL);
    REQUIRE(fabs(rc_receiver.get_gimbal_left_y() - 0.0) <= FLOAT_TOL);

    REQUIRE(fabs(rc_receiver.get_gimbal_right_x() - 0.5) <= FLOAT_TOL);
    REQUIRE(fabs(rc_receiver.get_gimbal_right_y() - 0.5) <= FLOAT_TOL);

    REQUIRE(fabs(rc_receiver.get_knob() - 0.0) <= FLOAT_TOL);

    REQUIRE(rc_receiver.get_switch_left() == SwitchLr::High);
    REQUIRE(rc_receiver.get_switch_right() == SwitchLr::High);
    REQUIRE(rc_receiver.get_switch_middle() == SwitchM::High);
}

void assert_valid_values(Tgyia6c &rc_receiver)
{
    REQUIRE(fabs(rc_receiver.get_gimbal_left_x() - 0.0) <= FLOAT_TOL);
    REQUIRE(fabs(rc_receiver.get_gimbal_left_y() - 1.0) <= FLOAT_TOL);

    REQUIRE(fabs(rc_receiver.get_gimbal_right_x() - 0.0) <= FLOAT_TOL);
    REQUIRE(fabs(rc_receiver.get_gimbal_right_y() - 0.5) <= FLOAT_TOL);

    REQUIRE(fabs(rc_receiver.get_knob() - 0.0) <= FLOAT_TOL);

    REQUIRE(rc_receiver.get_switch_left() == SwitchLr::Middle);
    REQUIRE(rc_receiver.get_switch_right() == SwitchLr::Middle);
    REQUIRE(rc_receiver.get_switch_middle() == SwitchM::Low);
}

TEST_CASE("tgyia6c initialized")
{
    SerialConnStub serial_conn;
    Tgyia6c rc_receiver(serial_conn);

    SECTION("status")
    {
        REQUIRE(rc_receiver.get_status() == TGYIA6C_STATUS_OK);
    }
    SECTION("init values")
    {
        assert_init_values(rc_receiver);
    }
}

TEST_CASE("tgyia6c parse data")
{
    SerialConnStub serial_conn;
    Tgyia6c rc_receiver(serial_conn);

    SECTION("valid")
    {
        serial_conn.set_read_buf(VALID_BUF, VALID_BUF_SIZE);
        rc_receiver.update();

        assert_valid_values(rc_receiver);
    }
    SECTION("invalid")
    {
        serial_conn.set_read_buf(INVALID_BUF, INVALID_BUF_SIZE);
        rc_receiver.update();

        assert_init_values(rc_receiver);
    }
}
