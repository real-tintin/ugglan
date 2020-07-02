/*
TGYIA6C radio receiver parsing FlySky I-BUS, see data sheet
https://hobbyking.com/en_us/turnigy-ia6c-ppm-sbus-receiver.html.

Heavily inspired by https://github.com/aanon4/FlySkyIBus.
*/

#ifndef TGYIA6C_H
#define TGYIA6C_H

#include <cstdint>
#include <map>
#include <science_const.h>

#if defined(UNIT_TEST)
#include <serial_conn_stub.h>
#else
#include <serial_conn.h>
#endif

enum class SwitchLr {Low, Middle, High};
enum class SwitchM {Low, High};

static const uint8_t TGYIA6C_PROTOCOL_LENGTH    = 0x20;
static const uint8_t TGYIA6C_PROTOCOL_OVERHEAD  = 3; // <len><cmd><data....><chkl><chkh>
static const uint8_t TGYIA6C_PROTOCOL_CHANNELS  = 10;
static const uint8_t TGYIA6C_PROTOCOL_COMMAND40 = 0x40; // Command is always 0x40

static const uint8_t TGYIA6C_PARSE_LENGTH  = 0x00;
static const uint8_t TGYIA6C_PARSE_DATA    = 0x01;
static const uint8_t TGYIA6C_PARSE_CHKSUML = 0x02;
static const uint8_t TGYIA6C_PARSE_CHKSUMH = 0x03;
static const uint8_t TGYIA6C_PARSE_DISCARD = 0x04;

static const uint8_t TGYIA6C_RAW_CHANNEL_COMMAND = 0x00;

static const uint8_t TGYIA6C_CHANNEL_GIMBAL_LEFT_X  = 0x03;
static const uint8_t TGYIA6C_CHANNEL_GIMBAL_LEFT_Y  = 0x02;
static const uint8_t TGYIA6C_CHANNEL_GIMBAL_RIGHT_X = 0x00;
static const uint8_t TGYIA6C_CHANNEL_GIMBAL_RIGHT_Y = 0x01;
static const uint8_t TGYIA6C_CHANNEL_KNOB           = 0x06;
static const uint8_t TGYIA6C_CHANNEL_SWITCH_LEFT    = 0x07;
static const uint8_t TGYIA6C_CHANNEL_SWITCH_RIGHT   = 0x04; // Currently not available!
static const uint8_t TGYIA6C_CHANNEL_SWITCH_MIDDLE  = 0x05;

static const uint16_t TGYIA6C_RAW_SWITCH_LR_LOW    = 2000;
static const uint16_t TGYIA6C_RAW_SWITCH_LR_MIDDLE = 1500;
static const uint16_t TGYIA6C_RAW_SWITCH_LR_HIGH   = 1000;

static const uint16_t TGYIA6C_RAW_SWITCH_M_LOW   = 2000;
static const uint16_t TGYIA6C_RAW_SWITCH_M_HIGH  = 1000;

static const double TGYIA6C_GIMBAL_OFFSET = -1000;
static const double TGYIA6C_GIMBAL_SCALE  = 0.001;

static const double TGYIA6C_KNOB_OFFSET = -1000;
static const double TGYIA6C_KNOB_SCALE  = 0.001;

static const std::map<uint16_t, SwitchLr> SwitchLrMap =
{
    {TGYIA6C_RAW_SWITCH_LR_LOW, SwitchLr::Low},
    {TGYIA6C_RAW_SWITCH_LR_MIDDLE, SwitchLr::Middle},
    {TGYIA6C_RAW_SWITCH_LR_HIGH, SwitchLr::High}
};

static const std::map<uint16_t, SwitchM> SwitchMMap =
{
    {TGYIA6C_RAW_SWITCH_M_LOW, SwitchM::Low},
    {TGYIA6C_RAW_SWITCH_M_HIGH, SwitchM::High}
};

static const uint16_t TGYIA6C_INIT_GIMBAL_LEFT_X  = 1500; // Centered
static const uint16_t TGYIA6C_INIT_GIMBAL_LEFT_Y  = 1000; // Low (default, throttle low)
static const uint16_t TGYIA6C_INIT_GIMBAL_RIGHT_X = 1500; // Centered
static const uint16_t TGYIA6C_INIT_GIMBAL_RIGHT_Y = 1500; // Centered
static const uint16_t TGYIA6C_INIT_KNOB           = 1000; // Zero
static const uint16_t TGYIA6C_INIT_SWITCH_LEFT    = TGYIA6C_RAW_SWITCH_LR_HIGH; // High
static const uint16_t TGYIA6C_INIT_SWITCH_RIGHT   = TGYIA6C_RAW_SWITCH_LR_HIGH; // High
static const uint16_t TGYIA6C_INIT_SWITCH_MIDDLE  = TGYIA6C_RAW_SWITCH_M_HIGH; // High

static const uint8_t TGYIA6C_STATUS_OK       = 0x00;
static const uint8_t TGYIA6C_STATUS_ERR_INIT = 0x01;

static const uint8_t TGYIA6C_BUF_SIZE = 255;

static const Mode TGYIA6C_SERIAL_MODE = (O_RDWR | O_NOCTTY | O_NDELAY);
static const ControlFlags TGYIA6C_SERIAL_OPT =
{
    (B115200 | CS8 | CLOCAL | CREAD), // 115200 bps
    IGNPAR,
    0,
    0
};

class Tgyia6c
{
public:
    Tgyia6c(SerialConn* serial_conn);

    void update();

    double get_gimbal_left_x();
    double get_gimbal_left_y();

    double get_gimbal_right_x();
    double get_gimbal_right_y();

    double get_knob();

    SwitchLr get_switch_left();
    SwitchLr get_switch_right();
    SwitchM get_switch_middle();

    uint8_t get_status();
private:
    uint8_t _status = TGYIA6C_STATUS_OK;

    uint8_t _buf[TGYIA6C_BUF_SIZE] = {0};
    uint32_t _n_bytes = 0;

    uint8_t _parse_state = TGYIA6C_PARSE_DISCARD;

    uint8_t _pkg_len = 0;
    uint16_t _pkg_chksum = 0;
    uint8_t _rec_chksum_l = 0;

    uint8_t _raw_channel_idx = 0;
    uint8_t _raw_channel[TGYIA6C_PROTOCOL_LENGTH] = {0};

    uint16_t _channel[TGYIA6C_PROTOCOL_CHANNELS] = {0};

    SerialConn* _serial_conn;

    void _open_serial_conn();
    void _set_init_values();
    void _read_to_buffer();
    void _parse_buffer();
};

#endif /* TGYIA6C_H */
