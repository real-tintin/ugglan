/*
TGYIA6C radio receiver parsing FlySky I-BUS, see data sheet
https://hobbyking.com/en_us/turnigy-ia6c-ppm-sbus-receiver.html
*/

#ifndef TGYIA6C_H
#define TGYIA6C_H

#include <cstdint>
#include <science_const.h>

#if defined(UNIT_TEST)
#include <serial_conn_stub.h>
#else
#include <serial_conn.h>
#endif

static const uint8_t TGYIA6C_STATUS_OK       = 0x00;
static const uint8_t TGYIA6C_STATUS_ERR_INIT = 0x01;
static const uint8_t TGYIA6C_STATUS_ERR_READ = 0x02;

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

	uint16_t get_gimbal_left_x();
	uint16_t get_gimbal_left_y();

	uint16_t get_gimbal_right_x();
	uint16_t get_gimbal_right_y();

	uint16_t get_knob();

	uint8_t get_switch_left();
	uint8_t get_switch_right();
	uint8_t get_switch_middle();

	uint8_t get_status();
private:
    uint8_t _status = TGYIA6C_STATUS_OK;
	SerialConn* _serial_conn;

	void _open_serial_conn();
    void _parse_buffer();
};

#endif /* TGYIA6C_H */
