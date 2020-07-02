#include <tgyia6c.h>

Tgyia6c::Tgyia6c(SerialConn* serial_conn) :
    _serial_conn(serial_conn)
{
    _open_serial_conn();
    _set_init_values();
}

void Tgyia6c::update()
{
    _read_to_buffer();
    _parse_buffer();
}

// Returns gimbal left X [0.0-1.0]
double Tgyia6c::get_gimbal_left_x()
{
	return (TGYIA6C_GIMBAL_SCALE *
        (double(_channel[TGYIA6C_CHANNEL_GIMBAL_LEFT_X]) + TGYIA6C_GIMBAL_OFFSET));
}

// Returns gimbal left Y [0.0-1.0]
double Tgyia6c::get_gimbal_left_y()
{
	return (TGYIA6C_GIMBAL_SCALE *
        (double(_channel[TGYIA6C_CHANNEL_GIMBAL_LEFT_Y]) + TGYIA6C_GIMBAL_OFFSET));
}

// Returns gimbal right X [0.0-1.0]
double Tgyia6c::get_gimbal_right_x()
{
	return (TGYIA6C_GIMBAL_SCALE *
        (double(_channel[TGYIA6C_CHANNEL_GIMBAL_RIGHT_X]) + TGYIA6C_GIMBAL_OFFSET));
}

// Returns gimbal right Y [0.0-1.0]
double Tgyia6c::get_gimbal_right_y()
{
	return (TGYIA6C_GIMBAL_SCALE *
        (double(_channel[TGYIA6C_CHANNEL_GIMBAL_RIGHT_Y]) + TGYIA6C_GIMBAL_OFFSET));
}

// Returns knob [0.0-1.0]
double Tgyia6c::get_knob()
{
	return (TGYIA6C_KNOB_SCALE *
        (double(_channel[TGYIA6C_CHANNEL_KNOB]) + TGYIA6C_KNOB_OFFSET));
}

// Returns switch left [low, mid, high]
SwitchLr Tgyia6c::get_switch_left()
{
    return SwitchLrMap.find(_channel[TGYIA6C_CHANNEL_SWITCH_LEFT])->second;
}

// Returns switch right [low, mid, high]
SwitchLr Tgyia6c::get_switch_right()
{
    return SwitchLrMap.find(_channel[TGYIA6C_CHANNEL_SWITCH_RIGHT])->second;
}

// Returns switch middle [low, high]
SwitchM Tgyia6c::get_switch_middle()
{
    return SwitchMMap.find(_channel[TGYIA6C_CHANNEL_SWITCH_MIDDLE])->second;
}

uint8_t Tgyia6c::get_status()
{
    return _status;
}

void Tgyia6c::_open_serial_conn()
{
    if (!_serial_conn->open(TGYIA6C_SERIAL_MODE, TGYIA6C_SERIAL_OPT))
    {
        _status = TGYIA6C_STATUS_ERR_INIT;
    }
}

void Tgyia6c::_set_init_values()
{
    _channel[TGYIA6C_CHANNEL_GIMBAL_LEFT_X] = TGYIA6C_INIT_GIMBAL_LEFT_X;
    _channel[TGYIA6C_CHANNEL_GIMBAL_LEFT_Y] = TGYIA6C_INIT_GIMBAL_LEFT_Y;

    _channel[TGYIA6C_CHANNEL_GIMBAL_RIGHT_X] = TGYIA6C_INIT_GIMBAL_RIGHT_X;
    _channel[TGYIA6C_CHANNEL_GIMBAL_RIGHT_Y] = TGYIA6C_INIT_GIMBAL_RIGHT_Y;

    _channel[TGYIA6C_CHANNEL_KNOB] = TGYIA6C_INIT_KNOB;

    _channel[TGYIA6C_CHANNEL_SWITCH_LEFT] = TGYIA6C_INIT_SWITCH_LEFT;
    _channel[TGYIA6C_CHANNEL_SWITCH_RIGHT] = TGYIA6C_INIT_SWITCH_RIGHT;
    _channel[TGYIA6C_CHANNEL_SWITCH_MIDDLE] = TGYIA6C_INIT_SWITCH_MIDDLE;
}

void Tgyia6c::_read_to_buffer()
{
    _n_bytes = _serial_conn->read(_buf, TGYIA6C_BUF_SIZE); // Non-blocking mode
}

void Tgyia6c::_parse_buffer()
{
    // TODO
}
