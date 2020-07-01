#include <tgyia6c.h>

Tgyia6c::Tgyia6c(SerialConn* serial_conn) :
    _serial_conn(serial_conn)
{
    _open_serial_conn();
}

void Tgyia6c::update()
{
    _read_to_buffer();
    _parse_buffer();
}

// Returns gimbal left X [0-1000]
uint16_t Tgyia6c::get_gimbal_left_x()
{
    return 0;
    // TODO
}

// Returns gimbal left Y [0-1000]
uint16_t Tgyia6c::get_gimbal_left_y()
{
    return 0;
    // TODO
}

// Returns gimbal right X [0-1000]
uint16_t Tgyia6c::get_gimbal_right_x()
{
    return 0;
    // TODO
}

// Returns gimbal right Y [0-1000]
uint16_t Tgyia6c::get_gimbal_right_y()
{
    return 0;
    // TODO
}

// Returns knob [0-1000]
uint16_t Tgyia6c::get_knob()
{
    return 0;
    // TODO
}

// Returns switch left [low, mid, high]
uint8_t Tgyia6c::get_switch_left()
{
    return 0;
    // TODO
}

// Returns switch right [low, mid, high]
uint8_t Tgyia6c::get_switch_right()
{
    return 0;
    // TODO
}

// Returns switch middle [low, high]
uint8_t Tgyia6c::get_switch_middle()
{
    return 0;
    // TODO
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

void Tgyia6c::_read_to_buffer()
{
    _n_bytes = _serial_conn->read(_buf, TGYIA6C_BUF_SIZE); // Non-blocking mode
}

void Tgyia6c::_parse_buffer()
{
    // TODO
}
