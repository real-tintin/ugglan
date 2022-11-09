#include <tgyia6c.h>

Tgyia6c::Tgyia6c(SerialConn& serial_conn) :
    _serial_conn(serial_conn)
{
    _open_serial_conn();
    _set_init_values();
}

Tgyia6c::~Tgyia6c()
{
    _close_serial_conn();
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
    if (!_serial_conn.open(TGYIA6C_SERIAL_MODE, TGYIA6C_SERIAL_OPT))
    {
        _status = TGYIA6C_STATUS_ERR_INIT;
    }
}

void Tgyia6c::_close_serial_conn()
{
    _serial_conn.close();
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
    _n_bytes = _serial_conn.read(_buf, TGYIA6C_BUF_SIZE); // Non-blocking mode
}

void Tgyia6c::_parse_buffer()
{
    uint8_t byte;
    uint16_t rec_chksum;

    for (uint32_t i_byte = 0; i_byte < _n_bytes; i_byte++)
    {
        byte = _buf[i_byte];

        if (_parse_state == TGYIA6C_PARSE_DISCARD)
        {
            _parse_state = TGYIA6C_PARSE_LENGTH;
        }

        switch (_parse_state)
        {
            case TGYIA6C_PARSE_LENGTH:

                if ((byte <= TGYIA6C_PROTOCOL_LENGTH) &&
                    (byte > TGYIA6C_PROTOCOL_OVERHEAD))
                {
                  _raw_channel_idx = 0;
                  _pkg_len = byte - TGYIA6C_PROTOCOL_OVERHEAD;
                  _pkg_chksum = 0xFFFF - byte;

                  _parse_state = TGYIA6C_PARSE_DATA;
                }
                else
                {
                  _parse_state = TGYIA6C_PARSE_DISCARD;
                }

                break;

            case TGYIA6C_PARSE_DATA:

                _raw_channel[_raw_channel_idx++] = byte;
                _pkg_chksum -= byte;

                if (_raw_channel_idx >= _pkg_len)
                {
                  _parse_state = TGYIA6C_PARSE_CHKSUML;
                }

                break;

            case TGYIA6C_PARSE_CHKSUML:

                _rec_chksum_l = byte;
                _parse_state = TGYIA6C_PARSE_CHKSUMH;

                break;

            case TGYIA6C_PARSE_CHKSUMH:

                rec_chksum = (byte << 8) + _rec_chksum_l;

                if (_pkg_chksum == rec_chksum)
                {
                    switch (_raw_channel[TGYIA6C_RAW_CHANNEL_COMMAND])
                    {
                        case TGYIA6C_PROTOCOL_COMMAND40:

                            for (uint8_t i_raw = 1; i_raw < (TGYIA6C_PROTOCOL_CHANNELS * 2 + 1); i_raw += 2)
                            {
                                _channel[i_raw / 2] = (_raw_channel[i_raw + 1] << 8) | _raw_channel[i_raw];
                            }

                            break;

                        default:
                            break;

                    }
                }
                _parse_state = TGYIA6C_PARSE_DISCARD;

                break;

            case TGYIA6C_PARSE_DISCARD:
            default:
                break;
        }
    }
}
