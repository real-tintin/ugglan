#include <pololu_alt_imu.hpp>

PololuAltImu::PololuAltImu(I2cConn &i2c_conn, std::string sensor_name) : _i2c_conn(i2c_conn), _sensor_name(sensor_name)
{
    _open_i2c_conn();
}

PololuAltImu::~PololuAltImu()
{
    _deallocate_buffer();
    _close_i2c_conn();
}

void PololuAltImu::update()
{
    bool did_read = true;

    for (auto const &it : _read_map)
    {
        uint8_t reg = it.first;
        uint8_t size = it.second;

        did_read &= _i2c_conn.read_block_data(reg | POLOLU_AUTO_INCREMENT, size, _get_buffer(reg));
    }

    if (!did_read)
    {
        _status |= POLOLU_STATUS_ERR_READ;
    }
    else
    {
        _status &= ~POLOLU_STATUS_ERR_READ;
    }
}

uint8_t PololuAltImu::get_status()
{
    return _status;
}

void PololuAltImu::_open_i2c_conn()
{
    if (!_i2c_conn.open())
    {
        _status = POLOLU_STATUS_ERR_INIT;
    }
}

void PololuAltImu::_close_i2c_conn()
{
    _i2c_conn.close();
}

void PololuAltImu::_setup(ConfigMap config_map, ReadMap read_map)
{
    _config_map = config_map;
    _read_map = read_map;

    _write_config();
    _allocate_buffer();
}

void PololuAltImu::_write_config()
{
    bool did_read = true;
    bool did_write = true;

    for (auto const &it : _config_map)
    {
        uint8_t reg = it.first;
        uint8_t exp_data = it.second;
        uint8_t act_data;

        did_read &= _i2c_conn.read_byte_data(reg, &act_data);

        if (exp_data != act_data)
        {
            _write_config_log_msg(reg, exp_data, act_data);
            did_write &= _i2c_conn.write_byte_data(reg, exp_data);
        }
    }

    if (!did_read || !did_write)
    {
        _status |= POLOLU_STATUS_ERR_CONF;
    }
}

void PololuAltImu::_write_config_log_msg(uint8_t reg, uint8_t exp_data, uint8_t act_data)
{
    logger.debug(_sensor_name + " updating config: " + common_utils::byte_to_hex_str(reg) + ": " +
                 common_utils::byte_to_bit_str(act_data) + " -> " + common_utils::byte_to_bit_str(exp_data));
}

void PololuAltImu::_allocate_buffer()
{
    for (auto const &it : _read_map)
    {
        uint8_t reg = it.first;
        uint8_t size = it.second;

        uint8_t *buf = new uint8_t[size];
        _buffer.insert({reg, buf});
    }
}

void PololuAltImu::_deallocate_buffer()
{
    for (auto const &it : _buffer)
    {
        uint8_t *buf = it.second;
        delete[] buf;
    }
}

uint8_t *PololuAltImu::_get_buffer(uint8_t reg)
{
    auto it = _buffer.find(reg);

    if (it != _buffer.end())
    {
        return it->second;
    }

    throw std::out_of_range("Register was not found in buffer");
}
