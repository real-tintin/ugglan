#include <pololu_alt_imu.h>

PololuAltImu::PololuAltImu(I2cConn* i2c_conn) : _i2c_conn{ i2c_conn }
{
}

PololuAltImu::~PololuAltImu()
{
    for (auto const& pair: _buffer)
    {
        uint8_t* buf = pair.second;
        delete[] buf;
    }
}

void PololuAltImu::update()
{
    bool did_read = true;

    for (auto const &pair: _read_map)
    {
        did_read = _i2c_conn->read_block_data(pair.first | POLOLU_AUTO_INCREMENT, pair.second, _get_buffer(pair.first));
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

void PololuAltImu::_setup(ConfigMap config_map, ReadMap read_map)
{
    _config_map = config_map;
    _read_map = read_map;

    if (!_i2c_conn->open())
    {
        _status = POLOLU_STATUS_ERR_INIT;
    }
    else
    {
        bool did_write = true;

        for (auto const& pair: _config_map)
        {
            did_write = _i2c_conn->write_byte_data(pair.first, pair.second);
        }

        if (!did_write)
        {
            _status |= POLOLU_STATUS_ERR_CONF;
        }
    }

    for (auto const& pair: _read_map)
    {
        uint8_t* buf = new uint8_t[pair.second];
        _buffer.insert({pair.first, buf});
    }
}

uint8_t* PololuAltImu::_get_buffer(uint8_t reg)
{
    auto it = _buffer.find(reg);

    if (it != _buffer.end())
    {
        return it->second;
    }

    throw std::out_of_range("Register was not found in buffer");
}
