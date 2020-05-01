#ifndef POLOLU_ALT_IMU_H
#define POLOLU_ALT_IMU_H

#include <cstdint>
#include <map>
#include <vector>
#include <algorithm>

#if defined(UNIT_TEST)
#include <i2c_conn_stub.h>
#else
#include <i2c_conn.h>
#endif

static const uint8_t POLOLU_STATUS_OK       = 0x00;
static const uint8_t POLOLU_STATUS_ERR_INIT = 0x01;
static const uint8_t POLOLU_STATUS_ERR_CONF = 0x02;
static const uint8_t POLOLU_STATUS_ERR_READ = 0x04;

typedef std::map<uint8_t, uint8_t> ConfigMap;
typedef std::map<uint8_t, uint8_t> ReadMap;
typedef std::map<uint8_t, uint8_t*> BufferMap;

class PololuAltImu
{
public:
    PololuAltImu(I2cConn* i2c_conn, ConfigMap config_map, ReadMap read_map) :
        _i2c_conn{ i2c_conn },
        _config_map{ config_map },
        _read_map{ read_map }
    {
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

            if (!did_write) { _status |= POLOLU_STATUS_ERR_CONF; }
        }

        for (auto const& pair: _read_map)
        {
            uint8_t* buf = new uint8_t[pair.second];
            _buffer.insert({pair.first, buf});
        }

    }

    ~PololuAltImu()
    {
        for (auto const& pair: _buffer)
        {
            uint8_t* buf = pair.second;
            delete[] buf;
        }
    }

    void update()
    {
        bool did_read = true;

        for (auto const &pair: _read_map)
        {
            did_read = _i2c_conn->read_block_data(pair.first, pair.second, get_buffer(pair.first));
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

    uint8_t* get_buffer(uint8_t reg)
    {
        auto it = _buffer.find(reg);

        if (it != _buffer.end())
        {
            return it->second;
        }
        throw std::out_of_range("Register was not found in buffer");
    }

    uint8_t get_status() { return _status; }
private:
    uint8_t _status = POLOLU_STATUS_OK;

    I2cConn* _i2c_conn;

    ConfigMap _config_map;
    ReadMap _read_map;
    BufferMap _buffer;
};

#endif
