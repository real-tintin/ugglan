#ifndef POLOLU_ALT_IMU_HPP
#define POLOLU_ALT_IMU_HPP

#include <algorithm>
#include <cstdint>
#include <map>

#include <common_utils.hpp>
#include <i2c_conn.hpp>
#include <logger.hpp>

inline const uint8_t POLOLU_AUTO_INCREMENT = 0b10000000;

inline const uint8_t POLOLU_STATUS_OK = 0x00;
inline const uint8_t POLOLU_STATUS_ERR_INIT = 0x01;
inline const uint8_t POLOLU_STATUS_ERR_CONF = 0x02;
inline const uint8_t POLOLU_STATUS_ERR_READ = 0x04;

typedef std::map<uint8_t, uint8_t> ConfigMap;
typedef std::map<uint8_t, uint8_t> ReadMap;
typedef std::map<uint8_t, uint8_t *> BufferMap;

class PololuAltImu
{
public:
    PololuAltImu(I2cConn &i2c_conn, std::string sensor_name);

    ~PololuAltImu();

    void update();

    uint8_t get_status() const;

protected:
    void _open_i2c_conn();
    void _close_i2c_conn();

    void _setup(ConfigMap config_map, ReadMap read_map);

    void _write_config();
    void _write_config_log_msg(uint8_t reg, uint8_t exp_data, uint8_t act_data);

    void _allocate_buffer();
    void _deallocate_buffer();

    uint8_t *_get_buffer(uint8_t reg);

private:
    I2cConn &_i2c_conn;
    std::string _sensor_name;

    uint8_t _status = POLOLU_STATUS_OK;

    ConfigMap _config_map;
    ReadMap _read_map;
    BufferMap _buffer;
};

#endif /* POLOLU_ALT_IMU_HPP */
