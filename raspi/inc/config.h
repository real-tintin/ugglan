#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <filesystem>
#include <vector>
#include <logger.h>
#include <utils.h>
#include <attitude_estimation.h>
#include <pilot_control.h>

namespace config
{
    class Config
    {
    public:
        void load();
        void print_to_debug();

        std::filesystem::path get_data_log_root() { return _data_log_root; }

        LogLevel get_logger_level() { return _logger_level; }

        std::string get_serial_device() { return _serial_device; }

        std::string get_i2c_device_imu() { return _i2c_device_imu; }
        std::string get_i2c_device_esc() { return _i2c_device_esc; }

        AttEstConfig get_att_est() { return _att_est; }

        PilotCtrlConfig get_pilot_ctrl() { return _pilot_ctrl; }
    private:
        std::vector<std::string> _env_lines;

        std::filesystem::path _data_log_root;

        LogLevel _logger_level;

        std::string _serial_device;

        std::string _i2c_device_imu;
        std::string _i2c_device_esc;

        AttEstConfig _att_est;

        PilotCtrlConfig _pilot_ctrl;

        void _load_data_log_root();
        void _load_logger_level();
        void _load_bus_devices();
        void _load_att_est();
        void _load_pilot_ctrl();

        LogLevel _log_str_to_level(std::string);

        template <typename T>
        void _read_env(T &dst, std::string name)
        {
            std::string _env_line = name + ": ";
            utils::read_env(dst, name);

            if constexpr (!std::is_same<T, std::string>::value)
            {
                _env_line += std::to_string(dst);
            }
            else
            {
                _env_line += dst;
            }

            _env_lines.push_back(_env_line);
        }
    };
}

#endif /* CONFIG_H */
