#include <filesystem>
#include <string>
#include <vector>
#include <thread>
#include <data_log_queue.h>
#include <data_logger.h>
#include <task.h>
#include <utils.h>
#include <lsm303d.h>
#include <l3gd20h.h>
#include <lps25h.h>
#include <afro_esc.h>
#include <tgyia6c.h>

#if !defined(UNIT_TEST)

static const std::filesystem::path DATA_LOG_ROOT = get_env_str("DATA_LOG_ROOT");
static const std::string LOGGER_LEVEL = get_env_str("LOGGER_LEVEL");

static const uint32_t TASK_ACC_MAG_EXEC_PERIOD_MS     = 20; // 50 Hz
static const uint32_t TASK_GYRO_EXEC_PERIOD_MS        = 20; // 50 Hz
static const uint32_t TASK_BAROMETER_EXEC_PERIOD_MS   = 100; // 10 Hz
static const uint32_t TASK_ESC_READ_EXEC_PERIOD_MS    = 100; // 10 Hz
static const uint32_t TASK_ESC_WRITE_EXEC_PERIOD_MS   = 10; // 100 Hz
static const uint32_t TASK_RC_RECEIVER_EXEC_PERIOD_MS = 20; // 50 Hz
static const uint32_t TASK_DATA_LOGGER_EXEC_PERIOD_MS = 10; // 100 Hz

static const uint8_t I2C_ADDRESS_ACC_MAG   = 0x1d;
static const uint8_t I2C_ADDRESS_GYRO      = 0x6b;
static const uint8_t I2C_ADDRESS_BAROMETER = 0x5d;

static const uint8_t I2C_ADDRESS_ESC_0 = 0x2a;
static const uint8_t I2C_ADDRESS_ESC_1 = 0x2b;
static const uint8_t I2C_ADDRESS_ESC_2 = 0x2c;
static const uint8_t I2C_ADDRESS_ESC_3 = 0x2d;

static const std::string SERIAL_DEVICE_RC_RECEIVER = "/dev/ttyAMA0";

static const uint32_t MAIN_SLEEP_MS = 1000;

class TaskAccMag : public Task
{
public:
    TaskAccMag(uint32_t exec_period_ms, void (*exec_period_exceeded_cb)(), std::string name,
               uint8_t i2c_address, DataLogQueue& data_log_queue) :
        Task(exec_period_ms, exec_period_exceeded_cb, name),
             _i2c_conn(i2c_address), _acc_mag(_i2c_conn), _data_log_queue(data_log_queue) {}
protected:
    void _execute()
    {
        _acc_mag.update();

        _data_log_queue.push(_acc_mag.get_acceleration_x(), DataLogSignal::ImuAccelerationX);
        _data_log_queue.push(_acc_mag.get_acceleration_y(), DataLogSignal::ImuAccelerationY);
        _data_log_queue.push(_acc_mag.get_acceleration_z(), DataLogSignal::ImuAccelerationZ);

        _data_log_queue.push(_acc_mag.get_magnetic_field_x(), DataLogSignal::ImuMagneticFieldX);
        _data_log_queue.push(_acc_mag.get_magnetic_field_y(), DataLogSignal::ImuMagneticFieldY);
        _data_log_queue.push(_acc_mag.get_magnetic_field_z(), DataLogSignal::ImuMagneticFieldZ);

        _data_log_queue.push(_acc_mag.get_status(), DataLogSignal::ImuAccMagStatus);
    }
private:
    I2cConn _i2c_conn;
    Lsm303d _acc_mag;
    DataLogQueue& _data_log_queue;
};

class TaskGyro : public Task
{
public:
    TaskGyro(uint32_t exec_period_ms, void (*exec_period_exceeded_cb)(), std::string name,
             uint8_t i2c_address, DataLogQueue& data_log_queue) :
        Task(exec_period_ms, exec_period_exceeded_cb, name),
             _i2c_conn(i2c_address), _gyro(_i2c_conn), _data_log_queue(data_log_queue) {}
protected:
    void _execute()
    {
        _gyro.update();

        _data_log_queue.push(_gyro.get_angular_rate_x(), DataLogSignal::ImuAngularRateX);
        _data_log_queue.push(_gyro.get_angular_rate_y(), DataLogSignal::ImuAngularRateY);
        _data_log_queue.push(_gyro.get_angular_rate_z(), DataLogSignal::ImuAngularRateZ);

        _data_log_queue.push(_gyro.get_status(), DataLogSignal::ImuGyroStatus);
    }
private:
    I2cConn _i2c_conn;
    L3gd20h _gyro;
    DataLogQueue& _data_log_queue;
};

class TaskBarometer : public Task
{
public:
    TaskBarometer(uint32_t exec_period_ms, void (*exec_period_exceeded_cb)(), std::string name,
                  uint8_t i2c_address, DataLogQueue& data_log_queue) :
        Task(exec_period_ms, exec_period_exceeded_cb, name),
             _i2c_conn(i2c_address), _barometer(_i2c_conn), _data_log_queue(data_log_queue) {}
protected:
    void _execute()
    {
        _barometer.update();

        _data_log_queue.push(_barometer.get_pressure(), DataLogSignal::ImuPressure);
        _data_log_queue.push(_barometer.get_temperature(), DataLogSignal::ImuTemperature);

        _data_log_queue.push(_barometer.get_status(), DataLogSignal::ImuBarometerStatus);
    }
private:
    I2cConn _i2c_conn;
    Lps25h _barometer;
    DataLogQueue& _data_log_queue;
};

class TaskEsc : public Task
{
public:
    TaskEsc(uint32_t exec_period_ms, void (*exec_period_exceeded_cb)(), std::string name,
            uint8_t i2c_address_0, uint8_t i2c_address_1,
            uint8_t i2c_address_2, uint8_t i2c_address_3,
            DataLogQueue& data_log_queue) :
        Task(exec_period_ms, exec_period_exceeded_cb, name),
             _i2c_conn_0(i2c_address_0), _i2c_conn_1(i2c_address_1),
             _i2c_conn_2(i2c_address_2), _i2c_conn_3(i2c_address_3),
             _esc{AfroEsc(_i2c_conn_0), AfroEsc(_i2c_conn_1),
                  AfroEsc(_i2c_conn_2), AfroEsc(_i2c_conn_3)},
             _data_log_queue(data_log_queue) {}
protected:
    void _execute()
    {
        bool do_read = false;
        if (_read_counter >= _read_counter_reset) { _read_counter = 0; }

        for (uint8_t i_esc = 0; i_esc < _n_esc; i_esc++)
        {
            int16_t rpm_cmd = _get_rpm_cmd(i_esc);
            _esc[i_esc].write(rpm_cmd);

            if (do_read)
            {
                _esc[i_esc].read();

                _data_log_queue.push(_esc[i_esc].get_is_alive(), _sid_is_alive[i_esc]);
                _data_log_queue.push(_esc[i_esc].get_rpm(), _sid_rpm[i_esc]);
                _data_log_queue.push(_esc[i_esc].get_voltage(), _sid_voltage[i_esc]);
                _data_log_queue.push(_esc[i_esc].get_current(), _sid_current[i_esc]);
                _data_log_queue.push(_esc[i_esc].get_temperature(), _sid_temperature[i_esc]);
                _data_log_queue.push(_esc[i_esc].get_status(), _sid_status[i_esc]);
            }
        }

        _read_counter++;
    }
private:
    static const uint8_t _n_esc = 4;

    static constexpr DataLogSignal _sid_is_alive[_n_esc] = {
        DataLogSignal::EscIsAlive0, DataLogSignal::EscIsAlive1,
        DataLogSignal::EscIsAlive2, DataLogSignal::EscIsAlive3};

    static constexpr DataLogSignal _sid_rpm[_n_esc] = {
        DataLogSignal::EscRpm0, DataLogSignal::EscRpm1,
        DataLogSignal::EscRpm2, DataLogSignal::EscRpm3};

    static constexpr DataLogSignal _sid_voltage[_n_esc] = {
        DataLogSignal::EscVoltage0, DataLogSignal::EscVoltage1,
        DataLogSignal::EscVoltage2, DataLogSignal::EscVoltage3};

    static constexpr DataLogSignal _sid_current[_n_esc] = {
        DataLogSignal::EscCurrent0, DataLogSignal::EscCurrent1,
        DataLogSignal::EscCurrent2, DataLogSignal::EscCurrent3};

    static constexpr DataLogSignal _sid_temperature[_n_esc] = {
        DataLogSignal::EscTemperature0, DataLogSignal::EscTemperature1,
        DataLogSignal::EscTemperature2, DataLogSignal::EscTemperature3};

    static constexpr DataLogSignal _sid_status[_n_esc] = {
        DataLogSignal::EscStatus0, DataLogSignal::EscStatus1,
        DataLogSignal::EscStatus2, DataLogSignal::EscStatus3};

    static constexpr uint8_t _read_counter_reset =
        (TASK_ESC_READ_EXEC_PERIOD_MS / TASK_ESC_WRITE_EXEC_PERIOD_MS) - 1;
    uint8_t _read_counter = 0;

    I2cConn _i2c_conn_0, _i2c_conn_1, _i2c_conn_2, _i2c_conn_3;
    AfroEsc _esc[_n_esc];
    DataLogQueue& _data_log_queue;

    int16_t _get_rpm_cmd(uint8_t i_esc)
    {
        // TODO: Should be given by the state controller.
        double rc_gimbal_left_x;
        _data_log_queue.last_signal_data(&rc_gimbal_left_x, DataLogSignal::RcGimbalLeftX);

        int16_t rpm_cmd = rc_gimbal_left_x * 1000.0;
        if (rpm_cmd > 1000) { rpm_cmd = 1000; }
        if (rpm_cmd < 0) { rpm_cmd = 0; }

        return rpm_cmd;
    }
};

class TaskRcReceiver : public Task
{
public:
    TaskRcReceiver(uint32_t exec_period_ms, void (*exec_period_exceeded_cb)(), std::string name,
                   std::string serial_device, DataLogQueue& data_log_queue) :
        Task(exec_period_ms, exec_period_exceeded_cb, name),
             _serial_conn(serial_device), _rc(_serial_conn), _data_log_queue(data_log_queue) {}
protected:
    void _execute()
    {
        _rc.update();

        _data_log_queue.push(_rc.get_gimbal_left_x(), DataLogSignal::RcGimbalLeftX);
        _data_log_queue.push(_rc.get_gimbal_left_y(), DataLogSignal::RcGimbalLeftY);

        _data_log_queue.push(_rc.get_gimbal_right_x(), DataLogSignal::RcGimbalRightX);
        _data_log_queue.push(_rc.get_gimbal_right_y(), DataLogSignal::RcGimbalRightY);

        _data_log_queue.push(_rc.get_switch_left(), DataLogSignal::RcSwitchLeft);
        _data_log_queue.push(_rc.get_switch_right(), DataLogSignal::RcSwitchRight);
        _data_log_queue.push(_rc.get_switch_middle(), DataLogSignal::RcSwitchMiddle);

        _data_log_queue.push(_rc.get_knob(), DataLogSignal::RcKnob);
        _data_log_queue.push(_rc.get_status(), DataLogSignal::RcStatus);
    }
private:
    SerialConn _serial_conn;
    Tgyia6c _rc;
    DataLogQueue& _data_log_queue;
};

class TaskDataLogger : public Task
{
public:
    TaskDataLogger(uint32_t exec_period_ms, void (*exec_period_exceeded_cb)(), std::string name,
                   std::filesystem::path root_path, DataLogQueue& data_log_queue) :
        Task(exec_period_ms, exec_period_exceeded_cb, name),
             _data_logger(data_log_queue, root_path) {}
protected:
    void _setup() { _data_logger.start(); }
    void _execute() { _data_logger.pack(); }
    void _finish() { _data_logger.stop(); }
private:
    DataLogger _data_logger;
};

void set_logger_level()
{
    if      (LOGGER_LEVEL == "DEBUG") { logger.set_level(LogLevel::debug); }
    else if (LOGGER_LEVEL == "INFO")  { logger.set_level(LogLevel::info); }
    else if (LOGGER_LEVEL == "WARN")  { logger.set_level(LogLevel::warn); }
    else if (LOGGER_LEVEL == "ERROR") { logger.set_level(LogLevel::error); }
    else if (LOGGER_LEVEL == "OFF")   { logger.set_level(LogLevel::off); }
    else                              { /* Keep default. */ }
}

void print_for_debug()
{
    logger.debug("DATA_LOG_ROOT: " + DATA_LOG_ROOT.string());
    logger.debug("LOGGER_LEVEL: " + LOGGER_LEVEL);
}

void wait_for_shutdown(DataLogQueue& data_log_queue)
{
    SwitchM rc_switch_middle = SwitchM::High;

    while (rc_switch_middle == SwitchM::High)
    {
        data_log_queue.last_signal_data(&rc_switch_middle, DataLogSignal::RcSwitchMiddle, SwitchM::High);

        switch(rc_switch_middle)
        {
            case SwitchM::Low:
                logger.debug("Rc middle switch: low. Will shutdown.");
                break;
            case SwitchM::High:
                logger.debug("Rc middle switch: high. Keep running.");
                break;
            default:
                break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(MAIN_SLEEP_MS));
    }
}

int main()
{
    DataLogQueue data_log_queue;
    std::vector<std::unique_ptr<Task>> tasks;

    tasks.emplace_back(new TaskAccMag(TASK_ACC_MAG_EXEC_PERIOD_MS, nullptr, "ImuAccMag",
                                      I2C_ADDRESS_ACC_MAG, data_log_queue));

    tasks.emplace_back(new TaskGyro(TASK_GYRO_EXEC_PERIOD_MS, nullptr, "ImuGyro",
                                    I2C_ADDRESS_GYRO, data_log_queue));

    tasks.emplace_back(new TaskBarometer(TASK_BAROMETER_EXEC_PERIOD_MS, nullptr, "ImuBarometer",
                                         I2C_ADDRESS_BAROMETER, data_log_queue));

    tasks.emplace_back(new TaskEsc(TASK_ESC_READ_EXEC_PERIOD_MS, nullptr, "Esc",
                                      I2C_ADDRESS_ESC_0, I2C_ADDRESS_ESC_1,
                                      I2C_ADDRESS_ESC_2, I2C_ADDRESS_ESC_3,
                                      data_log_queue));

    tasks.emplace_back(new TaskRcReceiver(TASK_RC_RECEIVER_EXEC_PERIOD_MS, nullptr, "RcReceiver",
                                          SERIAL_DEVICE_RC_RECEIVER, data_log_queue));

    tasks.emplace_back(new TaskDataLogger(TASK_DATA_LOGGER_EXEC_PERIOD_MS, nullptr, "DataLogger",
                                          DATA_LOG_ROOT, data_log_queue));

    set_logger_level();
    print_for_debug();

    for(auto const& task: tasks) { task->launch(); }
    wait_for_shutdown(data_log_queue);
    for(auto const& task: tasks) { task->teardown(); }

    return 0;
}

#endif /* !defined(UNIT_TEST) */
