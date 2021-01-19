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
#include <attitude_estimation.h>
#include <motor_control.h>
#include <pilot_control.h>

#if !defined(UNIT_TEST)

static const std::filesystem::path DATA_LOG_ROOT = get_env_str("DATA_LOG_ROOT");
static const std::string LOGGER_LEVEL = get_env_str("LOGGER_LEVEL");

static const uint32_t TASK_ACC_MAG_EXEC_PERIOD_MS            = 20;  // 50 Hz
static const uint32_t TASK_GYRO_EXEC_PERIOD_MS               = 20;  // 50 Hz
static const uint32_t TASK_BAROMETER_EXEC_PERIOD_MS          = 100; // 10 Hz
static const uint32_t TASK_STATE_EST_AND_CTRL_EXEC_PERIOD_MS = 20;  // 50 Hz
static const uint32_t TASK_ESC_READ_EXEC_PERIOD_MS           = 200; // 5 Hz
static const uint32_t TASK_ESC_WRITE_EXEC_PERIOD_MS          = 20;  // 50 Hz
static const uint32_t TASK_RC_RECEIVER_EXEC_PERIOD_MS        = 20;  // 50 Hz
static const uint32_t TASK_DATA_LOGGER_EXEC_PERIOD_MS        = 10;  // 100 Hz

static const uint8_t I2C_ADDRESS_ACC_MAG   = 0x1d;
static const uint8_t I2C_ADDRESS_GYRO      = 0x6b;
static const uint8_t I2C_ADDRESS_BAROMETER = 0x5d;

static const uint8_t I2C_ADDRESS_ESC_0 = 0x2a;
static const uint8_t I2C_ADDRESS_ESC_1 = 0x2b;
static const uint8_t I2C_ADDRESS_ESC_2 = 0x2c;
static const uint8_t I2C_ADDRESS_ESC_3 = 0x2d;

static const uint8_t N_ESC = 4;

static const std::string SERIAL_DEVICE_RC_RECEIVER = "/dev/ttyAMA0";

static const std::string I2C_DEVICE_IMU = "/dev/i2c-1"; // Using HW in fast mode (400 kHz)
static const std::string I2C_DEVICE_ESC = "/dev/i2c-4"; // Using SW in normal mode (100 kHz)

static const uint32_t MAIN_SLEEP_MS = 1000;

static const double ATT_EST_INPUT_SAMPLE_RATE_S = 20 / 1e3; // Acc, Mag & Gyro run at 50 Hz

enum class TaskId {
    AccMag,
    Gyro,
    Barometer,
    EscRead0,
    EscRead1,
    EscRead2,
    EscRead3,
    EscWrite0,
    EscWrite1,
    EscWrite2,
    EscWrite3,
    RcReceiver,
    StateEstAndCtrl,
    DataLogger
};

class TaskAccMag : public Task
{
public:
    TaskAccMag(uint32_t exec_period_ms, std::string name,
               std::string i2c_device, uint8_t i2c_address, DataLogQueue& data_log_queue) :
         Task(exec_period_ms, name),
              _i2c_conn(i2c_device, i2c_address), _acc_mag(_i2c_conn),
              _data_log_queue(data_log_queue) {}
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
        _data_log_queue.push(uint8_t(TaskId::AccMag), DataLogSignal::TaskExecute);
    }
private:
    I2cConn _i2c_conn;
    Lsm303d _acc_mag;
    DataLogQueue& _data_log_queue;
};

class TaskGyro : public Task
{
public:
    TaskGyro(uint32_t exec_period_ms, std::string name,
             std::string i2c_device, uint8_t i2c_address, DataLogQueue& data_log_queue) :
        Task(exec_period_ms, name),
             _i2c_conn(i2c_device, i2c_address), _gyro(_i2c_conn),
             _data_log_queue(data_log_queue) {}
protected:
    void _execute()
    {
        _gyro.update();

        _data_log_queue.push(_gyro.get_angular_rate_x(), DataLogSignal::ImuAngularRateX);
        _data_log_queue.push(_gyro.get_angular_rate_y(), DataLogSignal::ImuAngularRateY);
        _data_log_queue.push(_gyro.get_angular_rate_z(), DataLogSignal::ImuAngularRateZ);

        _data_log_queue.push(_gyro.get_status(), DataLogSignal::ImuGyroStatus);
        _data_log_queue.push(uint8_t(TaskId::Gyro), DataLogSignal::TaskExecute);
    }
private:
    I2cConn _i2c_conn;
    L3gd20h _gyro;
    DataLogQueue& _data_log_queue;
};

class TaskBarometer : public Task
{
public:
    TaskBarometer(uint32_t exec_period_ms, std::string name,
                  std::string i2c_device, uint8_t i2c_address, DataLogQueue& data_log_queue) :
             Task(exec_period_ms, name),
                  _i2c_conn(i2c_device, i2c_address), _barometer(_i2c_conn),
                  _data_log_queue(data_log_queue) {}
protected:
    void _execute()
    {
        _barometer.update();

        _data_log_queue.push(_barometer.get_pressure(), DataLogSignal::ImuPressure);
        _data_log_queue.push(_barometer.get_temperature(), DataLogSignal::ImuTemperature);

        _data_log_queue.push(_barometer.get_status(), DataLogSignal::ImuBarometerStatus);
        _data_log_queue.push(uint8_t(TaskId::Barometer), DataLogSignal::TaskExecute);
    }
private:
    I2cConn _i2c_conn;
    Lps25h _barometer;
    DataLogQueue& _data_log_queue;
};

class TaskStateEstAndCtrl : public Task
{
public:
    TaskStateEstAndCtrl(uint32_t exec_period_ms, std::string name,
                        double input_sample_rate_s, DataLogQueue& data_log_queue) :
                   Task(exec_period_ms, name),
                        _att(input_sample_rate_s), _pilot_ctrl(input_sample_rate_s),
                        _data_log_queue(data_log_queue) {}
protected:
    void _execute()
    {
        _exec_att_est();
        _exec_pilot_ctrl();
        _exec_motor_ctrl();

        _data_log_queue.push(uint8_t(TaskId::StateEstAndCtrl), DataLogSignal::TaskExecute);
    }
private:
    AttEstInput _att_in;
    AttEstimate _att_est;
    AttitudeEstimation _att;

    double _gimbal_left_x, _gimbal_left_y, _gimbal_right_x, _gimbal_right_y;

    PilotControl _pilot_ctrl;
    PilotCtrlRef _ctrl_ref;

    DataLogQueue& _data_log_queue;

    BodyControl _body_ctrl;
    MotorControl _motor_ctrl;

    static constexpr DataLogSignal _sid_motor_cmd[N_ESC] = {
        DataLogSignal::EscMotorCmd0, DataLogSignal::EscMotorCmd1,
        DataLogSignal::EscMotorCmd2, DataLogSignal::EscMotorCmd3};

    void _exec_att_est()
    {
        _data_log_queue.last_signal_data(&_att_in.acc_x, DataLogSignal::ImuAccelerationX);
        _data_log_queue.last_signal_data(&_att_in.acc_y, DataLogSignal::ImuAccelerationY);
        _data_log_queue.last_signal_data(&_att_in.acc_z, DataLogSignal::ImuAccelerationZ);

        _data_log_queue.last_signal_data(&_att_in.ang_rate_x, DataLogSignal::ImuAngularRateX);
        _data_log_queue.last_signal_data(&_att_in.ang_rate_y, DataLogSignal::ImuAngularRateY);
        _data_log_queue.last_signal_data(&_att_in.ang_rate_z, DataLogSignal::ImuAngularRateZ);

        _data_log_queue.last_signal_data(&_att_in.mag_field_x, DataLogSignal::ImuMagneticFieldX);
        _data_log_queue.last_signal_data(&_att_in.mag_field_y, DataLogSignal::ImuMagneticFieldY);
        _data_log_queue.last_signal_data(&_att_in.mag_field_z, DataLogSignal::ImuMagneticFieldZ);

        _att.update(_att_in);
        _att_est = _att.get_estimate();

        _data_log_queue.push(_att_est.roll, DataLogSignal::StateEstRoll);
        _data_log_queue.push(_att_est.pitch, DataLogSignal::StateEstPitch);
        _data_log_queue.push(_att_est.yaw, DataLogSignal::StateEstYaw);

        _data_log_queue.push(_att_est.roll_rate, DataLogSignal::StateEstRollRate);
        _data_log_queue.push(_att_est.pitch_rate, DataLogSignal::StateEstPitchRate);
        _data_log_queue.push(_att_est.yaw_rate, DataLogSignal::StateEstYawRate);

        _data_log_queue.push(_att.is_calibrated(), DataLogSignal::StateEstAttIsCalib);
    }

    void _exec_pilot_ctrl()
    {
        _data_log_queue.last_signal_data(&_gimbal_left_x, DataLogSignal::RcGimbalLeftX);
        _data_log_queue.last_signal_data(&_gimbal_left_y, DataLogSignal::RcGimbalLeftY);
        _data_log_queue.last_signal_data(&_gimbal_right_x, DataLogSignal::RcGimbalRightX);
        _data_log_queue.last_signal_data(&_gimbal_right_y, DataLogSignal::RcGimbalRightY);

        _ctrl_ref = tgyia6c_to_pilot_ctrl_ref(_gimbal_left_x, _gimbal_left_y,
                                              _gimbal_right_x, _gimbal_right_y);

        _pilot_ctrl.update(_att_est, _ctrl_ref);
        _body_ctrl = _pilot_ctrl.get_ctrl();

        _data_log_queue.push(_ctrl_ref.roll, DataLogSignal::StateCtrlRollRef);
        _data_log_queue.push(_ctrl_ref.pitch, DataLogSignal::StateCtrlPitchRef);
        _data_log_queue.push(_ctrl_ref.yaw_rate, DataLogSignal::StateCtrlYawRateRef);
        _data_log_queue.push(_ctrl_ref.f_z, DataLogSignal::StateCtrlFzRef);

        _data_log_queue.push(_body_ctrl.m_x, DataLogSignal::StateCtrlMx);
        _data_log_queue.push(_body_ctrl.m_y, DataLogSignal::StateCtrlMy);
        _data_log_queue.push(_body_ctrl.m_z, DataLogSignal::StateCtrlMz);
        _data_log_queue.push(_body_ctrl.f_z, DataLogSignal::StateCtrlFz);
    }

    void _exec_motor_ctrl()
    {
        _motor_ctrl = body_to_motor_controls(_body_ctrl);

        for (uint8_t i_esc = 0; i_esc < N_ESC; i_esc++)
        {
            _data_log_queue.push(_motor_ctrl[i_esc], _sid_motor_cmd[i_esc]);
        }
    }
};

class TaskEscRead : public Task
{
public:
    TaskEscRead(uint32_t exec_period_ms, std::string name,
                AfroEsc (&esc)[N_ESC], DataLogQueue& data_log_queue) :
           Task(exec_period_ms / N_ESC, name),
                _esc(esc), _data_log_queue(data_log_queue) {}
protected:
    void _execute()
    {
        uint8_t i_esc = _load_balance_step;
        _esc[i_esc].read();

        _data_log_queue.push(_esc[i_esc].get_is_alive(), _sid_is_alive[i_esc]);
        _data_log_queue.push(_esc[i_esc].get_angular_rate(), _sid_ang_rate[i_esc]);
        _data_log_queue.push(_esc[i_esc].get_voltage(), _sid_voltage[i_esc]);
        _data_log_queue.push(_esc[i_esc].get_current(), _sid_current[i_esc]);
        _data_log_queue.push(_esc[i_esc].get_temperature(), _sid_temperature[i_esc]);
        _data_log_queue.push(_esc[i_esc].get_status(), _sid_status[i_esc]);
        _data_log_queue.push(uint8_t(_task_read_ids[i_esc]), DataLogSignal::TaskExecute);

        _load_balance_step = (_load_balance_step + 1) % N_ESC;
    }
private:
    AfroEsc (&_esc)[N_ESC];
    DataLogQueue& _data_log_queue;

    static constexpr DataLogSignal _sid_is_alive[N_ESC] = {
        DataLogSignal::EscIsAlive0, DataLogSignal::EscIsAlive1,
        DataLogSignal::EscIsAlive2, DataLogSignal::EscIsAlive3};

    static constexpr DataLogSignal _sid_ang_rate[N_ESC] = {
        DataLogSignal::EscAngularRate0, DataLogSignal::EscAngularRate1,
        DataLogSignal::EscAngularRate2, DataLogSignal::EscAngularRate3};

    static constexpr DataLogSignal _sid_voltage[N_ESC] = {
        DataLogSignal::EscVoltage0, DataLogSignal::EscVoltage1,
        DataLogSignal::EscVoltage2, DataLogSignal::EscVoltage3};

    static constexpr DataLogSignal _sid_current[N_ESC] = {
        DataLogSignal::EscCurrent0, DataLogSignal::EscCurrent1,
        DataLogSignal::EscCurrent2, DataLogSignal::EscCurrent3};

    static constexpr DataLogSignal _sid_temperature[N_ESC] = {
        DataLogSignal::EscTemperature0, DataLogSignal::EscTemperature1,
        DataLogSignal::EscTemperature2, DataLogSignal::EscTemperature3};

    static constexpr DataLogSignal _sid_status[N_ESC] = {
        DataLogSignal::EscStatus0, DataLogSignal::EscStatus1,
        DataLogSignal::EscStatus2, DataLogSignal::EscStatus3};

    static constexpr TaskId _task_read_ids[N_ESC] = {
        TaskId::EscRead0, TaskId::EscRead1, TaskId::EscRead2, TaskId::EscRead3};

    uint8_t _load_balance_step = 0;
};

class TaskEscWrite : public Task
{
public:
    TaskEscWrite(uint32_t exec_period_ms, std::string name,
                 AfroEsc (&esc)[N_ESC], DataLogQueue& data_log_queue) :
            Task(exec_period_ms, name),
                 _esc(esc), _data_log_queue(data_log_queue) {}
protected:
    void _setup()
    {
        for (uint8_t i_esc = 0; i_esc < N_ESC; i_esc++)
        {
            _esc[i_esc].arm();
            _data_log_queue.push(uint8_t(_task_write_ids[i_esc]), DataLogSignal::TaskSetup);
        }
    }

    void _execute()
    {
        int16_t motor_cmd;
        SwitchLr switch_left;

        _data_log_queue.last_signal_data(&switch_left, DataLogSignal::RcSwitchLeft, SwitchLr::High);

        for (uint8_t i_esc = 0; i_esc < N_ESC; i_esc++)
        {
            switch(switch_left)
            {
                case SwitchLr::Low:
                    _data_log_queue.last_signal_data(&motor_cmd, _sid_motor_cmd[i_esc]);
                    _esc[i_esc].write(motor_cmd);
                    break;
                case SwitchLr::Middle:
                    _esc[i_esc].arm_fast();
                    break;
                case SwitchLr::High:
                    // Do nothing (disarm).
                    break;
                default:
                    // Do nothing (disarm).
                    break;
            }
            _data_log_queue.push(uint8_t(_task_write_ids[i_esc]), DataLogSignal::TaskExecute);
        }
    }

private:
    AfroEsc (&_esc)[N_ESC];
    DataLogQueue& _data_log_queue;

    static constexpr DataLogSignal _sid_motor_cmd[N_ESC] = {
        DataLogSignal::EscMotorCmd0, DataLogSignal::EscMotorCmd1,
        DataLogSignal::EscMotorCmd2, DataLogSignal::EscMotorCmd3};

    static constexpr TaskId _task_write_ids[N_ESC] = {
        TaskId::EscWrite0, TaskId::EscWrite1, TaskId::EscWrite2, TaskId::EscWrite3};
};

class TaskRcReceiver : public Task
{
public:
    TaskRcReceiver(uint32_t exec_period_ms, std::string name,
                   std::string serial_device, DataLogQueue& data_log_queue) :
              Task(exec_period_ms, name),
                   _serial_conn(serial_device), _rc(_serial_conn),
                   _data_log_queue(data_log_queue) {}
protected:
    void _execute()
    {
        _rc.update();

        _data_log_queue.push(_rc.get_gimbal_left_x(), DataLogSignal::RcGimbalLeftX);
        _data_log_queue.push(_rc.get_gimbal_left_y(), DataLogSignal::RcGimbalLeftY);

        _data_log_queue.push(_rc.get_gimbal_right_x(), DataLogSignal::RcGimbalRightX);
        _data_log_queue.push(_rc.get_gimbal_right_y(), DataLogSignal::RcGimbalRightY);

        _data_log_queue.push(uint8_t(_rc.get_switch_left()), DataLogSignal::RcSwitchLeft);
        _data_log_queue.push(uint8_t(_rc.get_switch_right()), DataLogSignal::RcSwitchRight);
        _data_log_queue.push(uint8_t(_rc.get_switch_middle()), DataLogSignal::RcSwitchMiddle);

        _data_log_queue.push(_rc.get_knob(), DataLogSignal::RcKnob);
        _data_log_queue.push(_rc.get_status(), DataLogSignal::RcStatus);
        _data_log_queue.push(uint8_t(TaskId::RcReceiver), DataLogSignal::TaskExecute);
    }
private:
    SerialConn _serial_conn;
    Tgyia6c _rc;
    DataLogQueue& _data_log_queue;
};

class TaskDataLogger : public Task
{
public:
    TaskDataLogger(uint32_t exec_period_ms, std::string name,
                   std::filesystem::path root_path, DataLogQueue& data_log_queue) :
              Task(exec_period_ms, name),
                   _data_logger(data_log_queue, root_path),
                   _data_log_queue(data_log_queue) {}
protected:
    void _setup()
    {
        _data_logger.start();
        _data_log_queue.push(uint8_t(TaskId::DataLogger), DataLogSignal::TaskSetup);
    }
    void _execute()
    {
        _data_logger.pack();
        _data_log_queue.push(uint8_t(TaskId::DataLogger), DataLogSignal::TaskExecute);
    }
    void _finish()
    {
        _data_logger.stop();
        _data_log_queue.push(uint8_t(TaskId::DataLogger), DataLogSignal::TaskFinish);
    }
private:
    DataLogger _data_logger;
    DataLogQueue& _data_log_queue;
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

void print_env_vars()
{
    logger.debug("DATA_LOG_ROOT: " + DATA_LOG_ROOT.string());
    logger.debug("LOGGER_LEVEL: " + LOGGER_LEVEL);
}

bool is_shutdown_ready(DataLogQueue& data_log_queue)
{
    SwitchLr switch_left = SwitchLr::Low;
    SwitchM switch_middle = SwitchM::High;

    data_log_queue.last_signal_data(&switch_left, DataLogSignal::RcSwitchLeft, SwitchLr::Low);
    data_log_queue.last_signal_data(&switch_middle, DataLogSignal::RcSwitchMiddle, SwitchM::High);

    return ((switch_left == SwitchLr::High) && (switch_middle == SwitchM::Low));
}

void wait_for_shutdown(DataLogQueue& data_log_queue)
{
    while (!is_shutdown_ready(data_log_queue))
    {
        logger.debug("Shutdown not ready. Keep running.");
        std::this_thread::sleep_for(std::chrono::milliseconds(MAIN_SLEEP_MS));
    }
    logger.info("Shutdown is ready. Will shutdown.");
}

int main()
{
    set_logger_level();
    print_env_vars();

    DataLogQueue data_log_queue;
    std::vector<std::unique_ptr<Task>> tasks;

    I2cConn esc_conn_0(I2C_DEVICE_ESC, I2C_ADDRESS_ESC_0), esc_conn_1(I2C_DEVICE_ESC, I2C_ADDRESS_ESC_1),
            esc_conn_2(I2C_DEVICE_ESC, I2C_ADDRESS_ESC_2), esc_conn_3(I2C_DEVICE_ESC, I2C_ADDRESS_ESC_3);
    AfroEsc esc[N_ESC] = {esc_conn_0, esc_conn_1, esc_conn_2, esc_conn_3};

    tasks.emplace_back(new TaskAccMag(TASK_ACC_MAG_EXEC_PERIOD_MS, "ImuAccMag",
                                      I2C_DEVICE_IMU, I2C_ADDRESS_ACC_MAG, data_log_queue));

    tasks.emplace_back(new TaskGyro(TASK_GYRO_EXEC_PERIOD_MS, "ImuGyro",
                                    I2C_DEVICE_IMU, I2C_ADDRESS_GYRO, data_log_queue));

    tasks.emplace_back(new TaskBarometer(TASK_BAROMETER_EXEC_PERIOD_MS, "ImuBarometer",
                                         I2C_DEVICE_IMU, I2C_ADDRESS_BAROMETER, data_log_queue));

    tasks.emplace_back(new TaskEscWrite(TASK_ESC_WRITE_EXEC_PERIOD_MS, "EscWrite",
                                        esc, data_log_queue));

    tasks.emplace_back(new TaskEscRead(TASK_ESC_READ_EXEC_PERIOD_MS, "EscRead",
                                       esc, data_log_queue));

    tasks.emplace_back(new TaskRcReceiver(TASK_RC_RECEIVER_EXEC_PERIOD_MS, "RcReceiver",
                                          SERIAL_DEVICE_RC_RECEIVER, data_log_queue));

    tasks.emplace_back(new TaskStateEstAndCtrl(TASK_STATE_EST_AND_CTRL_EXEC_PERIOD_MS, "StateEstAndCtrl",
                                               ATT_EST_INPUT_SAMPLE_RATE_S, data_log_queue));

    tasks.emplace_back(new TaskDataLogger(TASK_DATA_LOGGER_EXEC_PERIOD_MS, "DataLogger",
                                          DATA_LOG_ROOT, data_log_queue));

    for(auto const& task: tasks) { task->launch(); }
    wait_for_shutdown(data_log_queue);
    for(auto const& task: tasks) { task->teardown(); }

    return 0;
}

#endif /* !defined(UNIT_TEST) */
