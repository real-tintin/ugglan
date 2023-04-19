#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include <afro_esc.hpp>
#include <attitude_estimation.hpp>
#include <config.hpp>
#include <data_log_queue.hpp>
#include <data_logger.hpp>
#include <drone_props.hpp>
#include <graceful_killer.hpp>
#include <i2c_conn.hpp>
#include <l3gd20h.hpp>
#include <lps25h.hpp>
#include <lsm303d.hpp>
#include <motor_control.hpp>
#include <pilot_control.hpp>
#include <serial_conn.hpp>
#include <streamer_server.hpp>
#include <task.hpp>
#include <tgyia6c.hpp>
#include <zmq_conn.hpp>

static const uint32_t TASK_ACC_MAG_EXEC_PERIOD_MS = 10;         // 100 Hz.
static const uint32_t TASK_GYRO_EXEC_PERIOD_MS = 10;            // 100 Hz.
static const uint32_t TASK_BAROMETER_EXEC_PERIOD_MS = 100;      // 10 Hz.
static const uint32_t TASK_STATE_EST_EXEC_PERIOD_MS = 10;       // 100 Hz.
static const uint32_t TASK_STATE_CTRL_EXEC_PERIOD_MS = 10;      // 100 Hz.
static const uint32_t TASK_ESC_READ_EXEC_PERIOD_MS = 1000;      // 1 Hz.
static const uint32_t TASK_ESC_WRITE_EXEC_PERIOD_MS = 10;       // 100 Hz.
static const uint32_t TASK_RC_RECEIVER_EXEC_PERIOD_MS = 10;     // 100 Hz.
static const uint32_t TASK_DATA_LOGGER_EXEC_PERIOD_MS = 100;    // 10 Hz.
static const uint32_t TASK_STREAMER_SERVER_EXEC_PERIOD_MS = 10; // 100 Hz.

static const uint8_t I2C_ADDRESS_ACC_MAG = LSM303D_I2C_ADDRESS;
static const uint8_t I2C_ADDRESS_GYRO = L3GD20H_I2C_ADDRESS;
static const uint8_t I2C_ADDRESS_BAROMETER = LPS25H_I2C_ADDRESS;

static const uint8_t I2C_ADDRESS_ESC_0 = 0x2a;
static const uint8_t I2C_ADDRESS_ESC_1 = 0x2b;
static const uint8_t I2C_ADDRESS_ESC_2 = 0x2c;
static const uint8_t I2C_ADDRESS_ESC_3 = 0x2d;

static const uint8_t N_ESC = droneprops::N_MOTORS;

static const uint32_t MAIN_SLEEP_MS = 1000;

static const double ATT_EST_INPUT_SAMPLE_RATE_S = 0.01; // 100 Hz (assumes IMU acc & gyro at 100 Hz)
static const double PILOT_CTRL_INPUT_SAMPLE_RATE_S = ATT_EST_INPUT_SAMPLE_RATE_S;

enum class TaskId
{
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
    StateEst,
    StateCtrl,
    DataLogger,
    StreamerServer
};

class TaskAccMag : public Task
{
public:
    TaskAccMag(uint32_t exec_period_ms,
               std::string name,
               std::string i2c_device,
               uint8_t i2c_address,
               DataLogQueue &data_log_queue)
        : Task(exec_period_ms, name), _i2c_conn(i2c_device, i2c_address), _acc_mag(_i2c_conn),
          _data_log_queue(data_log_queue)
    {
    }

protected:
    void _execute() override
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
    DataLogQueue &_data_log_queue;
};

class TaskGyro : public Task
{
public:
    TaskGyro(uint32_t exec_period_ms,
             std::string name,
             std::string i2c_device,
             uint8_t i2c_address,
             DataLogQueue &data_log_queue)
        : Task(exec_period_ms, name), _i2c_conn(i2c_device, i2c_address), _gyro(_i2c_conn),
          _data_log_queue(data_log_queue)
    {
    }

protected:
    void _execute() override
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
    DataLogQueue &_data_log_queue;
};

class TaskBarometer : public Task
{
public:
    TaskBarometer(uint32_t exec_period_ms,
                  std::string name,
                  std::string i2c_device,
                  uint8_t i2c_address,
                  DataLogQueue &data_log_queue)
        : Task(exec_period_ms, name), _i2c_conn(i2c_device, i2c_address), _barometer(_i2c_conn),
          _data_log_queue(data_log_queue)
    {
    }

protected:
    void _execute() override
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
    DataLogQueue &_data_log_queue;
};

class TaskStateEst : public Task
{
public:
    TaskStateEst(uint32_t exec_period_ms,
                 std::string name,
                 double input_sample_rate_s,
                 att_est::Config config,
                 DataLogQueue &data_log_queue)
        : Task(exec_period_ms, name), _estimator(input_sample_rate_s, config), _data_log_queue(data_log_queue)
    {
    }

protected:
    void _execute() override
    {
        _data_log_queue.last_signal_data(&_imu_uncompensated.acc_x, DataLogSignal::ImuAccelerationX);
        _data_log_queue.last_signal_data(&_imu_uncompensated.acc_y, DataLogSignal::ImuAccelerationY);
        _data_log_queue.last_signal_data(&_imu_uncompensated.acc_z, DataLogSignal::ImuAccelerationZ);

        _data_log_queue.last_signal_data(&_imu_uncompensated.ang_rate_x, DataLogSignal::ImuAngularRateX);
        _data_log_queue.last_signal_data(&_imu_uncompensated.ang_rate_y, DataLogSignal::ImuAngularRateY);
        _data_log_queue.last_signal_data(&_imu_uncompensated.ang_rate_z, DataLogSignal::ImuAngularRateZ);

        _data_log_queue.last_signal_data(&_imu_uncompensated.mag_field_x, DataLogSignal::ImuMagneticFieldX);
        _data_log_queue.last_signal_data(&_imu_uncompensated.mag_field_y, DataLogSignal::ImuMagneticFieldY);
        _data_log_queue.last_signal_data(&_imu_uncompensated.mag_field_z, DataLogSignal::ImuMagneticFieldZ);

        _estimator.update(_imu_uncompensated);

        _attitude = _estimator.get_attitude();
        _imu_compensated = _estimator.get_imu_compensated();

        _data_log_queue.push(_attitude.roll.angle, DataLogSignal::StateEstRoll);
        _data_log_queue.push(_attitude.pitch.angle, DataLogSignal::StateEstPitch);
        _data_log_queue.push(_attitude.yaw.angle, DataLogSignal::StateEstYaw);

        _data_log_queue.push(_attitude.roll.rate, DataLogSignal::StateEstRollRate);
        _data_log_queue.push(_attitude.pitch.rate, DataLogSignal::StateEstPitchRate);
        _data_log_queue.push(_attitude.yaw.rate, DataLogSignal::StateEstYawRate);

        _data_log_queue.push(_attitude.roll.acc, DataLogSignal::StateEstRollAcc);
        _data_log_queue.push(_attitude.pitch.acc, DataLogSignal::StateEstPitchAcc);
        _data_log_queue.push(_attitude.yaw.acc, DataLogSignal::StateEstYawAcc);

        _data_log_queue.push(_imu_compensated.acc_x, DataLogSignal::StateEstCompAccX);
        _data_log_queue.push(_imu_compensated.acc_y, DataLogSignal::StateEstCompAccY);
        _data_log_queue.push(_imu_compensated.acc_z, DataLogSignal::StateEstCompAccZ);

        _data_log_queue.push(_imu_compensated.ang_rate_x, DataLogSignal::StateEstCompAngRateX);
        _data_log_queue.push(_imu_compensated.ang_rate_y, DataLogSignal::StateEstCompAngRateY);
        _data_log_queue.push(_imu_compensated.ang_rate_z, DataLogSignal::StateEstCompAngRateZ);

        _data_log_queue.push(_imu_compensated.mag_field_x, DataLogSignal::StateEstCompMagFieldX);
        _data_log_queue.push(_imu_compensated.mag_field_y, DataLogSignal::StateEstCompMagFieldY);
        _data_log_queue.push(_imu_compensated.mag_field_z, DataLogSignal::StateEstCompMagFieldZ);

        _data_log_queue.push(_estimator.is_calibrated(), DataLogSignal::StateEstAttIsCalibrated);
        _data_log_queue.push(_estimator.is_standstill(), DataLogSignal::StateEstAttIsStandstill);

        _data_log_queue.push(uint8_t(TaskId::StateEst), DataLogSignal::TaskExecute);
    }

private:
    att_est::Imu _imu_uncompensated;
    att_est::Imu _imu_compensated;
    att_est::Attitude _attitude;
    att_est::Estimator _estimator;

    DataLogQueue &_data_log_queue;
};

class TaskStateCtrl : public Task
{
public:
    TaskStateCtrl(uint32_t exec_period_ms,
                  std::string name,
                  double input_sample_rate_s,
                  PilotCtrlConfig config,
                  DataLogQueue &data_log_queue)
        : Task(exec_period_ms, name), _pilot_ctrl(input_sample_rate_s, config), _data_log_queue(data_log_queue)
    {
    }

protected:
    void _execute() override
    {
        _exec_pilot_ctrl();
        _exec_motor_ctrl();

        _data_log_queue.push(uint8_t(TaskId::StateCtrl), DataLogSignal::TaskExecute);
    }

private:
    bool _att_est_is_calibrated;
    bool _state_ctrl_reset;
    att_est::Attitude _attitude;

    double _gimbal_left_x, _gimbal_left_y, _gimbal_right_x, _gimbal_right_y;

    PilotControl _pilot_ctrl;
    PilotCtrlRef _ctrl_ref;

    DataLogQueue &_data_log_queue;

    BodyControl _body_ctrl;
    MotorControl _motor_ctrl;

    static constexpr DataLogSignal _sid_motor_cmd[N_ESC] = {DataLogSignal::EscMotorCmd0,
                                                            DataLogSignal::EscMotorCmd1,
                                                            DataLogSignal::EscMotorCmd2,
                                                            DataLogSignal::EscMotorCmd3};

    void _exec_pilot_ctrl()
    {
        _data_log_queue.last_signal_data(&_att_est_is_calibrated, DataLogSignal::StateEstAttIsCalibrated, false);
        _data_log_queue.last_signal_data(&_state_ctrl_reset, DataLogSignal::StateCtrlReset, true);

        if (_state_ctrl_reset)
        {
            _pilot_ctrl.reset();
        }
        else if (_att_est_is_calibrated && !_state_ctrl_reset)
        {
            _data_log_queue.last_signal_data(&_attitude.roll.angle, DataLogSignal::StateEstRoll);
            _data_log_queue.last_signal_data(&_attitude.pitch.angle, DataLogSignal::StateEstPitch);
            _data_log_queue.last_signal_data(&_attitude.yaw.angle, DataLogSignal::StateEstYaw);

            _data_log_queue.last_signal_data(&_attitude.roll.rate, DataLogSignal::StateEstRollRate);
            _data_log_queue.last_signal_data(&_attitude.pitch.rate, DataLogSignal::StateEstPitchRate);
            _data_log_queue.last_signal_data(&_attitude.yaw.rate, DataLogSignal::StateEstYawRate);

            _data_log_queue.last_signal_data(&_attitude.roll.acc, DataLogSignal::StateEstRollAcc);
            _data_log_queue.last_signal_data(&_attitude.pitch.acc, DataLogSignal::StateEstPitchAcc);
            _data_log_queue.last_signal_data(&_attitude.yaw.acc, DataLogSignal::StateEstYawAcc);

            _data_log_queue.last_signal_data(&_gimbal_left_x, DataLogSignal::RcGimbalLeftX);
            _data_log_queue.last_signal_data(&_gimbal_left_y, DataLogSignal::RcGimbalLeftY);
            _data_log_queue.last_signal_data(&_gimbal_right_x, DataLogSignal::RcGimbalRightX);
            _data_log_queue.last_signal_data(&_gimbal_right_y, DataLogSignal::RcGimbalRightY);

            _ctrl_ref = tgyia6c_to_pilot_ctrl_ref(_gimbal_left_x, _gimbal_left_y, _gimbal_right_x, _gimbal_right_y);

            _pilot_ctrl.update(_attitude, _ctrl_ref);
        }

        _body_ctrl = _pilot_ctrl.get_ctrl();

        _data_log_queue.push(_ctrl_ref.roll, DataLogSignal::StateCtrlRollRef);
        _data_log_queue.push(_ctrl_ref.pitch, DataLogSignal::StateCtrlPitchRef);
        _data_log_queue.push(_ctrl_ref.yaw_rate, DataLogSignal::StateCtrlYawRateRef);
        _data_log_queue.push(_ctrl_ref.f_z, DataLogSignal::StateCtrlFzRef);

        _data_log_queue.push(_pilot_ctrl.get_state(PilotCtrlState::Phi0), DataLogSignal::StateCtrlPhi0);
        _data_log_queue.push(_pilot_ctrl.get_state(PilotCtrlState::Phi1), DataLogSignal::StateCtrlPhi1);
        _data_log_queue.push(_pilot_ctrl.get_state(PilotCtrlState::Phi2), DataLogSignal::StateCtrlPhi2);
        _data_log_queue.push(_pilot_ctrl.get_state(PilotCtrlState::Phi3), DataLogSignal::StateCtrlPhi3);

        _data_log_queue.push(_pilot_ctrl.get_state(PilotCtrlState::Theta0), DataLogSignal::StateCtrlTheta0);
        _data_log_queue.push(_pilot_ctrl.get_state(PilotCtrlState::Theta1), DataLogSignal::StateCtrlTheta1);
        _data_log_queue.push(_pilot_ctrl.get_state(PilotCtrlState::Theta2), DataLogSignal::StateCtrlTheta2);
        _data_log_queue.push(_pilot_ctrl.get_state(PilotCtrlState::Theta3), DataLogSignal::StateCtrlTheta3);

        _data_log_queue.push(_pilot_ctrl.get_state(PilotCtrlState::Psi0), DataLogSignal::StateCtrlPsi0);
        _data_log_queue.push(_pilot_ctrl.get_state(PilotCtrlState::Psi1), DataLogSignal::StateCtrlPsi1);
        _data_log_queue.push(_pilot_ctrl.get_state(PilotCtrlState::Psi2), DataLogSignal::StateCtrlPsi2);

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
    TaskEscRead(uint32_t exec_period_ms, std::string name, AfroEsc (&esc)[N_ESC], DataLogQueue &data_log_queue)
        : Task(exec_period_ms / N_ESC, name), _esc(esc), _data_log_queue(data_log_queue)
    {
    }

protected:
    void _execute() override
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
    DataLogQueue &_data_log_queue;

    static constexpr DataLogSignal _sid_is_alive[N_ESC] = {
        DataLogSignal::EscIsAlive0, DataLogSignal::EscIsAlive1, DataLogSignal::EscIsAlive2, DataLogSignal::EscIsAlive3};

    static constexpr DataLogSignal _sid_ang_rate[N_ESC] = {DataLogSignal::EscAngularRate0,
                                                           DataLogSignal::EscAngularRate1,
                                                           DataLogSignal::EscAngularRate2,
                                                           DataLogSignal::EscAngularRate3};

    static constexpr DataLogSignal _sid_voltage[N_ESC] = {
        DataLogSignal::EscVoltage0, DataLogSignal::EscVoltage1, DataLogSignal::EscVoltage2, DataLogSignal::EscVoltage3};

    static constexpr DataLogSignal _sid_current[N_ESC] = {
        DataLogSignal::EscCurrent0, DataLogSignal::EscCurrent1, DataLogSignal::EscCurrent2, DataLogSignal::EscCurrent3};

    static constexpr DataLogSignal _sid_temperature[N_ESC] = {DataLogSignal::EscTemperature0,
                                                              DataLogSignal::EscTemperature1,
                                                              DataLogSignal::EscTemperature2,
                                                              DataLogSignal::EscTemperature3};

    static constexpr DataLogSignal _sid_status[N_ESC] = {
        DataLogSignal::EscStatus0, DataLogSignal::EscStatus1, DataLogSignal::EscStatus2, DataLogSignal::EscStatus3};

    static constexpr TaskId _task_read_ids[N_ESC] = {
        TaskId::EscRead0, TaskId::EscRead1, TaskId::EscRead2, TaskId::EscRead3};

    uint8_t _load_balance_step = 0;
};

class TaskEscWrite : public Task
{
public:
    TaskEscWrite(uint32_t exec_period_ms, std::string name, AfroEsc (&esc)[N_ESC], DataLogQueue &data_log_queue)
        : Task(exec_period_ms, name), _esc(esc), _data_log_queue(data_log_queue)
    {
    }

protected:
    void _setup() override
    {
        _arm_escs();
        _push_task_state_to_data_log(DataLogSignal::TaskSetup);
    }

    void _execute() override
    {
        EscState user_reqeusted_esc_state = _get_user_requested_esc_state();
        EscState limited_esc_state = _limit_change_of_esc_state(user_reqeusted_esc_state, _old_esc_state);

        _update_escs(limited_esc_state);
        _old_esc_state = limited_esc_state;

        _push_task_state_to_data_log(DataLogSignal::TaskExecute);
    }

    void _finish() override
    {
        _halt_escs();
        _push_task_state_to_data_log(DataLogSignal::TaskFinish);
    }

private:
    enum class EscState
    {
        Run,
        Arm,
        Disarm
    };

    AfroEsc (&_esc)[N_ESC];
    DataLogQueue &_data_log_queue;

    static constexpr DataLogSignal _sid_motor_cmd[N_ESC] = {DataLogSignal::EscMotorCmd0,
                                                            DataLogSignal::EscMotorCmd1,
                                                            DataLogSignal::EscMotorCmd2,
                                                            DataLogSignal::EscMotorCmd3};

    static constexpr TaskId _task_write_ids[N_ESC] = {
        TaskId::EscWrite0, TaskId::EscWrite1, TaskId::EscWrite2, TaskId::EscWrite3};

    EscState _old_esc_state = EscState::Disarm;

    EscState _get_user_requested_esc_state()
    {
        SwitchLr switch_left;
        _data_log_queue.last_signal_data(&switch_left, DataLogSignal::RcSwitchLeft, SwitchLr::High);

        switch (switch_left)
        {
        case SwitchLr::Low:
            return EscState::Run;
        case SwitchLr::Middle:
            return EscState::Arm;
        case SwitchLr::High:
            return EscState::Disarm;
        default:
            return EscState::Disarm;
        }
    }

    EscState _limit_change_of_esc_state(EscState new_esc_state, EscState old_esc_state)
    {
        if (new_esc_state > old_esc_state)
        {
            return EscState(uint8_t(old_esc_state) + 1U);
        }
        else if (new_esc_state < old_esc_state)
        {
            return EscState(uint8_t(old_esc_state) - 1U);
        }
        else
        {
            return new_esc_state;
        }
    }

    void _update_escs(EscState esc_state)
    {
        switch (esc_state)
        {
        case EscState::Run:
            _data_log_queue.push(false, DataLogSignal::StateCtrlReset);
            _get_and_write_cmds();
            break;

        case EscState::Arm:
            _data_log_queue.push(true, DataLogSignal::StateCtrlReset);
            _arm_escs_fast();
            break;

        case EscState::Disarm:
            _data_log_queue.push(true, DataLogSignal::StateCtrlReset);
            break;

        default:
            _data_log_queue.push(true, DataLogSignal::StateCtrlReset);
            break;
        }
    }

    void _arm_escs()
    {
        for (uint8_t i_esc = 0; i_esc < N_ESC; i_esc++)
        {
            _esc[i_esc].arm();
        }
    }

    void _get_and_write_cmds()
    {
        uint16_t motor_cmd;

        for (uint8_t i_esc = 0; i_esc < N_ESC; i_esc++)
        {
            _data_log_queue.last_signal_data(&motor_cmd, _sid_motor_cmd[i_esc]);
            _esc[i_esc].write(motor_cmd);
        }
    }

    void _arm_escs_fast()
    {
        for (uint8_t i_esc = 0; i_esc < N_ESC; i_esc++)
        {
            _esc[i_esc].arm_fast();
        }
    }

    void _halt_escs()
    {
        for (uint8_t i_esc = 0; i_esc < N_ESC; i_esc++)
        {
            _esc[i_esc].halt();
        }
    }

    void _push_task_state_to_data_log(DataLogSignal task_state)
    {
        for (uint8_t i_esc = 0; i_esc < N_ESC; i_esc++)
        {
            _data_log_queue.push(uint8_t(_task_write_ids[i_esc]), task_state);
        }
    }
};

class TaskRcReceiver : public Task
{
public:
    TaskRcReceiver(uint32_t exec_period_ms, std::string name, std::string serial_device, DataLogQueue &data_log_queue)
        : Task(exec_period_ms, name), _serial_conn(serial_device), _rc(_serial_conn), _data_log_queue(data_log_queue)
    {
    }

protected:
    void _execute() override
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
    DataLogQueue &_data_log_queue;
};

class TaskDataLogger : public Task
{
public:
    TaskDataLogger(uint32_t exec_period_ms,
                   std::string name,
                   std::filesystem::path root_path,
                   DataLogQueue &data_log_queue)
        : Task(exec_period_ms, name), _data_logger(data_log_queue, root_path), _data_log_queue(data_log_queue)
    {
    }

protected:
    void _setup() override
    {
        _data_logger.start();
        _data_log_queue.push(uint8_t(TaskId::DataLogger), DataLogSignal::TaskSetup);
    }
    void _execute() override
    {
        _data_logger.pack();
        _data_log_queue.push(uint8_t(TaskId::DataLogger), DataLogSignal::TaskExecute);
    }
    void _finish() override
    {
        _data_logger.stop();
        _data_log_queue.push(uint8_t(TaskId::DataLogger), DataLogSignal::TaskFinish);
    }

private:
    DataLogger _data_logger;
    DataLogQueue &_data_log_queue;
};

class TaskStreamerServer : public Task
{
public:
    TaskStreamerServer(uint32_t exec_period_ms,
                       std::string name,
                       std::string zmq_address_request,
                       std::string zmq_address_stream,
                       DataLogQueue &data_log_queue)
        : Task(exec_period_ms, name), _request(zmq_address_request), _stream(zmq_address_stream),
          _server(_request, _stream, data_log_queue), _data_log_queue(data_log_queue)
    {
    }

protected:
    void _setup() override
    {
        _server.connect();
        _data_log_queue.push(uint8_t(TaskId::StreamerServer), DataLogSignal::TaskSetup);
    }
    void _execute() override
    {
        _server.execute();
        _data_log_queue.push(uint8_t(TaskId::StreamerServer), DataLogSignal::TaskExecute);
    }
    void _finish() override
    {
        _server.disconnect();
        _data_log_queue.push(uint8_t(TaskId::StreamerServer), DataLogSignal::TaskFinish);
    }

private:
    ZmqRep _request;
    ZmqPush _stream;

    streamer::Server _server;

    DataLogQueue &_data_log_queue;
};

bool user_requested_shutdown(DataLogQueue &data_log_queue)
{
    SwitchLr switch_left = SwitchLr::Low;
    SwitchM switch_middle = SwitchM::High;

    data_log_queue.last_signal_data(&switch_left, DataLogSignal::RcSwitchLeft, SwitchLr::Low);
    data_log_queue.last_signal_data(&switch_middle, DataLogSignal::RcSwitchMiddle, SwitchM::High);

    return ((switch_left == SwitchLr::High) && (switch_middle == SwitchM::Low));
}

void wait_for_shutdown(DataLogQueue &data_log_queue)
{
    GracefulKiller killer;

    while (!user_requested_shutdown(data_log_queue) && !killer.kill())
    {
        logger.debug("Shutdown not requested. Keep running.");
        std::this_thread::sleep_for(std::chrono::milliseconds(MAIN_SLEEP_MS));
    }
    logger.info("Shutdown requested. Will shutdown.");
}

int main()
{
    config::Config config;
    config.load();

    logger.set_level(config.get_logger_level());
    config.print_to_debug();

    DataLogQueue data_log_queue;
    std::vector<std::unique_ptr<Task>> tasks;

    I2cConn esc_conn_0(config.get_i2c_device_esc(), I2C_ADDRESS_ESC_0),
        esc_conn_1(config.get_i2c_device_esc(), I2C_ADDRESS_ESC_1),
        esc_conn_2(config.get_i2c_device_esc(), I2C_ADDRESS_ESC_2),
        esc_conn_3(config.get_i2c_device_esc(), I2C_ADDRESS_ESC_3);
    AfroEsc esc[N_ESC] = {esc_conn_0, esc_conn_1, esc_conn_2, esc_conn_3};

    tasks.emplace_back(new TaskAccMag(
        TASK_ACC_MAG_EXEC_PERIOD_MS, "ImuAccMag", config.get_i2c_device_imu(), I2C_ADDRESS_ACC_MAG, data_log_queue));

    tasks.emplace_back(new TaskGyro(
        TASK_GYRO_EXEC_PERIOD_MS, "ImuGyro", config.get_i2c_device_imu(), I2C_ADDRESS_GYRO, data_log_queue));

    tasks.emplace_back(new TaskBarometer(TASK_BAROMETER_EXEC_PERIOD_MS,
                                         "ImuBarometer",
                                         config.get_i2c_device_imu(),
                                         I2C_ADDRESS_BAROMETER,
                                         data_log_queue));

    tasks.emplace_back(new TaskEscWrite(TASK_ESC_WRITE_EXEC_PERIOD_MS, "EscWrite", esc, data_log_queue));

    tasks.emplace_back(new TaskEscRead(TASK_ESC_READ_EXEC_PERIOD_MS, "EscRead", esc, data_log_queue));

    tasks.emplace_back(
        new TaskRcReceiver(TASK_RC_RECEIVER_EXEC_PERIOD_MS, "RcReceiver", config.get_serial_device(), data_log_queue));

    tasks.emplace_back(new TaskStateEst(
        TASK_STATE_EST_EXEC_PERIOD_MS, "StateEst", ATT_EST_INPUT_SAMPLE_RATE_S, config.get_att_est(), data_log_queue));

    tasks.emplace_back(new TaskStateCtrl(TASK_STATE_CTRL_EXEC_PERIOD_MS,
                                         "StateCtrl",
                                         PILOT_CTRL_INPUT_SAMPLE_RATE_S,
                                         config.get_pilot_ctrl(),
                                         data_log_queue));

    tasks.emplace_back(
        new TaskDataLogger(TASK_DATA_LOGGER_EXEC_PERIOD_MS, "DataLogger", config.get_data_log_root(), data_log_queue));

    tasks.emplace_back(new TaskStreamerServer(TASK_STREAMER_SERVER_EXEC_PERIOD_MS,
                                              "StreamerServer",
                                              config.get_zmq_address_request(),
                                              config.get_zmq_address_stream(),
                                              data_log_queue));

    for (auto const &task : tasks)
    {
        task->launch();
    }
    wait_for_shutdown(data_log_queue);
    for (auto const &task : tasks)
    {
        task->teardown();
    }

    return 0;
}
