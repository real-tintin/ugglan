#ifndef DATA_LOG_SIGNALS_H
#define DATA_LOG_SIGNALS_H

#include <cstdint>
#include <map>
#include <data_log_types.h>

#if defined(UNIT_TEST)
enum class DataLogGroup {
    Imu,
    Esc
};

enum class DataLogSignal {
    ImuAccelerationX,
    EscStatus0
};
#else
enum class DataLogGroup {
    Imu,
    Esc,
    Rc,
    StateEst,
    StateCtrl,
    Task
};

enum class DataLogSignal {
    ImuAccelerationX,
    ImuAccelerationY,
    ImuAccelerationZ,

    ImuMagneticFieldX,
    ImuMagneticFieldY,
    ImuMagneticFieldZ,

    ImuAngularRateX,
    ImuAngularRateY,
    ImuAngularRateZ,

    ImuPressure,
    ImuTemperature,

    ImuAccMagStatus,
    ImuGyroStatus,
    ImuBarometerStatus,

    EscIsAlive0,
    EscIsAlive1,
    EscIsAlive2,
    EscIsAlive3,

    EscAngularRate0,
    EscAngularRate1,
    EscAngularRate2,
    EscAngularRate3,

    EscVoltage0,
    EscVoltage1,
    EscVoltage2,
    EscVoltage3,

    EscCurrent0,
    EscCurrent1,
    EscCurrent2,
    EscCurrent3,

    EscTemperature0,
    EscTemperature1,
    EscTemperature2,
    EscTemperature3,

    EscMotorCmd0,
    EscMotorCmd1,
    EscMotorCmd2,
    EscMotorCmd3,

    EscStatus0,
    EscStatus1,
    EscStatus2,
    EscStatus3,

    RcGimbalLeftX,
    RcGimbalLeftY,

    RcGimbalRightX,
    RcGimbalRightY,

    RcSwitchLeft,
    RcSwitchRight,
    RcSwitchMiddle,

    RcKnob,
    RcStatus,

    StateEstRoll,
    StateEstPitch,
    StateEstYaw,

    StateEstRollRate,
    StateEstPitchRate,
    StateEstYawRate,

    StateEstRollAcc,
    StateEstPitchAcc,
    StateEstYawAcc,

    StateEstAttIsCalib,
    StateEstAttIsStandstill,

    StateCtrlRollRef,
    StateCtrlPitchRef,
    StateCtrlYawRateRef,
    StateCtrlFzRef,

    StateCtrlPhi0,
    StateCtrlPhi1,
    StateCtrlPhi2,
    StateCtrlPhi3,

    StateCtrlTheta0,
    StateCtrlTheta1,
    StateCtrlTheta2,
    StateCtrlTheta3,

    StateCtrlPsi0,
    StateCtrlPsi1,
    StateCtrlPsi2,

    StateCtrlMx,
    StateCtrlMy,
    StateCtrlMz,
    StateCtrlFz,

    TaskSetup,
    TaskExecute,
    TaskFinish
};
#endif

struct DataLogGroupInfo{
    std::string name;
};

struct DataLogSignalInfo{
    std::string name;
    DataLogGroup group;
    DataLogType type;
};

typedef std::map<DataLogGroup, DataLogGroupInfo> DataLogGroupMap;
typedef std::map<DataLogSignal, DataLogSignalInfo> DataLogSignalMap;

#if defined(UNIT_TEST)
inline const DataLogGroupMap DATA_LOG_GROUP_MAP = {
    {DataLogGroup::Imu, {"IMU"}},
    {DataLogGroup::Esc, {"ESC"}}
    };

inline const DataLogSignalMap DATA_LOG_SIGNAL_MAP = {
    {DataLogSignal::ImuAccelerationX, {"AccelerationX", DataLogGroup::Imu, DataLogType::DOUBLE}},
    {DataLogSignal::EscStatus0, {"Status0", DataLogGroup::Esc, DataLogType::UINT8}}
    };
#else
inline const DataLogGroupMap DATA_LOG_GROUP_MAP = {
    {DataLogGroup::Imu, {"Imu"}},
    {DataLogGroup::Esc, {"Esc"}},
    {DataLogGroup::Rc, {"Rc"}},
    {DataLogGroup::StateEst, {"StateEst"}},
    {DataLogGroup::StateCtrl, {"StateCtrl"}},
    {DataLogGroup::Task, {"Task"}}
    };

inline const DataLogSignalMap DATA_LOG_SIGNAL_MAP = {
    {DataLogSignal::ImuAccelerationX, {"AccelerationX", DataLogGroup::Imu, DataLogType::DOUBLE}},
    {DataLogSignal::ImuAccelerationY, {"AccelerationY", DataLogGroup::Imu, DataLogType::DOUBLE}},
    {DataLogSignal::ImuAccelerationZ, {"AccelerationZ", DataLogGroup::Imu, DataLogType::DOUBLE}},

    {DataLogSignal::ImuMagneticFieldX, {"MagneticFieldX", DataLogGroup::Imu, DataLogType::DOUBLE}},
    {DataLogSignal::ImuMagneticFieldY, {"MagneticFieldY", DataLogGroup::Imu, DataLogType::DOUBLE}},
    {DataLogSignal::ImuMagneticFieldZ, {"MagneticFieldZ", DataLogGroup::Imu, DataLogType::DOUBLE}},

    {DataLogSignal::ImuAngularRateX, {"AngularRateX", DataLogGroup::Imu, DataLogType::DOUBLE}},
    {DataLogSignal::ImuAngularRateY, {"AngularRateY", DataLogGroup::Imu, DataLogType::DOUBLE}},
    {DataLogSignal::ImuAngularRateZ, {"AngularRateZ", DataLogGroup::Imu, DataLogType::DOUBLE}},

    {DataLogSignal::ImuPressure, {"Pressure", DataLogGroup::Imu, DataLogType::DOUBLE}},
    {DataLogSignal::ImuTemperature, {"Temperature", DataLogGroup::Imu, DataLogType::DOUBLE}},

    {DataLogSignal::ImuAccMagStatus, {"AccMagStatus", DataLogGroup::Imu, DataLogType::UINT8}},
    {DataLogSignal::ImuGyroStatus, {"GyroStatus", DataLogGroup::Imu, DataLogType::UINT8}},
    {DataLogSignal::ImuBarometerStatus, {"BarometerStatus", DataLogGroup::Imu, DataLogType::UINT8}},

    {DataLogSignal::EscIsAlive0, {"IsAlive0", DataLogGroup::Esc, DataLogType::BOOL}},
    {DataLogSignal::EscIsAlive1, {"IsAlive1", DataLogGroup::Esc, DataLogType::BOOL}},
    {DataLogSignal::EscIsAlive2, {"IsAlive2", DataLogGroup::Esc, DataLogType::BOOL}},
    {DataLogSignal::EscIsAlive3, {"IsAlive3", DataLogGroup::Esc, DataLogType::BOOL}},

    {DataLogSignal::EscAngularRate0, {"AngularRate0", DataLogGroup::Esc, DataLogType::DOUBLE}},
    {DataLogSignal::EscAngularRate1, {"AngularRate1", DataLogGroup::Esc, DataLogType::DOUBLE}},
    {DataLogSignal::EscAngularRate2, {"AngularRate2", DataLogGroup::Esc, DataLogType::DOUBLE}},
    {DataLogSignal::EscAngularRate3, {"AngularRate3", DataLogGroup::Esc, DataLogType::DOUBLE}},

    {DataLogSignal::EscVoltage0, {"Voltage0", DataLogGroup::Esc, DataLogType::DOUBLE}},
    {DataLogSignal::EscVoltage1, {"Voltage1", DataLogGroup::Esc, DataLogType::DOUBLE}},
    {DataLogSignal::EscVoltage2, {"Voltage2", DataLogGroup::Esc, DataLogType::DOUBLE}},
    {DataLogSignal::EscVoltage3, {"Voltage3", DataLogGroup::Esc, DataLogType::DOUBLE}},

    {DataLogSignal::EscCurrent0, {"Current0", DataLogGroup::Esc, DataLogType::DOUBLE}},
    {DataLogSignal::EscCurrent1, {"Current1", DataLogGroup::Esc, DataLogType::DOUBLE}},
    {DataLogSignal::EscCurrent2, {"Current2", DataLogGroup::Esc, DataLogType::DOUBLE}},
    {DataLogSignal::EscCurrent3, {"Current3", DataLogGroup::Esc, DataLogType::DOUBLE}},

    {DataLogSignal::EscTemperature0, {"Temperature0", DataLogGroup::Esc, DataLogType::DOUBLE}},
    {DataLogSignal::EscTemperature1, {"Temperature1", DataLogGroup::Esc, DataLogType::DOUBLE}},
    {DataLogSignal::EscTemperature2, {"Temperature2", DataLogGroup::Esc, DataLogType::DOUBLE}},
    {DataLogSignal::EscTemperature3, {"Temperature3", DataLogGroup::Esc, DataLogType::DOUBLE}},

    {DataLogSignal::EscMotorCmd0, {"MotorCmd0", DataLogGroup::Esc, DataLogType::SINT16}},
    {DataLogSignal::EscMotorCmd1, {"MotorCmd1", DataLogGroup::Esc, DataLogType::SINT16}},
    {DataLogSignal::EscMotorCmd2, {"MotorCmd2", DataLogGroup::Esc, DataLogType::SINT16}},
    {DataLogSignal::EscMotorCmd3, {"MotorCmd3", DataLogGroup::Esc, DataLogType::SINT16}},

    {DataLogSignal::EscStatus0, {"Status0", DataLogGroup::Esc, DataLogType::UINT8}},
    {DataLogSignal::EscStatus1, {"Status1", DataLogGroup::Esc, DataLogType::UINT8}},
    {DataLogSignal::EscStatus2, {"Status2", DataLogGroup::Esc, DataLogType::UINT8}},
    {DataLogSignal::EscStatus3, {"Status3", DataLogGroup::Esc, DataLogType::UINT8}},

    {DataLogSignal::RcGimbalLeftX, {"GimbalLeftX", DataLogGroup::Rc, DataLogType::DOUBLE}},
    {DataLogSignal::RcGimbalLeftY, {"GimbalLeftY", DataLogGroup::Rc, DataLogType::DOUBLE}},

    {DataLogSignal::RcGimbalRightX, {"GimbalRightX", DataLogGroup::Rc, DataLogType::DOUBLE}},
    {DataLogSignal::RcGimbalRightY, {"GimbalRightY", DataLogGroup::Rc, DataLogType::DOUBLE}},

    {DataLogSignal::RcSwitchLeft, {"SwitchLeft", DataLogGroup::Rc, DataLogType::UINT8}},
    {DataLogSignal::RcSwitchRight, {"SwitchRight", DataLogGroup::Rc, DataLogType::UINT8}},
    {DataLogSignal::RcSwitchMiddle, {"SwitchMiddle", DataLogGroup::Rc, DataLogType::UINT8}},

    {DataLogSignal::RcKnob, {"Knob", DataLogGroup::Rc, DataLogType::DOUBLE}},
    {DataLogSignal::RcStatus, {"Status", DataLogGroup::Rc, DataLogType::UINT8}},

    {DataLogSignal::StateEstRoll, {"Roll", DataLogGroup::StateEst, DataLogType::DOUBLE}},
    {DataLogSignal::StateEstPitch, {"Pitch", DataLogGroup::StateEst, DataLogType::DOUBLE}},
    {DataLogSignal::StateEstYaw, {"Yaw", DataLogGroup::StateEst, DataLogType::DOUBLE}},

    {DataLogSignal::StateEstRollRate, {"RollRate", DataLogGroup::StateEst, DataLogType::DOUBLE}},
    {DataLogSignal::StateEstPitchRate, {"PitchRate", DataLogGroup::StateEst, DataLogType::DOUBLE}},
    {DataLogSignal::StateEstYawRate, {"YawRate", DataLogGroup::StateEst, DataLogType::DOUBLE}},

    {DataLogSignal::StateEstRollAcc, {"RollAcc", DataLogGroup::StateEst, DataLogType::DOUBLE}},
    {DataLogSignal::StateEstPitchAcc, {"PitchAcc", DataLogGroup::StateEst, DataLogType::DOUBLE}},
    {DataLogSignal::StateEstYawAcc, {"YawAcc", DataLogGroup::StateEst, DataLogType::DOUBLE}},

    {DataLogSignal::StateEstAttIsCalib, {"AttIsCalib", DataLogGroup::StateEst, DataLogType::BOOL}},
    {DataLogSignal::StateEstAttIsStandstill, {"AttIsStandstill", DataLogGroup::StateEst, DataLogType::BOOL}},

    {DataLogSignal::StateCtrlRollRef, {"RollRef", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},
    {DataLogSignal::StateCtrlPitchRef, {"PitchRef", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},
    {DataLogSignal::StateCtrlYawRateRef, {"YawRateRef", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},
    {DataLogSignal::StateCtrlFzRef, {"FzRef", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},

    {DataLogSignal::StateCtrlPhi0, {"Phi0", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},
    {DataLogSignal::StateCtrlPhi1, {"Phi1", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},
    {DataLogSignal::StateCtrlPhi2, {"Phi2", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},
    {DataLogSignal::StateCtrlPhi3, {"Phi3", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},

    {DataLogSignal::StateCtrlTheta0, {"Theta0", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},
    {DataLogSignal::StateCtrlTheta1, {"Theta1", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},
    {DataLogSignal::StateCtrlTheta2, {"Theta2", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},
    {DataLogSignal::StateCtrlTheta3, {"Theta3", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},

    {DataLogSignal::StateCtrlPsi0, {"Psi0", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},
    {DataLogSignal::StateCtrlPsi1, {"Psi1", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},
    {DataLogSignal::StateCtrlPsi2, {"Psi2", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},

    {DataLogSignal::StateCtrlMx, {"Mx", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},
    {DataLogSignal::StateCtrlMy, {"My", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},
    {DataLogSignal::StateCtrlMz, {"Mz", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},
    {DataLogSignal::StateCtrlFz, {"Fz", DataLogGroup::StateCtrl, DataLogType::DOUBLE}},

    {DataLogSignal::TaskSetup, {"Setup", DataLogGroup::Task, DataLogType::UINT8}},
    {DataLogSignal::TaskExecute, {"Execute", DataLogGroup::Task, DataLogType::UINT8}},
    {DataLogSignal::TaskFinish, {"Finish", DataLogGroup::Task, DataLogType::UINT8}}
    };
#endif

#endif /* DATA_LOG_SIGNALS_H */
