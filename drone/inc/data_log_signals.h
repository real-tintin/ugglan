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
    Rc
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
    RcStatus
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
    {DataLogGroup::Rc, {"Rc"}}
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
    {DataLogSignal::RcStatus, {"Status", DataLogGroup::Rc, DataLogType::UINT8}}
    };
#endif

#endif /* DATA_LOG_SIGNALS_H */
