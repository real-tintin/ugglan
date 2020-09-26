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
    Esc
};

enum class DataLogSignal {
    ImuAccelerationX,
    ImuAccelerationY,
    ImuAccelerationZ,

    EscStatus0
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
    {DataLogGroup::Imu, {"IMU"}},
    {DataLogGroup::Esc, {"ESC"}}
    };

inline const DataLogSignalMap DATA_LOG_SIGNAL_MAP = {
    {DataLogSignal::ImuAccelerationX, {"AccelerationX", DataLogGroup::Imu, DataLogType::DOUBLE}},
    {DataLogSignal::ImuAccelerationY, {"AccelerationY", DataLogGroup::Imu, DataLogType::DOUBLE}},
    {DataLogSignal::ImuAccelerationZ, {"AccelerationZ", DataLogGroup::Imu, DataLogType::DOUBLE}},

    {DataLogSignal::EscStatus0, {"Status0", DataLogGroup::Esc, DataLogType::UINT8}}
    };
#endif

#endif /* DATA_LOG_SIGNALS_H */
