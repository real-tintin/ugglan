#ifndef DATA_LOG_META_H
#define DATA_LOG_META_H

#include <cstdint>
#include <map>

enum class DataLogType {
    UINT8,
    UINT16,
    UINT32,
    SINT8,
    SINT16,
    SINT32,
    FLOAT,
    DOUBLE
};

enum class DataLogGroup {
    Imu,
    Esc
};

enum class DataLogSignal {
    ImuAccelerationX,
    ImuAccelerationY,
    ImuAccelerationZ
};

typedef struct {
    std::string name;
} GroupInfo;

typedef struct {
    std::string name;
    DataLogGroup group;
    DataLogType data_type;
} SignalInfo;

typedef std::map<DataLogGroup, GroupInfo> DataLogGroupInfo;
typedef std::map<DataLogSignal, SignalInfo> DataLogSignalInfo;

inline const DataLogGroupInfo DATA_LOG_GROUP_INFO = {
    {DataLogGroup::Imu, {"IMU"}},
    {DataLogGroup::Esc, {"ESC"}}
    };

inline const DataLogSignalInfo DATA_LOG_SIGNAL_INFO = {
    {DataLogSignal::ImuAccelerationX, {"AccelerationX", DataLogGroup::Imu, DataLogType::DOUBLE}},
    {DataLogSignal::ImuAccelerationY, {"AccelerationY", DataLogGroup::Imu, DataLogType::DOUBLE}},
    {DataLogSignal::ImuAccelerationZ, {"AccelerationZ", DataLogGroup::Imu, DataLogType::DOUBLE}}
    };

#endif /* DATA_LOG_META_H */
