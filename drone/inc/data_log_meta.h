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
} DataLogGroupInfo;

typedef struct {
    std::string name;
    DataLogGroup group;
    DataLogType type;
} DataLogSignalInfo;

typedef std::map<DataLogGroup, DataLogGroupInfo> DataLogGroupMap;
typedef std::map<DataLogSignal, DataLogSignalInfo> DataLogSignalMap;

inline const DataLogGroupMap DATA_LOG_GROUP_MAP = {
    {DataLogGroup::Imu, {"IMU"}},
    {DataLogGroup::Esc, {"ESC"}}
    };

inline const DataLogSignalMap DATA_LOG_SIGNAL_MAP = {
    {DataLogSignal::ImuAccelerationX, {"AccelerationX", DataLogGroup::Imu, DataLogType::DOUBLE}},
    {DataLogSignal::ImuAccelerationY, {"AccelerationY", DataLogGroup::Imu, DataLogType::DOUBLE}},
    {DataLogSignal::ImuAccelerationZ, {"AccelerationZ", DataLogGroup::Imu, DataLogType::DOUBLE}}
    };

#endif /* DATA_LOG_META_H */
