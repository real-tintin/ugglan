#ifndef DATA_LOG_TYPES_H
#define DATA_LOG_TYPES_H

#include <string>

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

typedef std::map<DataLogType, std::string> DataLogTypeMap;

inline const DataLogTypeMap DATA_LOG_TYPE_MAP = {
    {DataLogType::UINT8, "UINT8"},
    {DataLogType::UINT16, "UINT16"},
    {DataLogType::UINT32, "UINT32"},
    {DataLogType::SINT8, "SINT8"},
    {DataLogType::SINT16, "SINT16"},
    {DataLogType::SINT32, "SINT32"},
    {DataLogType::FLOAT, "FLOAT"},
    {DataLogType::DOUBLE, "DOUBLE"}
    };

#endif /* DATA_LOG_TYPES_H */