// DataTypes.h
#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <stdint.h>
#include <stdbool.h>

enum DataTypeID {
    DATA_TYPE_I8 = 0,
    DATA_TYPE_U8 = 1,
    DATA_TYPE_I16 = 2,
    DATA_TYPE_U16 = 3,
    DATA_TYPE_I24 = 4,
    DATA_TYPE_U24 = 5,
    DATA_TYPE_I32 = 6,
    DATA_TYPE_U32 = 7,
    DATA_TYPE_I48 = 8,
    DATA_TYPE_U48 = 9,
    DATA_TYPE_I64 = 10,
    DATA_TYPE_U64 = 11,
    DATA_TYPE_STRING8 = 12,
    DATA_TYPE_U24_VERSION_NUMBER = 13,
    DATA_TYPE_U32_VERSION_NUMBER = 14,
    DATA_TYPE_U64_UNIQUE_ID = 15,
    DATA_TYPE_CRC32 = 17,
    DATA_TYPE_U8_ALIAS = 18,
    DATA_TYPE_BUF10 = 19,
    DATA_TYPE_LIST_2D = 200,
    DATA_TYPE_STRING_NULL_TERM = 201,
    DATA_TYPE_UNKNOWN_DATA = 202,
    DATA_TYPE_FIRMWARE_PAGE = 203,
    DATA_TYPE_GENERAL_DATA = 204,
    DATA_TYPE_SUCCESS_RESPONSE = 240
    // Add other data types if any
};

// Define data type structures if needed
typedef struct {
    uint16_t id;          // Data type ID
    const char* name;     // Data type name
    int32_t size;         // Size in bytes, -1 if variable
    int64_t min_value;    // Minimum value for integer types
    int64_t max_value;    // Maximum value for integer types
    bool is_integer;      // True if the data type is an integer
    const char* description; // Description of the data type
} DataType;

typedef struct {
    uint8_t patch;
    uint8_t minor;
    uint8_t major;
} VersionNumber24;

typedef struct {
    uint8_t dev;
    uint8_t patch;
    uint8_t minor;
    uint8_t major;
} VersionNumber32;

#endif // DATA_TYPES_H
