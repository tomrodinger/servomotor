// DataTypes.cpp
#include "DataTypes.h"

const DataType dataTypes[] = {
    {0, "i8", 1, -128, 127, true, "8-bit signed integer"},
    {1, "u8", 1, 0, 255, true, "8-bit unsigned integer"},
    {2, "i16", 2, -32768, 32767, true, "16-bit signed integer"},
    {3, "u16", 2, 0, 65535, true, "16-bit unsigned integer"},
    {4, "i24", 3, -8388608, 8388607, true, "24-bit signed integer"},
    {5, "u24", 3, 0, 16777215, true, "24-bit unsigned integer"},
    {6, "i32", 4, -2147483648LL, 2147483647LL, true, "32-bit signed integer"},
    {7, "u32", 4, 0, 4294967295ULL, true, "32-bit unsigned integer"},
    {8, "i48", 6, -140737488355328LL, 140737488355327LL, true, "48-bit signed integer"},
    {9, "u48", 6, 0, 281474976710655ULL, true, "48-bit unsigned integer"},
    {10, "i64", 8, INT64_MIN, INT64_MAX, true, "64-bit signed integer"},
    {11, "u64", 8, 0, static_cast<int64_t>(UINT64_MAX), true, "64-bit unsigned integer"},
    {12, "string8", 8, 0, 0, false, "8-byte string with null termination if shorter"},
    {13, "u24_version_number", 3, 0, 0, false, "3-byte version number (patch, minor, major)"},
    {14, "u32_version_number", 4, 0, 0, false, "4-byte version number (dev, patch, minor, major)"},
    {15, "u64_unique_id", 8, 0, 0, false, "Unique ID of the device (8 bytes)"},
    {17, "crc32", 4, 0, 0, false, "32-bit CRC"},
    {18, "u8_alias", 1, 0, 0, false, "ASCII character or number from 0 to 255"},
    {19, "buf10", 10, 0, 0, false, "10-byte buffer containing binary data"},
    {200, "list_2d", -1, 0, 0, false, "2D list in Python format"},
    {201, "string_null_term", -1, 0, 0, false, "Null-terminated string"},
    {202, "unknown_data", -1, 0, 0, false, "Unknown data type (work in progress)"},
    {203, "firmware_page", 2058, 0, 0, false, "Data for upgrading one page of flash memory"},
    {204, "general_data", -1, 0, 0, false, "General data of variable length"},
    {240, "success_response", -1, 0, 0, false, "Indicates successful command execution"}
    // Add other data type instances if any
};
