// Utils.cpp
#include "Utils.h"
#include "DataTypes.h"
#include <string.h> // For memcpy, strlen, etc>

// Packing functions

void packInt8LE(int8_t value, uint8_t* buffer) {
    buffer[0] = static_cast<uint8_t>(value);
}

void packUInt8LE(uint8_t value, uint8_t* buffer) {
    buffer[0] = value;
}

void packInt16LE(int16_t value, uint8_t* buffer) {
    buffer[0] = static_cast<uint8_t>(value & 0xFF);
    buffer[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void packUInt16LE(uint16_t value, uint8_t* buffer) {
    buffer[0] = static_cast<uint8_t>(value & 0xFF);
    buffer[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void packInt24LE(int32_t value, uint8_t* buffer) {
    // Ensure value fits in 24 bits
    if (value < -8388608 || value > 8388607) {
        // Handle error
    }
    buffer[0] = static_cast<uint8_t>(value & 0xFF);
    buffer[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buffer[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
}

void packUInt24LE(uint32_t value, uint8_t* buffer) {
    // Ensure value fits in 24 bits
    if (value > 16777215) {
        // Handle error
    }
    buffer[0] = static_cast<uint8_t>(value & 0xFF);
    buffer[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buffer[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
}

void packInt32LE(int32_t value, uint8_t* buffer) {
    for (int i = 0; i < 4; ++i) {
        buffer[i] = static_cast<uint8_t>((value >> (i * 8)) & 0xFF);
    }
}

void packUInt32LE(uint32_t value, uint8_t* buffer) {
    for (int i = 0; i < 4; ++i) {
        buffer[i] = static_cast<uint8_t>((value >> (i * 8)) & 0xFF);
    }
}

void packInt48LE(int64_t value, uint8_t* buffer) {
    // Ensure value fits in 48 bits
    if (value < -140737488355328LL || value > 140737488355327LL) {
        // Handle error
    }
    for (int i = 0; i < 6; ++i) {
        buffer[i] = static_cast<uint8_t>((value >> (i * 8)) & 0xFF);
    }
}

void packUInt48LE(uint64_t value, uint8_t* buffer) {
    // Ensure value fits in 48 bits
    if (value > 281474976710655ULL) {
        // Handle error
    }
    for (int i = 0; i < 6; ++i) {
        buffer[i] = static_cast<uint8_t>((value >> (i * 8)) & 0xFF);
    }
}

void packInt64LE(int64_t value, uint8_t* buffer) {
    for (int i = 0; i < 8; ++i) {
        buffer[i] = static_cast<uint8_t>((value >> (i * 8)) & 0xFF);
    }
}

void packUInt64LE(uint64_t value, uint8_t* buffer) {
    for (int i = 0; i < 8; ++i) {
        buffer[i] = static_cast<uint8_t>((value >> (i * 8)) & 0xFF);
    }
}

void packString8(const char* str, uint8_t* buffer) {
    size_t len = strlen(str);
    if (len >= 8) {
        memcpy(buffer, str, 8);
    } else {
        memcpy(buffer, str, len);
        buffer[len] = '\0'; // Null terminator
        memset(buffer + len + 1, 0, 8 - len - 1); // Pad the rest with zeros
    }
}

void packU24VersionNumber(VersionNumber24 version, uint8_t* buffer) {
    buffer[0] = version.patch;
    buffer[1] = version.minor;
    buffer[2] = version.major;
}

void packU32VersionNumber(VersionNumber32 version, uint8_t* buffer) {
    buffer[0] = version.dev;
    buffer[1] = version.patch;
    buffer[2] = version.minor;
    buffer[3] = version.major;
}

void packCRC32(uint32_t value, uint8_t* buffer) {
    packUInt32LE(value, buffer);
}

void packU8Alias(uint8_t value, uint8_t* buffer) {
    packUInt8LE(value, buffer);
}

void packBuf10(const uint8_t* data, uint8_t* buffer) {
    memcpy(buffer, data, 10);
}

void packStringNullTerm(const char* str, uint8_t* buffer, size_t& length) {
    length = strlen(str) + 1; // Include null terminator
    memcpy(buffer, str, length);
}

void packGeneralData(const uint8_t* data, size_t dataLength, uint8_t* buffer) {
    memcpy(buffer, data, dataLength);
}

// Unpacking functions

int8_t unpackInt8LE(const uint8_t* buffer) {
    return static_cast<int8_t>(buffer[0]);
}

uint8_t unpackUInt8LE(const uint8_t* buffer) {
    return buffer[0];
}

int16_t unpackInt16LE(const uint8_t* buffer) {
    return static_cast<int16_t>(buffer[0] | (buffer[1] << 8));
}

uint16_t unpackUInt16LE(const uint8_t* buffer) {
    return static_cast<uint16_t>(buffer[0] | (buffer[1] << 8));
}

int32_t unpackInt24LE(const uint8_t* buffer) {
    int32_t value = buffer[0] | (buffer[1] << 8) | (buffer[2] << 16);
    // Sign extend if negative
    if (value & 0x800000) {
        value |= 0xFF000000;
    }
    return value;
}

uint32_t unpackUInt24LE(const uint8_t* buffer) {
    return buffer[0] | (buffer[1] << 8) | (buffer[2] << 16);
}

int32_t unpackInt32LE(const uint8_t* buffer) {
    return static_cast<int32_t>(
        buffer[0] |
        (buffer[1] << 8) |
        (buffer[2] << 16) |
        (buffer[3] << 24)
    );
}

uint32_t unpackUInt32LE(const uint8_t* buffer) {
    return buffer[0] |
           (buffer[1] << 8) |
           (buffer[2] << 16) |
           (buffer[3] << 24);
}

int64_t unpackInt48LE(const uint8_t* buffer) {
    int64_t value = 0;
    for (int i = 0; i < 6; ++i) {
        value |= static_cast<int64_t>(buffer[i]) << (i * 8);
    }
    // Sign extend if negative
    if (value & 0x800000000000) {
        value |= 0xFFFF000000000000LL;
    }
    return value;
}

uint64_t unpackUInt48LE(const uint8_t* buffer) {
    uint64_t value = 0;
    for (int i = 0; i < 6; ++i) {
        value |= static_cast<uint64_t>(buffer[i]) << (i * 8);
    }
    return value;
}

int64_t unpackInt64LE(const uint8_t* buffer) {
    int64_t value = 0;
    for (int i = 0; i < 8; ++i) {
        value |= static_cast<int64_t>(buffer[i]) << (i * 8);
    }
    return value;
}

uint64_t unpackUInt64LE(const uint8_t* buffer) {
    uint64_t value = 0;
    for (int i = 0; i < 8; ++i) {
        value |= static_cast<uint64_t>(buffer[i]) << (i * 8);
    }
    return value;
}

void unpackString8(const uint8_t* buffer, char* str) {
    memcpy(str, buffer, 8);
    str[8] = '\0'; // Ensure null termination
}

void unpackU24VersionNumber(const uint8_t* buffer, VersionNumber24* version) {
    version->patch = buffer[0];
    version->minor = buffer[1];
    version->major = buffer[2];
}

void unpackU32VersionNumber(const uint8_t* buffer, VersionNumber32* version) {
    version->dev = buffer[0];
    version->patch = buffer[1];
    version->minor = buffer[2];
    version->major = buffer[3];
}

uint32_t unpackCRC32(const uint8_t* buffer) {
    return unpackUInt32LE(buffer);
}

uint8_t unpackU8Alias(const uint8_t* buffer) {
    return unpackUInt8LE(buffer);
}

void unpackBuf10(const uint8_t* buffer, uint8_t* data) {
    memcpy(data, buffer, 10);
}

void unpackStringNullTerm(const uint8_t* buffer, char* str, size_t maxLength) {
    strncpy(str, reinterpret_cast<const char*>(buffer), maxLength - 1);
    str[maxLength - 1] = '\0'; // Ensure null termination
}

void unpackGeneralData(const uint8_t* buffer, size_t dataLength, uint8_t* data) {
    memcpy(data, buffer, dataLength);
}
