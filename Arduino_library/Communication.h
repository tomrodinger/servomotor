#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#ifdef ARDUINO
#include <Arduino.h>
#else
// For desktop builds, ArduinoEmulator.h will be included by the main file
#include "ArduinoEmulator.h"
#endif

#define COMMUNICATION_ERROR_TIMEOUT  -1
#define COMMUNICATION_ERROR_DATA_WRONG_SIZE  -2
#define COMMUNICATION_ERROR_BAD_RESPONSE_CHAR  -3
#define COMMUNICATION_ERROR_BAD_STATUS_CHAR  -4
#define COMMUNICATION_ERROR_BUFFER_TOO_SMALL  -5
#define COMMUNICATION_ERROR_CRC32_MISMATCH  -6
#define COMMUNICATION_ERROR_BAD_FIRST_BYTE  -7
#define COMMUNICATION_ERROR_BAD_THIRD_BYTE  -8
#define COMMUNICATION_ERROR_PACKET_TOO_SMALL -9
#define COMMUNICATION_SUCCESS 0

// First byte encoding constants
#define FIRST_BYTE_LSB_MASK 0x01  // Mask for checking if LSB is set
#define FIRST_BYTE_SHIFT 1         // Number of bits to shift for size interpretation
#define DECODED_FIRST_BYTE_EXTENDED_SIZE 127  // Value indicating extended size format

// Reserved aliases with special meaning (these are the actual values, not encoded)
#define ALL_ALIAS 255             // to address all devices on the bus at the same time
#define EXTENDED_ADDRESSING 254   // indicates that we will use extended addressing
#define RESPONSE_CHARACTER_CRC32_ENABLED 253    // indicates that the response is coming from the device with CRC32 enabled
#define RESPONSE_CHARACTER_CRC32_DISABLED 252    // indicates that the response is coming from the device with CRC32 disabled

// CRC32 control commands
#define CRC32_ENABLE 1
#define CRC32_DISABLE 0
#define CRC32_ERROR_READ_ONLY 0
#define CRC32_ERROR_RESET 1

// Helper functions for size byte encoding/decoding
inline uint8_t encodeFirstByte(uint8_t firstByte) {
    return (firstByte << FIRST_BYTE_SHIFT) | FIRST_BYTE_LSB_MASK;
}

inline uint8_t decodeFirstByte(uint8_t encodedFirstByte) {
    return encodedFirstByte >> FIRST_BYTE_SHIFT;
}

inline bool isValidFirstByteFormat(uint8_t encodedFirstByte) {
    return (encodedFirstByte & FIRST_BYTE_LSB_MASK) == FIRST_BYTE_LSB_MASK;
}

// CRC32 calculation function
uint32_t calculate_crc32(const uint8_t* data, size_t length);

class Communication {
public:
    Communication(HardwareSerial& serialPort);

    void openSerialPort();
    
    // Standard addressing command
    void sendCommand(uint8_t alias, uint8_t commandID, const uint8_t* payload, uint16_t payloadSize);
    
    // Extended addressing command (using 64-bit Unique ID)
    void sendCommandByUniqueId(uint64_t uniqueId, uint8_t commandID, const uint8_t* payload, uint16_t payloadSize);
    
    int16_t getResponse(uint8_t* buffer, uint16_t bufferSize, uint16_t& receivedSize);
    void flush();
    
    // CRC32 control
    void enableCRC32();
    void disableCRC32();
    bool isCRC32Enabled() const;
protected:
    HardwareSerial& _serial; // Store the reference to the actual serial port
    bool _crc32Enabled; // Flag to track if CRC32 is enabled
    
private:
    // Core function that handles the common logic for both addressing modes
    void sendCommandCore(bool isExtended, uint64_t addressValue, uint8_t commandID,
                         const uint8_t* payload, uint16_t payloadSize);
    
    // Helper function to receive bytes with timeout checking
    int8_t receiveBytes(void* buffer, uint16_t bufferSize, int32_t numBytes, int32_t timeout_ms = 1000);
};
#endif // COMMUNICATION_H
