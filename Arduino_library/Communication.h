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
#define COMMUNICATION_SUCCESS 0

// Device ID bit manipulation constants
#define FIRST_BYTE_LSB_MASK 0x01  // Mask for checking if LSB is set
#define FIRST_BYTE_SHIFT 1         // Number of bits to shift for device ID interpretation

// Reserved aliases with special meaning (these are the actual values, not encoded)
#define ALL_ALIAS 255             // to address all devices on the bus at the same time
#define EXTENDED_ADDRESSING 254   // indicates that we will use extended addressing
#define RESPONSE_CHARACTER 253    // indicates that the response is coming from the device being addressed

// Helper functions for device ID encoding/decoding
inline uint8_t encodeFirstByte(uint8_t firstByte) {
    return (firstByte << FIRST_BYTE_SHIFT) | FIRST_BYTE_LSB_MASK;
}

inline uint8_t decodeFirstByte(uint8_t encodedFirstByte) {
    return encodedFirstByte >> FIRST_BYTE_SHIFT;
}

inline bool isValidFirstByteFormat(uint8_t encodedFirstByte) {
    return (encodedFirstByte & FIRST_BYTE_LSB_MASK) == FIRST_BYTE_LSB_MASK;
}

class Communication {
public:
    Communication(HardwareSerial& serialPort);

    void openSerialPort();
    void sendCommand(uint8_t alias, uint8_t commandID, const uint8_t* payload, uint16_t payloadSize);
    int8_t getResponse(uint8_t* buffer, uint16_t bufferSize, uint16_t& receivedSize);
    void flush(); // Add the flush declaration

protected:
    HardwareSerial& _serial; // Store the reference to the actual serial port
};
#endif // COMMUNICATION_H
