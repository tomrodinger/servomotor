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
