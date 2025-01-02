// Communication.h
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>

#define COMMUNICATION_SUCCESS                 0
#define COMMUNICATION_ERROR_TIMEOUT           -1
#define COMMUNICATION_ERROR_BAD_RESPONSE_CHAR -2
#define COMMUNICATION_ERROR_BAD_STATUS_CHAR   -3
#define COMMUNICATION_ERROR_BUFFER_TOO_SMALL  -4
#define COMMUNICATION_ERROR_DATA_WRONG_SIZE   -5

class Communication {
public:
    Communication(HardwareSerial& serialPort);
    void openSerialPort();
    void sendCommand(uint8_t alias, uint8_t commandID, const uint8_t* payload, uint16_t payloadSize);
    int8_t getResponse(uint8_t* buffer, uint16_t bufferSize, uint16_t& receivedSize);
    void flush(); // If you have implemented flush

private:
    HardwareSerial& _serial;
};

#endif // COMMUNICATION_H
