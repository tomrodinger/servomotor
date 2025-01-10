#ifndef COMMUNICATION_H
#define COMMUNICATION_H

/*
  Communication.h
  A minimal class that a ServoMotor can use to "send commands."
  On a real Arduino, you'd do something with Serial/RS-485. 
  Here we'll just store data or print messages. 
*/

#include "ArduinoEmulator.h"

class Communication {
public:
    Communication(HardwareSerial& /*port*/) {}
    virtual ~Communication() {}

    // Minimal function signature
    virtual void sendCommand(uint8_t alias, uint8_t commandID,
                             const uint8_t* payload, uint16_t payloadSize);

    // Some function to get a response (dummy)
    virtual int8_t getResponse(uint8_t* /*buffer*/, uint16_t /*bufferSize*/,
                               uint16_t& /*receivedSize*/) {
        // no real response
        return 0;
    }

protected:
    // You can store data if needed
};

#endif // COMMUNICATION_H