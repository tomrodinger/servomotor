// Communication.cpp
#include "Communication.h"

//#define VERBOSE
#define TIMEOUT_MS 1000

Communication::Communication(HardwareSerial& serialPort) : _serial(serialPort) {}

void Communication::openSerialPort() {
    // Initialize the serial port if necessary
    // For example: Serial.begin(230400);
//    _serial.begin(230400); // Initialize serial communication at the required baud rate
}

void Communication::sendCommand(uint8_t alias, uint8_t commandID, const uint8_t* payload, uint16_t payloadSize) {
    _serial.write(alias);
    _serial.write(commandID);
    if (payloadSize < 255) {
        _serial.write((uint8_t)payloadSize);
    } else {
        _serial.write(0xFF);
        _serial.write((uint8_t)(payloadSize & 0xFF));
        _serial.write((uint8_t)((payloadSize >> 8) & 0xFF));
    }
    if (payload != nullptr && payloadSize > 0) {
        _serial.write(payload, payloadSize);
    }
}

int8_t Communication::getResponse(uint8_t* buffer, uint16_t bufferSize, uint16_t& receivedSize) {
    uint32_t startTime = millis();

    // Wait for response start character (254)
    while (!_serial.available()) {
        if (millis() - startTime > TIMEOUT_MS) { // Timeout after 1 second
            #ifdef VERBOSE
            Serial.println("Timed out"); // DEBUG
            #endif
            return COMMUNICATION_ERROR_TIMEOUT;
        }
    }
    uint8_t responseChar = _serial.read();
    #ifdef VERBOSE
    Serial.print("responseChar: "); // DEBUG
    Serial.println(responseChar); // DEBUG
    #endif
    if (responseChar != 254) {
        #ifdef VERBOSE
        Serial.println("Invalid response character (not 254)"); // DEBUG
        #endif
        return COMMUNICATION_ERROR_BAD_RESPONSE_CHAR;
    }

    // Wait for status byte
    while (!_serial.available()) {
        if (millis() - startTime > TIMEOUT_MS) {
            #ifdef VERBOSE
            Serial.println("Timed out waiting for status byte and payload size"); // DEBUG
            #endif
            return COMMUNICATION_ERROR_TIMEOUT;
        }
    }
    uint8_t status = _serial.read();
    #ifdef VERBOSE
    Serial.print("status: "); // DEBUG
    Serial.println(status); // DEBUG
    #endif
    if ((status != 0) && (status != 1)) {
        #ifdef VERBOSE
        Serial.println("Invalid second byte (not 0 or 1)"); // DEBUG
        #endif
        return COMMUNICATION_ERROR_BAD_STATUS_CHAR;
    }

    // Wait for payload size
    while (!_serial.available()) {
        if (millis() - startTime > TIMEOUT_MS) {
            #ifdef VERBOSE
            Serial.println("Timed out waiting for payload size"); // DEBUG
            #endif
            return COMMUNICATION_ERROR_TIMEOUT;
        }
    }    
    uint16_t payloadSize = _serial.read();
    #ifdef VERBOSE
    Serial.print("payloadSize: "); // DEBUG
    Serial.println(payloadSize); // DEBUG
    #endif

    if (payloadSize == 0xFF) {
        // Extended payload size
        while (_serial.available() < 2) {
            if (millis() - startTime > TIMEOUT_MS) {
                #ifdef VERBOSE
                Serial.println("Timed out waiting for extended payload size"); // DEBUG
                #endif
                return COMMUNICATION_ERROR_TIMEOUT;
            }
        }
        payloadSize = _serial.read();
        payloadSize |= (_serial.read() << 8);
        #ifdef VERBOSE
        Serial.print("extended payloadSize: "); // DEBUG
        Serial.println(payloadSize); // DEBUG
        #endif
    }
    if (payloadSize > bufferSize) {
        // Payload too large for buffer
        #ifdef VERBOSE
        Serial.println("Payload too large for buffer"); // DEBUG
        #endif
        return COMMUNICATION_ERROR_BUFFER_TOO_SMALL;
    }
    uint16_t index = 0;
    while (index < payloadSize) {
        if (_serial.available()) {
            buffer[index++] = _serial.read();
        } else if (millis() - startTime > TIMEOUT_MS) {
            // Timeout
            #ifdef VERBOSE
            Serial.println("Timed out waiting for payload data"); // DEBUG
            #endif
            return COMMUNICATION_ERROR_TIMEOUT;
        }
    }
    receivedSize = payloadSize;
    #ifdef VERBOSE
    Serial.print("receivedSize: "); // DEBUG
    Serial.println(receivedSize); // DEBUG
    Serial.print("buffer: "); // DEBUG
    for (int16_t i = 0; i < receivedSize; i++) {
        Serial.print(" "); // DEBUG
        Serial.print(buffer[i]); // DEBUG
    }
    Serial.println(); // DEBUG
    #endif
    return COMMUNICATION_SUCCESS;
}

void Communication::flush() {
    _serial.flush(); // Flush any outgoing data
    while (_serial.available()) {
        _serial.read(); // Clear any incoming data
    }
}