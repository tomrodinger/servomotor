// Communication.cpp
#include "Communication.h"

#define VERBOSE
#define TIMEOUT_MS 1000  // Define timeout duration (1 second)

#ifndef ARDUINO
// Define Arduino constants for desktop builds
#define HEX 16
#endif

#define CRC32_POLYNOMIAL 0xEDB88320

static uint32_t crc32_value;

void crc32_init(void)
{
    crc32_value = 0xFFFFFFFF;
}

uint32_t calculate_crc32_buffer_without_reinit(const uint8_t* data, size_t length)
{
    uint32_t i, j;
    for (i = 0; i < length; i++) {
        crc32_value ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc32_value & 1)
                crc32_value = (crc32_value >> 1) ^ CRC32_POLYNOMIAL;
            else
                crc32_value = crc32_value >> 1;
        }
    }
    return ~crc32_value;
}

uint32_t calculate_crc32(const uint8_t* data, size_t length)
{
    crc32_init();
    return calculate_crc32_buffer_without_reinit(data, length);
}

uint32_t get_crc32(void)
{
    return ~crc32_value;
}

Communication::Communication(HardwareSerial& serialPort) : _serial(serialPort), _crc32Enabled(true) {
    // No initialization needed for direct CRC32 calculation
}

void Communication::openSerialPort() {
    // Serial port is already initialized by ArduinoEmulator
}

void Communication::writePacketSize(uint16_t size) {
    // The size parameter is the number of bytes in the packet EXCLUDING the size byte itself
    // We need to add 1 to include the size byte itself in the total packet size
    uint16_t totalSize = size + 1; // Add 1 for the size byte itself
    
    // If total size can fit in 7 bits (0-127), use a single byte
    if (totalSize <= 127) {
        // Encode size: shift left by 1 and set LSB to 1
        uint8_t encodedSize = (totalSize << FIRST_BYTE_SHIFT) | FIRST_BYTE_LSB_MASK;
        _serial.write(encodedSize);
        
        #ifdef VERBOSE
        Serial.print("Encoded size byte: 0x");
        if (encodedSize < 0x10) Serial.print("0");
        Serial.print(encodedSize, HEX);
        Serial.print(" (represents ");
        Serial.print(totalSize);
        Serial.println(" bytes including size byte)");
        #endif
    }
    // Otherwise, use extended size format (3 bytes)
    else {
        // First byte indicates extended size format
        uint8_t encodedFirstByte = (DECODED_FIRST_BYTE_EXTENDED_SIZE << FIRST_BYTE_SHIFT) | FIRST_BYTE_LSB_MASK;
        
        // Size bytes (little endian)
        uint8_t sizeLow = (totalSize & 0xFF);
        uint8_t sizeHigh = ((totalSize >> 8) & 0xFF);
        
        _serial.write(encodedFirstByte);
        _serial.write(sizeLow);
        _serial.write(sizeHigh);
        
        #ifdef VERBOSE
        Serial.print("Extended size bytes: 0x");
        if (encodedFirstByte < 0x10) Serial.print("0");
        Serial.print(encodedFirstByte, HEX);
        Serial.print(" 0x");
        if (sizeLow < 0x10) Serial.print("0");
        Serial.print(sizeLow, HEX);
        Serial.print(" 0x");
        if (sizeHigh < 0x10) Serial.print("0");
        Serial.print(sizeHigh, HEX);
        Serial.print(" (represents ");
        Serial.print(totalSize);
        Serial.println(" bytes including size bytes)");
        #endif
    }
}

void Communication::writePacket(const uint8_t* data, uint16_t size) {
    if (data != nullptr && size > 0) {
        _serial.write(data, size);
    }
}

void Communication::finalizePacket(uint8_t* buffer, uint16_t& size) {
    // Add CRC32 if enabled
    if (_crc32Enabled) {
        uint32_t crc = calculate_crc32(buffer, size);
        
        // Append CRC32 to the buffer (little endian)
        buffer[size++] = (uint8_t)(crc & 0xFF);
        buffer[size++] = (uint8_t)((crc >> 8) & 0xFF);
        buffer[size++] = (uint8_t)((crc >> 16) & 0xFF);
        buffer[size++] = (uint8_t)((crc >> 24) & 0xFF);
    }
}

void Communication::sendCommand(uint8_t alias, uint8_t commandID, const uint8_t* payload, uint16_t payloadSize) {
    uint8_t sizeByte;
    bool isExtendedSize = false;
    uint8_t sizeLow = 0;
    uint8_t sizeHigh = 0;

    // Calculate total packet size (including the size byte itself)
    uint16_t totalPacketSize = sizeof(sizeByte) + sizeof(alias) + sizeof(commandID) + payloadSize;
    if (_crc32Enabled) {
        totalPacketSize += sizeof(uint32_t); // Add 4 bytes for CRC32
    }
    
    // Determine the size byte value
    if (totalPacketSize <= DECODED_FIRST_BYTE_EXTENDED_SIZE) {
        sizeByte = encodeFirstByte(totalPacketSize);
        _serial.write(sizeByte);
    } else {
        isExtendedSize = true;
        totalPacketSize += sizeof(uint16_t); // Add 2 bytes for the extended size
        sizeByte = encodeFirstByte(DECODED_FIRST_BYTE_EXTENDED_SIZE);
        sizeLow = (uint8_t)(totalPacketSize & 0xFF);
        sizeHigh = (uint8_t)((totalPacketSize >> 8) & 0xFF);
        _serial.write(sizeByte);
        _serial.write(sizeLow);
        _serial.write(sizeHigh);
    }
    
    // Write alias byte
    _serial.write(alias);
    
    // Write command byte
    _serial.write(commandID);
    
    // Write payload
    if (payload != nullptr && payloadSize > 0) {
        _serial.write(payload, payloadSize);

        Serial.print("Payload bytes: ");
        for (uint16_t i = 0; i < payloadSize; i++) {
            Serial.print("0x");
            if (payload[i] < 0x10) Serial.print("0");
            Serial.print(payload[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
    else {
        Serial.println("No payload");
    }

    // Calculate and write CRC32 if enabled
    if (_crc32Enabled) {
        crc32_init();
        calculate_crc32_buffer_without_reinit(&sizeByte, sizeof(sizeByte));
        if (isExtendedSize) {
            calculate_crc32_buffer_without_reinit(&sizeLow, sizeof(sizeLow));
            calculate_crc32_buffer_without_reinit(&sizeHigh, sizeof(sizeHigh));
        }
        calculate_crc32_buffer_without_reinit(&alias, sizeof(alias));
        calculate_crc32_buffer_without_reinit(&commandID, sizeof(commandID));
        if (payload != nullptr && payloadSize > 0) {
            calculate_crc32_buffer_without_reinit(payload, payloadSize);
        }
        
        // Calculate CRC32 and transmit it in the case when CRC32 is enabled
        uint32_t crc = get_crc32();
        _serial.write((uint8_t*)(&crc), sizeof(crc));

        #ifdef VERBOSE
        Serial.print("calculated CRC32 bytes sent: 0x");
        Serial.print(crc, HEX);
        Serial.println();
        #endif
    }
}

void Communication::sendCommandExtended(uint64_t uniqueId, uint8_t commandID, const uint8_t* payload, uint16_t payloadSize) {
    // Calculate total packet size (excluding the size byte itself)
    uint16_t totalSize = 1; // Address byte (EXTENDED_ADDRESSING)
    totalSize += 8;         // 64-bit Unique ID
    totalSize += 1;         // Command byte
    totalSize += payloadSize;
    
    if (_crc32Enabled) {
        totalSize += 4;     // CRC32 (4 bytes)
    }
    
    // Note: The size byte itself will be added in writePacketSize
    uint16_t totalSizeWithSizeByte = totalSize + 1; // Include size byte
    
    // Write packet size
    writePacketSize(totalSizeWithSizeByte);
    
    // Write extended addressing indicator
    _serial.write(EXTENDED_ADDRESSING);
    
    // Write 64-bit Unique ID (little endian)
    for (int i = 0; i < 8; i++) {
        _serial.write((uint8_t)((uniqueId >> (i * 8)) & 0xFF));
    }
    
    // Write command byte
    _serial.write(commandID);
    
    // Write payload
    if (payload != nullptr && payloadSize > 0) {
        _serial.write(payload, payloadSize);
    }
    
    // Calculate and write CRC32 if enabled
    if (_crc32Enabled) {
        // Create a buffer with all the data for CRC calculation
        uint16_t bufferSize = totalSize + 1 - 4; // Total size + size bytes - CRC32
        uint8_t* buffer = new uint8_t[bufferSize];
        uint16_t index = 0;
        
        // Add size byte(s)
        uint8_t sizeByte;
        uint8_t sizeLow = 0, sizeHigh = 0;
        bool isExtendedSize = false;
        
        // Determine the size byte value
        if (totalSizeWithSizeByte <= 127) {
            sizeByte = encodeFirstByte(totalSizeWithSizeByte);
        } else {
            isExtendedSize = true;
            sizeByte = encodeFirstByte(DECODED_FIRST_BYTE_EXTENDED_SIZE);
            sizeLow = (uint8_t)(totalSizeWithSizeByte & 0xFF);
            sizeHigh = (uint8_t)((totalSizeWithSizeByte >> 8) & 0xFF);
        }
        
        buffer[index++] = sizeByte;
        if (isExtendedSize) {
            buffer[index++] = sizeLow;
            buffer[index++] = sizeHigh;
        }
        
        // Add extended addressing indicator
        buffer[index++] = EXTENDED_ADDRESSING;
        
        // Add 64-bit Unique ID
        for (int i = 0; i < 8; i++) {
            buffer[index++] = (uint8_t)((uniqueId >> (i * 8)) & 0xFF);
        }
        
        // Add command byte
        buffer[index++] = commandID;
        
        // Add payload
        if (payload != nullptr && payloadSize > 0) {
            memcpy(buffer + index, payload, payloadSize);
            index += payloadSize;
        }
        
        // Calculate CRC32
        uint32_t crc = calculate_crc32(buffer, index);
        
        // Write CRC32 (little endian)
        _serial.write((uint8_t)(crc & 0xFF));
        _serial.write((uint8_t)((crc >> 8) & 0xFF));
        _serial.write((uint8_t)((crc >> 16) & 0xFF));
        _serial.write((uint8_t)((crc >> 24) & 0xFF));
        
        delete[] buffer;
    }
    
    #ifdef VERBOSE
    Serial.print("Sent extended command with ");
    Serial.print(_crc32Enabled ? "CRC32 enabled" : "CRC32 disabled");
    Serial.print(", uniqueId: 0x");
    for (int i = 7; i >= 0; i--) {
        uint8_t byte = (uniqueId >> (i * 8)) & 0xFF;
        if (byte < 0x10) Serial.print("0");
        Serial.print(static_cast<int>(byte), HEX);
    }
    Serial.print(", command: ");
    Serial.println(commandID);
    
    // Print raw bytes being sent
    Serial.println("Raw bytes sent (hex):");
    
    // Size byte(s)
    if (totalSize + 1 <= 127) { // +1 for size byte itself
        uint8_t encodedSize = ((totalSize + 1) << FIRST_BYTE_SHIFT) | FIRST_BYTE_LSB_MASK;
        Serial.print("Size byte: 0x");
        if (encodedSize < 0x10) Serial.print("0");
        Serial.print(encodedSize, HEX);
        Serial.print(" (represents ");
        Serial.print(totalSize + 1);
        Serial.println(" bytes total)");
    } else {
        // Extended size format (3 bytes)
        uint8_t encodedFirstByte = (DECODED_FIRST_BYTE_EXTENDED_SIZE << FIRST_BYTE_SHIFT) | FIRST_BYTE_LSB_MASK;
        Serial.print("Size bytes: 0x");
        if (encodedFirstByte < 0x10) Serial.print("0");
        Serial.print(encodedFirstByte, HEX);
        Serial.print(" 0x");
        uint16_t extendedSize = totalSize + 3; // +3 for all three size bytes
        uint8_t sizeLow = (extendedSize & 0xFF);
        if (sizeLow < 0x10) Serial.print("0");
        Serial.print(sizeLow, HEX);
        Serial.print(" 0x");
        uint8_t sizeHigh = ((extendedSize >> 8) & 0xFF);
        if (sizeHigh < 0x10) Serial.print("0");
        Serial.print(sizeHigh, HEX);
        Serial.print(" (represents ");
        Serial.print(extendedSize);
        Serial.println(" bytes total)");
    }
    
    // Extended addressing indicator
    Serial.print("Extended addressing indicator: 0x");
    if (EXTENDED_ADDRESSING < 0x10) Serial.print("0");
    Serial.print(EXTENDED_ADDRESSING, HEX);
    Serial.println();
    
    // Unique ID bytes (little endian)
    Serial.print("Unique ID bytes (little endian): ");
    for (int i = 0; i < 8; i++) {
        uint8_t byte = (uniqueId >> (i * 8)) & 0xFF;
        Serial.print("0x");
        if (byte < 0x10) Serial.print("0");
        Serial.print(byte, HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Command byte
    Serial.print("Command byte: 0x");
    if (commandID < 0x10) Serial.print("0");
    Serial.print(commandID, HEX);
    Serial.println();
    
    // Payload bytes
    if (payload != nullptr && payloadSize > 0) {
        Serial.print("Payload bytes: ");
        for (uint16_t i = 0; i < payloadSize; i++) {
            Serial.print("0x");
            if (payload[i] < 0x10) Serial.print("0");
            Serial.print(payload[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    } else {
        Serial.println("No payload");
    }
    
    // CRC32 bytes
    if (_crc32Enabled) {
        Serial.println("CRC32 bytes: (calculated from data)");
    }
    
    Serial.println("End of extended command");
    #endif
}

int8_t Communication::getResponse(uint8_t* buffer, uint16_t bufferSize, uint16_t& receivedSize) {
    uint32_t startTime = millis();
    uint16_t packetSize = 0;
    bool extendedSize = false;
    
    // Wait for first size byte
    while (!_serial.available()) {
        if (millis() - startTime > TIMEOUT_MS) {
            #ifdef VERBOSE
            Serial.println("Timed out waiting for first byte"); // DEBUG
            #endif
            return COMMUNICATION_ERROR_TIMEOUT;
        }
    }
    
    // Read first byte (encoded size)
    uint8_t firstByte = _serial.read();
    
    #ifdef VERBOSE
    Serial.print("Received first byte: 0x");
    if (firstByte < 0x10) Serial.print("0");
    Serial.println(firstByte, HEX);
    Serial.print("Binary: ");
    for (int i = 7; i >= 0; i--) {
        Serial.print((firstByte >> i) & 1);
    }
    Serial.println();
    #endif
    
    // Validate first byte format (LSB must be 1)
    if (!isValidFirstByteFormat(firstByte)) {
        #ifdef VERBOSE
        Serial.println("Invalid first byte format (LSB not 1)"); // DEBUG
        #endif
        return COMMUNICATION_ERROR_BAD_FIRST_BYTE;
    }
    
    // Decode size from first byte
    uint8_t decodedSize = decodeFirstByte(firstByte);
    
    #ifdef VERBOSE
    Serial.print("Decoded size: ");
    Serial.println(decodedSize);
    #endif
    
    // Check if we have extended size format
    if (decodedSize == DECODED_FIRST_BYTE_EXTENDED_SIZE) {
        extendedSize = true;
        
        // Wait for extended size bytes (2 bytes)
        while (_serial.available() < 2) {
            if (millis() - startTime > TIMEOUT_MS) {
                #ifdef VERBOSE
                Serial.println("Timed out waiting for extended size bytes"); // DEBUG
                #endif
                return COMMUNICATION_ERROR_TIMEOUT;
            }
        }
        
        // Read extended size (little endian)
        uint8_t sizeLow = _serial.read();
        uint8_t sizeHigh = _serial.read();
        
        #ifdef VERBOSE
        Serial.print("Extended size bytes: 0x");
        if (sizeLow < 0x10) Serial.print("0");
        Serial.print(sizeLow, HEX);
        Serial.print(" 0x");
        if (sizeHigh < 0x10) Serial.print("0");
        Serial.println(sizeHigh, HEX);
        #endif
        
        // Combine into 16-bit size
        packetSize = (uint16_t)sizeLow | ((uint16_t)sizeHigh << 8);
    } else {
        // Use decoded size directly
        packetSize = decodedSize;
    }
    
    // Important: packetSize includes the size byte itself, so we need to subtract
    // 1 for standard size format or 3 for extended size format
    if (extendedSize) {
        packetSize -= 3; // Subtract the 3 size bytes
    } else {
        packetSize -= 1; // Subtract the 1 size byte
    }
    
    #ifdef VERBOSE
    Serial.print("Total packet size (excluding size byte): ");
    Serial.println(packetSize);
    #endif
    
    // Read response character
    startTime = millis();
    while (_serial.available() < 1) {
        if (millis() - startTime > TIMEOUT_MS) {
            #ifdef VERBOSE
            Serial.println("Timed out waiting for response character"); // DEBUG
            #endif
            return COMMUNICATION_ERROR_TIMEOUT;
        }
    }
    uint8_t responseChar = _serial.read();
    #ifdef VERBOSE
    Serial.print("Response character: 0x");
    if (responseChar < 0x10) Serial.print("0");
    Serial.println(responseChar, HEX);
    #endif
    
    if ((responseChar != RESPONSE_CHARACTER_CRC32_ENABLED) && (responseChar != RESPONSE_CHARACTER_CRC32_DISABLED)) {
        #ifdef VERBOSE
        Serial.print("Invalid response character. Expected 0x"); // DEBUG
        Serial.print(RESPONSE_CHARACTER_CRC32_ENABLED, HEX);
        Serial.print(" or 0x");
        Serial.print(RESPONSE_CHARACTER_CRC32_DISABLED, HEX);
        Serial.print(", but got 0x");
        Serial.println(responseChar, HEX); // DEBUG
        #endif
        return COMMUNICATION_ERROR_BAD_RESPONSE_CHAR;
    }

    // Read command byte
    startTime = millis();
    while (_serial.available() < 1) {
        if (millis() - startTime > TIMEOUT_MS) {
            #ifdef VERBOSE
            Serial.println("Timed out waiting for command byte"); // DEBUG
            #endif
            return COMMUNICATION_ERROR_TIMEOUT;
        }
    }
    uint8_t commandByte = _serial.read();
    #ifdef VERBOSE
    Serial.print("Command byte: ");
    Serial.println(commandByte);
    #endif
    
    // Validate command byte
    if (commandByte != 0 && commandByte != 1) {
        #ifdef VERBOSE
        Serial.println("Invalid command byte. Expected 0 or 1, but got something else."); // DEBUG
        #endif
        return COMMUNICATION_ERROR_BAD_THIRD_BYTE;
    }

    // Calculate the payload size
    uint16_t payloadSize = packetSize - 2; // Subtract 3 for the response character, command byte, and (optional) CRC32 size
    if (responseChar == RESPONSE_CHARACTER_CRC32_ENABLED) {
        payloadSize -= 4; // Subtract 4 for the CRC32 size if it's enabled
    }

    // Check if the buffer size matches the calculated payload size
    if (bufferSize != payloadSize) {
        #ifdef VERBOSE
        Serial.print("Buffer size is not the right size for the payload. Expected "); // DEBUG
        Serial.print(payloadSize); // DEBUG
        Serial.print(" bytes, but buffer is "); // DEBUG
        Serial.print(bufferSize); // DEBUG
        Serial.println(" bytes."); // DEBUG
        #endif
        return COMMUNICATION_ERROR_BUFFER_TOO_SMALL;
    }
    
    // Wait for the rest of the packet
    #ifdef VERBOSE
    Serial.print("Waiting for ");
    Serial.print(payloadSize);
    Serial.println(" more bytes (which is the size of the payload)");
    #endif
    
    uint32_t waitStartTime = millis();
    while (_serial.available() < payloadSize) {
        if (millis() - waitStartTime > TIMEOUT_MS) {
            #ifdef VERBOSE
            Serial.print("Timed out waiting for the payload. Expected "); // DEBUG
            Serial.print(payloadSize); // DEBUG
            Serial.print(" more bytes, but only "); // DEBUG
            Serial.print(_serial.available()); // DEBUG
            Serial.println(" available."); // DEBUG
            #endif
            return COMMUNICATION_ERROR_TIMEOUT;
        }
    }
    
    if (payloadSize > 0 && buffer != NULL) {
        for (uint16_t i = 0; i < payloadSize; i++) {
            buffer[i] = _serial.read();
        }
    }
    
    #ifdef VERBOSE
    Serial.print("Received full payload of ");
    Serial.print(payloadSize);
    Serial.println(" bytes");
    Serial.println("Raw packet data (hex):");
    for (uint16_t i = 0; i < payloadSize; i++) {
        Serial.print("0x");
        if (buffer[i] < 0x10) Serial.print("0");
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
        if ((i + 1) % 8 == 0) Serial.println();
    }
    Serial.println();
    #endif

    // Validate CRC32 if the response indicates it's enabled
    if (responseChar == RESPONSE_CHARACTER_CRC32_ENABLED) {
        // Receive the four bytes for the CRC32
        uint32_t waitStartTime = millis();
        while (_serial.available() < 4) {
            if (millis() - waitStartTime > TIMEOUT_MS) {
                #ifdef VERBOSE
                Serial.print("Timed out waiting for CRC32. Expected "); // DEBUG
                Serial.print(4); // DEBUG
                Serial.print(" more bytes, but only "); // DEBUG
                Serial.print(_serial.available()); // DEBUG
                Serial.println(" available."); // DEBUG
                #endif
                return COMMUNICATION_ERROR_TIMEOUT;
            }
        }
        
        // Extract CRC32 from the received bytes
        uint32_t receivedCRC = 0;
        receivedCRC |= _serial.read();
        receivedCRC |= _serial.read() << 8;
        receivedCRC |= _serial.read() << 16;
        receivedCRC |= _serial.read() << 24;
        
        #ifdef VERBOSE
        Serial.print("Received CRC32: 0x");
        Serial.println(receivedCRC, HEX);
        #endif
        
        // Calculate CRC32 of the packet data (excluding the CRC32 itself)
        uint8_t tempBuffer[payloadSize + 3]; // +3 for size byte, response character, and command byte
        tempBuffer[0] = firstByte; // size byte
        tempBuffer[1] = responseChar; // response character
        tempBuffer[2] = commandByte; // command byte
        if (payloadSize > 0) {
            memcpy(tempBuffer + 3, buffer, packetSize); // payload
        }
        uint32_t calculatedCRC = calculate_crc32(tempBuffer, sizeof(tempBuffer));
        
        #ifdef VERBOSE
        Serial.print("Calculated CRC32: 0x");
        Serial.println(calculatedCRC, HEX);
        #endif
        
        // Verify CRC32
        if (receivedCRC != calculatedCRC) {
            #ifdef VERBOSE
            Serial.print("CRC32 mismatch. Received: "); // DEBUG
            Serial.print(static_cast<int>(receivedCRC), HEX); // DEBUG
            Serial.print(", Calculated: "); // DEBUG
            Serial.println(static_cast<int>(calculatedCRC), HEX); // DEBUG
            #endif
            return COMMUNICATION_ERROR_CRC32_MISMATCH;
        }
    }
    
    receivedSize = payloadSize;
    return COMMUNICATION_SUCCESS;
}

void Communication::flush() {
    _serial.flush(); // Flush any outgoing data
    while (_serial.available()) {
        _serial.read(); // Clear any incoming data
    }
}

void Communication::enableCRC32() {
    _crc32Enabled = true;
}

void Communication::disableCRC32() {
    _crc32Enabled = false;
}

bool Communication::isCRC32Enabled() const {
    return _crc32Enabled;
}
