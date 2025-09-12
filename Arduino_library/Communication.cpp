// Communication.cpp
#include "Communication.h"
#include <limits>  // For std::numeric_limits

//#define VERBOSE
#define TIMEOUT_MS 1000  // Define timeout duration (1 second)

#ifndef ARDUINO
// Define Arduino constants for desktop builds
#define HEX 16
#endif

#define CRC32_POLYNOMIAL 0xEDB88320

static uint32_t crc32_value;
static bool s_commSerialOpened = false;

void crc32_init(void)
{
    crc32_value = 0xFFFFFFFF;
}

uint32_t calculate_crc32_buffer_without_reinit(const void* data, size_t length)
{
    uint32_t i, j;
    uint8_t *d = (uint8_t *)data;
    for (i = 0; i < length; i++) {
        crc32_value ^= d[i];
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
#ifdef ARDUINO
    // Initialize the RS485 hardware serial port once (even if multiple Servomotor instances exist)
    if (!s_commSerialOpened) {
        _serial.begin(230400);
        s_commSerialOpened = true;
    }
#else
    // On desktop (ArduinoEmulator), Serial1 is initialized in ArduinoEmulator.cpp main()
#endif
}

void Communication::sendCommand(uint8_t alias, uint8_t commandID, const uint8_t* payload, uint16_t payloadSize) {
    sendCommandCore(false, alias, commandID, payload, payloadSize);
}

void Communication::sendCommandByUniqueId(uint64_t uniqueId, uint8_t commandID, const uint8_t* payload, uint16_t payloadSize) {
    sendCommandCore(true, uniqueId, commandID, payload, payloadSize);
}

void Communication::sendCommandCore(bool isExtendedAddress, uint64_t addressValue, uint8_t commandID,
                                   const uint8_t* payload, uint16_t payloadSize) {
    
    uint8_t sizeByte;
    bool isExtendedSize = false;
    uint16_t size16Bit;
    // Calculate address size
    const uint16_t addressSize = isExtendedAddress ? (sizeof(uint8_t) + sizeof(uint64_t)) : sizeof(uint8_t); // Extended: 1 byte flag + 8 bytes ID, Standard: 1 byte alias
    
    // Calculate total packet size (initial calculation without extended size bytes)
    uint32_t totalPacketSize = sizeof(sizeByte) + addressSize + sizeof(commandID) + payloadSize;
    if (_crc32Enabled) {
        totalPacketSize += sizeof(uint32_t); // Add 4 bytes for CRC32
    }
    
    // Determine the size byte value
    if (totalPacketSize <= DECODED_FIRST_BYTE_EXTENDED_SIZE) {
        sizeByte = encodeFirstByte(totalPacketSize);
        _serial.write(sizeByte);
    } else {
        isExtendedSize = true;
        sizeByte = encodeFirstByte(DECODED_FIRST_BYTE_EXTENDED_SIZE);
        totalPacketSize += sizeof(uint16_t); // Add 2 bytes for the extended size
        if (totalPacketSize > std::numeric_limits<uint16_t>::max()) {
            #ifdef VERBOSE
            Serial.println("Packet larger than protocol supports. Nothing being transmitted.");
            #endif
            return;
        }
        size16Bit = (uint16_t)totalPacketSize;
        _serial.write(sizeByte);
        _serial.write((uint8_t *)&size16Bit, sizeof(size16Bit));
    }
    
    // Initialize CRC calculation if enabled
    if (_crc32Enabled) {
        crc32_init();
        calculate_crc32_buffer_without_reinit(&sizeByte, sizeof(sizeByte));
        if (isExtendedSize) {
            calculate_crc32_buffer_without_reinit(&size16Bit, sizeof(size16Bit));
        }
    }
    
    // Write address bytes
    if (isExtendedAddress) {
        const uint8_t extendedAddrByte = EXTENDED_ADDRESSING;
        _serial.write(extendedAddrByte);
        _serial.write((uint8_t *)&addressValue, sizeof(addressValue));
        if (_crc32Enabled) {
            calculate_crc32_buffer_without_reinit(&extendedAddrByte, sizeof(extendedAddrByte));
            calculate_crc32_buffer_without_reinit(&addressValue, sizeof(addressValue));        
        }
    }
    else {
        const uint8_t alias = (uint8_t)addressValue;
        _serial.write(alias);
        
        // Add to CRC if enabled
        if (_crc32Enabled) {
            calculate_crc32_buffer_without_reinit(&alias, sizeof(alias));
        }
    }
    
    // Write command byte
    _serial.write(commandID);
    
    #ifdef VERBOSE
    if (isExtendedAddress) {
        Serial.print("Sent extended command with ");
        Serial.print(_crc32Enabled ? "CRC32 enabled" : "CRC32 disabled");
        Serial.print(", uniqueId: 0x");
        // Print Unique ID in big-endian format for readability
        for (int i = 7; i >= 0; i--) {
            uint8_t byte = (addressValue >> (i * 8)) & 0xFF;
            if (byte < 0x10) Serial.print("0");
            Serial.print(static_cast<int>(byte), HEX);
        }
        Serial.print(", command: 0x");
        if (commandID < 0x10) Serial.print("0");
        Serial.println(commandID, HEX);
        Serial.println();
    }
    #endif

    // Add command to CRC if enabled
    if (_crc32Enabled) {
        calculate_crc32_buffer_without_reinit(&commandID, sizeof(commandID));
    }
    
    // Write payload
    if (payload != nullptr && payloadSize > 0) {
        _serial.write(payload, payloadSize);
        
        // Add payload to CRC if enabled
        if (_crc32Enabled) {
            calculate_crc32_buffer_without_reinit(payload, payloadSize);
        }
        
        #ifdef VERBOSE
        Serial.print("Payload bytes");
        if (isExtendedAddress) Serial.print(" (extended address)");
        Serial.print(": ");
        for (uint16_t i = 0; i < payloadSize; i++) {
            Serial.print("0x");
            if (payload[i] < 0x10) Serial.print("0");
            Serial.print(payload[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        #endif
    }
    else {
        #ifdef VERBOSE
        Serial.print("No payload");
        if (isExtendedAddress) Serial.print(" (extended address)");
        Serial.println();
        #endif
    }
    
    // Calculate and write CRC32 if enabled
    if (_crc32Enabled) {
        // Get final CRC and write it
        uint32_t crc = get_crc32();
        _serial.write((uint8_t*)(&crc), sizeof(crc));
        
        #ifdef VERBOSE
        Serial.print("calculated CRC32 bytes sent");
        if (isExtendedAddress) Serial.print(" (extended)");
        Serial.print(": 0x");
        Serial.print(crc, HEX);
        Serial.println();
        #endif
    }
}

int16_t Communication::getResponse(uint8_t* buffer, uint16_t bufferSize, uint16_t& receivedSize) {
    uint32_t startTime = millis();
    bool isExtendedSize = false;
    uint8_t sizeBytes[3];
    uint8_t sizeByteCount = 1;
    int32_t packetSize = 0;
    int16_t error_code = 0;
    int32_t bytesLeftToRead = 0;
    uint8_t remoteErrorCode = 0;
    uint8_t remoteErrorCodePresent = 0;
    int32_t bytesLeftToReadWithoutTheCRC32;

    // Read first (encoded) size byte. From it we will determine the packet size or determine it to indicate extended size
    if((error_code = receiveBytes(&(sizeBytes[0]), 1, 1, TIMEOUT_MS - (millis() - startTime))) != 0) {
        return error_code;
    }

    #ifdef VERBOSE
    Serial.print("Received first byte: 0x");
    if (sizeBytes[0] < 0x10) Serial.print("0");
    Serial.println(sizeBytes[0], HEX);
    Serial.print("Binary: ");
    for (int i = 7; i >= 0; i--) {
        Serial.print((sizeBytes[0] >> i) & 1);
    }
    Serial.println();
    #endif

    // Validate first byte format (LSB must be 1)
    if (!isValidFirstByteFormat(sizeBytes[0])) {
        #ifdef VERBOSE
        Serial.println("Invalid first byte format (LSB not 1)");
        #endif
        return COMMUNICATION_ERROR_BAD_FIRST_BYTE;
    }

    // Decode size from first byte
    uint8_t decodedSize = decodeFirstByte(sizeBytes[0]);

    #ifdef VERBOSE
    Serial.print("Decoded size: ");
    Serial.println(decodedSize);
    #endif

    // Check if we have extended size format
    if (decodedSize == DECODED_FIRST_BYTE_EXTENDED_SIZE) {
        isExtendedSize = true;

        // We determined that the size byte indicates an extended size where the size is incoded in 16-bite. Read them.
        uint16_t extendedSize;
        if((error_code = receiveBytes(&extendedSize, sizeof(extendedSize), sizeof(extendedSize), TIMEOUT_MS - (millis() - startTime))) != 0) {
            return error_code;
        }
        sizeBytes[1] = ((uint8_t*)&extendedSize)[0];
        sizeBytes[2] = ((uint8_t*)&extendedSize)[1];
        sizeByteCount = 3;

        #ifdef VERBOSE
        Serial.print("Extended size bytes: 0x");
        if (sizeBytes[1] < 0x10) Serial.print("0");
        Serial.print(sizeBytes[1], HEX);
        Serial.print(" 0x");
        if (sizeBytes[2] < 0x10) Serial.print("0");
        Serial.println(sizeBytes[2], HEX);
        #endif

        bytesLeftToRead = extendedSize - sizeByteCount;
    } else {
        bytesLeftToRead = decodedSize - sizeByteCount;
    }

    #ifdef VERBOSE
    Serial.print("The number of bytes left to read are: ");
    Serial.println(bytesLeftToRead);
    #endif

    if(bytesLeftToRead < 1) { // we need at least one extra byte beyond the size byte(s)
        #ifdef VERBOSE
        Serial.println("The packet size is too small to be valid");
        #endif
        error_code = COMMUNICATION_ERROR_PACKET_TOO_SMALL;
        goto flush_read_remaining_bytes_and_return_error;
    }
        
    // Read response character
    uint8_t responseChar;
    if((error_code = receiveBytes(&responseChar, 1, 1, TIMEOUT_MS - (millis() - startTime))) != 0) {
        goto flush_read_remaining_bytes_and_return_error;
    }
    #ifdef VERBOSE
    Serial.print("Response character: 0x");
    if (responseChar < 0x10) Serial.print("0");
    Serial.println(responseChar, HEX);
    #endif
    // Calculate the payload size (everything after response character)
    // This includes the error code byte plus actual data
    bytesLeftToRead--; // Subtract 1 for the response character
    
    if ((responseChar != RESPONSE_CHARACTER_CRC32_ENABLED) && (responseChar != RESPONSE_CHARACTER_CRC32_DISABLED)) {
        #ifdef VERBOSE
        Serial.print("Invalid response character. Expected 0x"); // DEBUG
        Serial.print(RESPONSE_CHARACTER_CRC32_ENABLED, HEX);
        Serial.print(" or 0x");
        Serial.print(RESPONSE_CHARACTER_CRC32_DISABLED, HEX);
        Serial.print(", but got 0x");
        Serial.println(responseChar, HEX); // DEBUG
        #endif
        error_code = COMMUNICATION_ERROR_BAD_RESPONSE_CHAR;
        goto flush_read_remaining_bytes_and_return_error;
    }

    bytesLeftToReadWithoutTheCRC32 = bytesLeftToRead;
    if (responseChar == RESPONSE_CHARACTER_CRC32_ENABLED) {
        if(bytesLeftToRead < 4) { // we need at least 4 bytes for the CRC32
            #ifdef VERBOSE
            Serial.println("The packet size is too small to hold the expected 4-byte CRC32 value");
            #endif
            error_code = COMMUNICATION_ERROR_PACKET_TOO_SMALL;
            goto flush_read_remaining_bytes_and_return_error;
        }
        bytesLeftToReadWithoutTheCRC32 -= 4; // Subtract 4 for the CRC32 size if it's enabled
    }
    if ((bytesLeftToReadWithoutTheCRC32 == 0) && (bufferSize != 0)) {
        error_code = COMMUNICATION_ERROR_DATA_WRONG_SIZE;
        goto flush_read_remaining_bytes_and_return_error;
    }

    // Now receive the error code byte from the remote device if there are enough bytes left
    if (bytesLeftToReadWithoutTheCRC32 >= 1) {
        if((error_code = receiveBytes(&remoteErrorCode, sizeof(remoteErrorCode), sizeof(remoteErrorCode), TIMEOUT_MS - (millis() - startTime))) != 0) {
            goto flush_read_remaining_bytes_and_return_error;
        }
        remoteErrorCodePresent = 1;
        if (remoteErrorCode != 0) {
            error_code = remoteErrorCode;
            goto flush_read_remaining_bytes_and_return_error;
        }
        bytesLeftToReadWithoutTheCRC32--; // Subtract 1 for the remote error code that we just read
    }

    // Read the payload
    if((error_code = receiveBytes(buffer, bufferSize, bytesLeftToReadWithoutTheCRC32, TIMEOUT_MS - (millis() - startTime))) != 0) {
        goto flush_read_remaining_bytes_and_return_error;
    }
    receivedSize = bytesLeftToReadWithoutTheCRC32;

    // Receive the CRC32 if there is one
    if (responseChar == RESPONSE_CHARACTER_CRC32_ENABLED) {
        uint32_t crc32 = 0;
        if((error_code = receiveBytes(&crc32, sizeof(crc32), sizeof(crc32), TIMEOUT_MS - (millis() - startTime))) != 0) {
            goto flush_read_remaining_bytes_and_return_error;
        }
        // And also calculate the CRC32 and compare to make sure that the received CRC32 is correct. Include all bytes, including the size bytes, in the calculation except don't include the CRC32 bytes themselves.
        calculate_crc32(sizeBytes, sizeByteCount);
        calculate_crc32_buffer_without_reinit(&responseChar, sizeof(responseChar));
        if (remoteErrorCodePresent) {
            calculate_crc32_buffer_without_reinit(&remoteErrorCode, sizeof(remoteErrorCode));
        }
        uint32_t calculated_crc32 = calculate_crc32_buffer_without_reinit(buffer, bytesLeftToReadWithoutTheCRC32);
        if (calculated_crc32 != crc32) {
            #ifdef VERBOSE
            Serial.print("CRC32 mismatch! Calculated: 0x");
            Serial.print(calculated_crc32, HEX);
            Serial.print(", Received: 0x");
            Serial.println(crc32, HEX);
            #endif
            error_code = COMMUNICATION_ERROR_CRC32_MISMATCH;
            goto flush_read_remaining_bytes_and_return_error;
        }
    }

    return COMMUNICATION_SUCCESS;

/*
    // Always read full payload (needed for CRC32 calculation)
    // receiveBytes will handle buffer overflow gracefully
    if (payloadSize > 0) {
        // We are expecting more bytes. Two cases:
        // (1) there is just one more byte, and that should be an error code or 0 if no error
        // (2) there is more than one byte, and we need to read the error code first (one byte) and then we need to read the entire payload into the given receive buffer
        #ifdef VERBOSE
        Serial.print("There is an error status byte present and we need to read it now. After that error status byte, there are ");
        Serial.print(payloadSize - 1);
        Serial.println(" more bytes (which is the payload)");
        #endif
        if((result = receiveBytes(buffer, bufferSize, payloadSize)) != 0) {
            #ifdef VERBOSE
            Serial.println("An error occured while receiving");
            Serial.print("Error code: ");
            Serial.println(result);
            #endif
            return result;
        }
        
        #ifdef VERBOSE
        Serial.print("Waiting for ");
        Serial.print(payloadSize);
        Serial.println(" more bytes (which is the size of the payload)");
        Serial.print("Received full payload of ");
        Serial.print(payloadSize);
        Serial.println(" bytes");
        Serial.println("Raw packet data (hex):");
        uint16_t printSize = (buffer != nullptr) ? payloadSize : 0;
        for (uint16_t i = 0; i < printSize; i++) {
            Serial.print("0x");
            if (buffer[i] < 0x10) Serial.print("0");
            Serial.print(buffer[i], HEX);
            Serial.print(" ");
            if ((i + 1) % 8 == 0) Serial.println();
        }
        if (printSize % 8 != 0) Serial.println();
        #endif
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
        // This matches the Python implementation: size_bytes + packet_data[:-4]
        uint8_t crcData[payloadSize + 4]; // max: 3 size + 1 response
        memcpy(crcData, sizeBytes, sizeByteCount);
        crcData[sizeByteCount] = responseChar;
        if (payloadSize > 0) {
            memcpy(crcData + sizeByteCount + 1, buffer, payloadSize);
        }
        uint32_t calculatedCRC = calculate_crc32(crcData, sizeByteCount + 1 + payloadSize);

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
    
    // Process error code and adjust payload
    if (payloadSize == 0) {
        // Case 1: Empty payload - success response
        #ifdef VERBOSE
        Serial.println("Empty payload - success response");
        #endif
        receivedSize = 0;
        return COMMUNICATION_SUCCESS;
    }
    
    // Extract error code from first byte of payload
    // Note: we might not have the data if buffer was too small, but we have error code
    uint8_t errorCode = (buffer != nullptr) ? buffer[0] : 0; // Default to success if no buffer
    #ifdef VERBOSE
    Serial.print("Error code in payload: ");
    Serial.println(errorCode);
    #endif
    
    if (payloadSize == 1) {
        // Case 2: Only error code byte
        receivedSize = 0;
        return (errorCode == 0) ? COMMUNICATION_SUCCESS : -errorCode;
    }
    
    // Case 3: Error code + data bytes
    uint16_t dataSize = payloadSize - 1;
    
    // Check if buffer can hold final data size
    if (buffer != nullptr && bufferSize < dataSize) {
        #ifdef VERBOSE
        Serial.print("Buffer too small for final data. Need ");
        Serial.print(dataSize);
        Serial.print(" bytes, buffer is ");
        Serial.println(bufferSize);
        #endif
        receivedSize = 0;
        return COMMUNICATION_ERROR_BUFFER_TOO_SMALL;
    }
    
    // If we have a buffer and it was big enough for data, move data
    if (buffer != nullptr && bufferSize >= dataSize) {
        memmove(buffer, buffer + 1, dataSize);
        receivedSize = dataSize;
    } else {
        receivedSize = 0;
    }
    
    #ifdef VERBOSE
    Serial.print("Actual payload size after removing error code: ");
    Serial.println(receivedSize);
    #endif
    
    return (errorCode == 0) ? COMMUNICATION_SUCCESS : -errorCode;
*/
flush_read_remaining_bytes_and_return_error:
    if (bytesLeftToRead > 0) {
        receiveBytes(nullptr, 0, bytesLeftToRead, TIMEOUT_MS - (millis() - startTime));
    }
    return error_code;
}

void Communication::flush() {
    _serial.flush(); // Flush any outgoing data
    while (_serial.available()) {
        _serial.read(); // Clear any incoming data
    }
}

int8_t Communication::receiveBytes(void* buffer, uint16_t bufferSize, int32_t numBytes, int32_t timeout_ms) {
    if (numBytes == 0) {
        return COMMUNICATION_SUCCESS;
    }

    if (numBytes < 0) {
        return COMMUNICATION_ERROR_PACKET_TOO_SMALL;
    }


    bool bufferTooSmall = (buffer != nullptr && bufferSize < numBytes);

    // Wait for all bytes to arrive
    uint32_t startTime = millis();
    while (_serial.available() < numBytes) {
        if (millis() - startTime > timeout_ms) {
            #ifdef VERBOSE
            Serial.println("A timeout error occured while receiving");
            #endif
            return COMMUNICATION_ERROR_TIMEOUT;
        }
    }

    // Read all bytes (store in buffer if adequate, otherwise discard)
    for (uint16_t i = 0; i < numBytes; i++) {
        uint8_t byte = _serial.read();
        if (buffer != nullptr && !bufferTooSmall) {
            ((char*)buffer)[i] = byte;
        }
    }

    if(bufferTooSmall) {
        #ifdef VERBOSE
        Serial.print("Buffer too small for final data. Need ");
        Serial.print(numBytes);
        Serial.print(" bytes, buffer is ");
        Serial.println(bufferSize);
        #endif
        return COMMUNICATION_ERROR_BUFFER_TOO_SMALL;
    }
    else {
        return COMMUNICATION_SUCCESS;
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
