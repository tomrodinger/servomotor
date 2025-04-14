#include "test_framework.h"
#include "Communication.h"  // For error code constants
#include <iomanip> // For std::setw, std::setfill
#include <iostream>
#include <cstdlib>  // For exit()
#include <sstream> // For std::stringstream
#include <string>  // For std::string

// Error code to message mapping
const char* getErrorMessage(int errorCode) {
    switch (errorCode) {
        case COMMUNICATION_SUCCESS:
            return "No error";
        case COMMUNICATION_ERROR_TIMEOUT:
            return "Communication error: Timeout waiting for response bytes";
        case COMMUNICATION_ERROR_DATA_WRONG_SIZE:
            return "Communication error: Received data size does not match expected size";
        case COMMUNICATION_ERROR_BAD_RESPONSE_CHAR:
            return "Communication error: Invalid response character (not 252 or 253)";
        case COMMUNICATION_ERROR_BUFFER_TOO_SMALL:
            return "Communication error: Buffer size mismatch with received payload";
        case COMMUNICATION_ERROR_CRC32_MISMATCH:
            return "Communication error: CRC32 validation failed";
        case COMMUNICATION_ERROR_BAD_FIRST_BYTE:
            return "Communication error: Invalid first byte format (LSB not set)";
        case COMMUNICATION_ERROR_BAD_THIRD_BYTE:
            return "Communication error: Invalid command byte in response";
        default:
            return "Unknown error";
    }
}

// Initialize static member
std::vector<TestResult> TestRunner::results;

void TestRunner::addResult(const std::string& name, bool passed, const std::string& msg) {
    results.push_back({name, passed, msg});
}

void TestRunner::printResults() {
    std::cout << "\nTest Results Summary\n";
    std::cout << "==================\n";
    
    int maxNameLength = 0;
    for (const auto& result : results) {
        maxNameLength = std::max(maxNameLength, static_cast<int>(result.name.length()));
    }
    
    for (const auto& result : results) {
        std::cout << std::left << std::setw(maxNameLength + 2) << result.name
                  << (result.passed ? "PASS" : "FAIL");
        if (!result.message.empty()) {
            std::cout << " - " << result.message;
        }
        std::cout << "\n";
    }
}

bool TestRunner::allTestsPassed() {
    for (const auto& result : results) {
        if (!result.passed) {
            return false;
        }
    }
    return true;
}

bool TestRunner::parseArgs(int argc, char* argv[], std::string& serialPort, std::string& deviceId, std::string& addressingMode) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <serial_port> <device_id>\n";
        std::cerr << "  serial_port: Serial port device (e.g. /dev/ttys014)\n";
        std::cerr << "  device_id: Device alias (ASCII char or number 0-251) or unique ID (16-char hex)\n";
        std::cerr << "    - If device_id is 4 or more characters, it is treated as a unique ID\n";
        std::cerr << "    - If device_id is less than 4 characters, it is treated as an alias\n";
        return false;
    }

    serialPort = argv[1];
    deviceId = argv[2];

    // Determine addressing mode based on device ID length
    if (deviceId.length() >= 4) {
        // Treat as unique ID
        addressingMode = "unique";
        // Validate unique ID format (16-character hex)
        if (deviceId.length() != 16 || deviceId.find_first_not_of("0123456789ABCDEFabcdef") != std::string::npos) {
            std::cerr << "Error: Unique ID must be a 16-character hex number\n";
            return false;
        }
    } else {
        // Treat as alias
        addressingMode = "alias";
        // Check if alias is a single character or number 0-251
        if (deviceId.length() == 1 && isalnum(deviceId[0])) {
            // Valid single character alias
        } else if (deviceId.find_first_not_of("0123456789") == std::string::npos) {
            // Check if it's a valid number
            int aliasNum = std::stoi(deviceId);
            if (aliasNum < 0 || aliasNum > 251) {
                std::cerr << "Error: Alias number must be between 0 and 251\n";
                return false;
            }
        } else {
            std::cerr << "Error: Alias must be a single character or number 0-251\n";
            return false;
        }
    }

    return true;
}

#if 0
Servomotor createMotor(const std::string& serialPort, const std::string& deviceId, const std::string& addressingMode) {
    // Initialize Serial1 with the specified port
    Serial1.begin(230400);
    
    if (addressingMode == "alias") {
        // Create motor with alias
        if (deviceId.length() == 1 && isalnum(deviceId[0])) {
            return Servomotor(deviceId[0], Serial1);
        } else {
            // Convert numeric alias to char
            int aliasNum = std::stoi(deviceId);
            return Servomotor(static_cast<char>(aliasNum), Serial1);
        }
    } else {
        // Create motor with unique ID
        // Convert hex string to uint64_t
        uint64_t uniqueId;
        std::stringstream ss;
        ss << std::hex << deviceId;
        ss >> uniqueId;
        return Servomotor::withUniqueId(uniqueId, Serial1);
    }
}
#endif

void checkMotorError(Servomotor& motor, const std::string& commandName) {
    if (motor.getError() != 0) {
        std::cerr << "Error in " << commandName << ": " << getErrorMessage(motor.getError()) << "\n";
        exit(1);
    }
}

Servomotor* Servomotor_TestModeConvenienceWrapper() {
    if (g_useUniqueId) {
        // Use extended addressing with uniqueID
        // Use stringstream for reliable hex formatting
        std::stringstream ss;
        ss << std::hex << std::setw(16) << std::setfill('0') << g_uniqueId;
        Serial.print("Creating motor with uniqueID: 0x");
        Serial.println(ss.str().c_str()); // Use c_str() for ConsoleSerial compatibility
        
        // Create motor with standard addressing first
        Servomotor* motor = new Servomotor(255, Serial1);
        
        // Then set it up for extended addressing by default
        motor->useUniqueId(g_uniqueId);
        
        return motor;
    } else {
        // Use standard addressing with alias
        Serial.print("Creating motor with alias: ");
        Serial.println(g_motorAlias);
        return new Servomotor(g_motorAlias, Serial1);
    }
}
