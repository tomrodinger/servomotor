#include "test_framework.h"
#include "Communication.h"  // For error code constants
#include <iomanip>
#include <iostream>
#include <cstdlib>  // For exit()

// Error code to message mapping
const char* getErrorMessage(int errorCode) {
    switch (errorCode) {
        case COMMUNICATION_SUCCESS:
            return "No error";
        case COMMUNICATION_ERROR_TIMEOUT:
            return "Communication error: Timeout";
        case COMMUNICATION_ERROR_DATA_WRONG_SIZE:
            return "Communication error: Wrong data size";
        case COMMUNICATION_ERROR_BAD_RESPONSE_CHAR:
            return "Communication error: Bad response character";
        case COMMUNICATION_ERROR_BAD_STATUS_CHAR:
            return "Communication error: Bad status character";
        case COMMUNICATION_ERROR_BUFFER_TOO_SMALL:
            return "Communication error: Buffer too small";
        default:
            return "Unknown error";
    }
}

void checkMotorError(ServoMotor& motor, const std::string& commandName) {
    int error = motor.getError();
    if (error != 0) {
        std::cerr << "\nERROR: Motor failed to respond to " << commandName << " command.\n";
        std::cerr << "Error code: " << error << " - " << getErrorMessage(error) << "\n";
        std::cerr << "Make sure the motor is connected and powered on.\n";
        exit(1);
    }
}

// Initialize static member
std::vector<TestResult> TestRunner::results;

void TestRunner::addResult(const std::string& name, bool passed, const std::string& msg) {
    results.push_back({name, passed, msg});
}

void TestRunner::printResults() {
    // Calculate column widths
    size_t nameWidth = 4;  // "Name"
    size_t statusWidth = 6;  // "Status"
    size_t messageWidth = 7;  // "Message"

    for (const auto& result : results) {
        nameWidth = std::max(nameWidth, result.name.length());
        messageWidth = std::max(messageWidth, result.message.length());
    }

    // Print header
    std::cout << "\nTest Results\n";
    std::cout << std::string(nameWidth + statusWidth + messageWidth + 8, '=') << "\n";
    std::cout << std::left << std::setw(nameWidth) << "Name" << " | "
              << std::setw(statusWidth) << "Status" << " | "
              << "Message\n";
    std::cout << std::string(nameWidth + statusWidth + messageWidth + 8, '-') << "\n";

    // Print results
    int passed = 0;
    for (const auto& result : results) {
        std::cout << std::left << std::setw(nameWidth) << result.name << " | "
                  << std::setw(statusWidth) << (result.passed ? "PASS" : "FAIL") << " | "
                  << result.message << "\n";
        if (result.passed) passed++;
    }

    // Print summary
    std::cout << std::string(nameWidth + statusWidth + messageWidth + 8, '-') << "\n";
    std::cout << "Summary: " << passed << "/" << results.size() << " tests passed ("
              << (results.empty() ? 0 : (passed * 100 / results.size())) << "%)\n";
}

bool TestRunner::allTestsPassed() {
    for (const auto& result : results) {
        if (!result.passed) return false;
    }
    return true;
}
