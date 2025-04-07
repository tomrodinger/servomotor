#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <cstdlib>
#include "Servomotor.h"

// Helper function to check if two floats are approximately equal
inline bool approxEqual(float a, float b, float tolerance = 0.01f) {
    return std::fabs(a - b) <= tolerance;
}

// Represents the result of a single test
struct TestResult {
    std::string name;      // Name of the test
    bool passed;           // Whether the test passed
    std::string message;   // Optional message (e.g. error details)
};

// Static class to manage test results
class TestRunner {
public:
    // Add a test result
    static void addResult(const std::string& name, bool passed, const std::string& msg = "");
    
    // Print results in a table format
    static void printResults();
    
    // Returns true if all tests passed
    static bool allTestsPassed();

    // Parse command line arguments
    static bool parseArgs(int argc, char* argv[], std::string& serialPort, std::string& deviceId, std::string& addressingMode);

private:
    static std::vector<TestResult> results;
};

// Function to check for motor errors and exit if an error is found
void checkMotorError(Servomotor& motor, const std::string& commandName);

// Function to create a Servomotor instance based on global configuration
Servomotor* Servomotor_TestModeConvenienceWrapper();

// Macro to help with test reporting
#define TEST_RESULT(name, condition) \
    do { \
        bool passed = (condition); \
        TestRunner::addResult(name, passed, passed ? "" : "Test condition failed"); \
        printf("%s: %s\n", name, passed ? "PASS" : "FAIL"); \
    } while(0)
