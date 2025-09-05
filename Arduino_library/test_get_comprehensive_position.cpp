#include "Servomotor.h"
#include "test_framework.h"
#include <string>

void setup() {
    Serial.begin(115200);  // For debug output
    Serial.println("test_get_comprehensive_position: BEGIN\n");

    // Create a Servomotor instance using the wrapper
    Servomotor* motor = Servomotor_TestModeConvenienceWrapper();
    
    // Reset system to get to a known state
    motor->systemReset();
    delay(1500);  // Wait for system to reset
    
    // Enable motor for testing
    motor->enableMosfets();
    delay(100);
    
    // Reset time and zero position
    motor->resetTime();
    motor->zeroPosition();
    delay(100);
    
    // First, move the motor to a known position (1 rotation)
    Serial.println("\n---- Moving motor to test position ----");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor->setTimeUnit(TimeUnit::SECONDS);
    motor->setVelocityUnit(VelocityUnit::ROTATIONS_PER_SECOND);
    
    // Move 1 rotation over 2 seconds
    motor->trapezoidMove(1.0f, 2.0f);
    delay(2500);  // Wait for move to complete (2 seconds + buffer)
    
    // Test 1: Get comprehensive position in SHAFT_ROTATIONS
    Serial.println("\n---- Testing getComprehensivePosition with SHAFT_ROTATIONS unit ----");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    auto positions = motor->getComprehensivePosition();
    checkMotorError(*motor, "getComprehensivePosition - ROTATIONS");
    
    Serial.print("Commanded position in SHAFT_ROTATIONS: ");
    Serial.println(positions.commandedPosition);
    Serial.print("Hall sensor position in SHAFT_ROTATIONS: ");
    Serial.println(positions.hallSensorPosition);
    Serial.print("External encoder position in SHAFT_ROTATIONS: ");
    Serial.println(positions.externalEncoderPosition);
    
    TEST_RESULT("Commanded Position in SHAFT_ROTATIONS",
                approxEqual(positions.commandedPosition, 1.0f, 0.05f));
    TEST_RESULT("Hall Sensor Position in SHAFT_ROTATIONS",
                approxEqual(positions.hallSensorPosition, 1.0f, 0.05f));
    
    // Get raw position data for comparison
    auto rawPos = motor->getComprehensivePositionRaw();
    checkMotorError(*motor, "getComprehensivePositionRaw");
    Serial.print("Raw commanded position: "); Serial.println(std::to_string(rawPos.commandedPosition).c_str());
    Serial.print("Raw hall sensor position: "); Serial.println(std::to_string(rawPos.hallSensorPosition).c_str());
    Serial.print("Raw external encoder position: "); Serial.println(std::to_string(rawPos.externalEncoderPosition).c_str());
    
    // Verify the conversion is consistent with our helper functions
    float expected_commanded = convertPosition((float)rawPos.commandedPosition,
                                               PositionUnit::SHAFT_ROTATIONS,
                                               ConversionDirection::FROM_INTERNAL);
    TEST_RESULT("SHAFT_ROTATIONS Conversion Accurate (Commanded)",
                approxEqual(positions.commandedPosition, expected_commanded, 0.01f));
    
    // Test 2: Get comprehensive position in DEGREES
    Serial.println("\n---- Testing getComprehensivePosition with DEGREES unit ----");
    motor->setPositionUnit(PositionUnit::DEGREES);
    positions = motor->getComprehensivePosition();
    checkMotorError(*motor, "getComprehensivePosition - DEGREES");
    
    Serial.print("Commanded position in DEGREES: ");
    Serial.println(positions.commandedPosition);
    Serial.print("Hall sensor position in DEGREES: ");
    Serial.println(positions.hallSensorPosition);
    
    // 1 rotation = 360 degrees
    TEST_RESULT("Commanded Position in DEGREES",
                approxEqual(positions.commandedPosition, 360.0f, 5.0f));
    TEST_RESULT("Hall Sensor Position in DEGREES",
                approxEqual(positions.hallSensorPosition, 360.0f, 5.0f));
    
    // Verify conversion matches what we expect
    float expected_degrees_cmd = convertPosition((float)rawPos.commandedPosition,
                                                PositionUnit::DEGREES,
                                                ConversionDirection::FROM_INTERNAL);
    TEST_RESULT("DEGREES Conversion Correct (Commanded)",
                approxEqual(positions.commandedPosition, expected_degrees_cmd, 0.1f));
    
    // Test 3: Get comprehensive position in RADIANS
    Serial.println("\n---- Testing getComprehensivePosition with RADIANS unit ----");
    motor->setPositionUnit(PositionUnit::RADIANS);
    positions = motor->getComprehensivePosition();
    checkMotorError(*motor, "getComprehensivePosition - RADIANS");
    
    Serial.print("Commanded position in RADIANS: ");
    Serial.println(positions.commandedPosition);
    Serial.print("Hall sensor position in RADIANS: ");
    Serial.println(positions.hallSensorPosition);
    
    // 1 rotation = 2π radians ≈ 6.28 radians
    TEST_RESULT("Commanded Position in RADIANS",
                approxEqual(positions.commandedPosition, 6.28f, 0.1f));
    TEST_RESULT("Hall Sensor Position in RADIANS",
                approxEqual(positions.hallSensorPosition, 6.28f, 0.1f));
    
    // Verify conversion matches what we expect
    float expected_radians_cmd = convertPosition((float)rawPos.commandedPosition,
                                                PositionUnit::RADIANS,
                                                ConversionDirection::FROM_INTERNAL);
    TEST_RESULT("RADIANS Conversion Correct (Commanded)",
                approxEqual(positions.commandedPosition, expected_radians_cmd, 0.1f));
    
    // Test 4: Get comprehensive position in ENCODER_COUNTS
    Serial.println("\n---- Testing getComprehensivePosition with ENCODER_COUNTS unit ----");
    motor->setPositionUnit(PositionUnit::ENCODER_COUNTS);
    positions = motor->getComprehensivePosition();
    checkMotorError(*motor, "getComprehensivePosition - ENCODER_COUNTS");
    
    Serial.print("Commanded position in ENCODER_COUNTS: ");
    Serial.println(positions.commandedPosition);
    Serial.print("Hall sensor position in ENCODER_COUNTS: ");
    Serial.println(positions.hallSensorPosition);
    
    // Verify counts match the raw values
    TEST_RESULT("ENCODER_COUNTS Match Raw (Commanded)",
                approxEqual(positions.commandedPosition, (float)rawPos.commandedPosition, 300.0f));
    TEST_RESULT("ENCODER_COUNTS Match Raw (Hall Sensor)",
                approxEqual(positions.hallSensorPosition, (float)rawPos.hallSensorPosition, 300.0f));
    
    // Test 5: Compare with standard getPosition and getHallSensorPosition methods
    Serial.println("\n---- Comparing with standard position methods ----");
    motor->setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    float std_position = motor->getPosition();
    float hall_position = motor->getHallSensorPosition();
    positions = motor->getComprehensivePosition();
    
    Serial.print("Standard getPosition: "); Serial.println(std_position);
    Serial.print("Standard getHallSensorPosition: "); Serial.println(hall_position);
    Serial.print("getComprehensivePosition (commanded): "); Serial.println(positions.commandedPosition);
    Serial.print("getComprehensivePosition (hall): "); Serial.println(positions.hallSensorPosition);
    
    TEST_RESULT("getPosition matches getComprehensivePosition",
                approxEqual(std_position, positions.commandedPosition, 0.05f));
    TEST_RESULT("getHallSensorPosition matches getComprehensivePosition",
                approxEqual(hall_position, positions.hallSensorPosition, 0.05f));
    
    // Test consistent values between commanded and hall position
    TEST_RESULT("Commanded vs Hall position",
                approxEqual(positions.commandedPosition, positions.hallSensorPosition, 0.1f));
    
    // Print test results
    TestRunner::printResults();
    
    // Clean up
    delete motor;

    // Exit with appropriate status
    exit(TestRunner::allTestsPassed() ? 0 : 1);
}

void loop() {
    // Not used in this test
}