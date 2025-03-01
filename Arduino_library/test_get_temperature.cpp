#include "ServomotorCommands.h"
#include "test_framework.h"

void setup() {
    Serial.begin(115200);  // For debug output
    Serial.println("test_get_temperature: BEGIN\n");

    // Create a ServoMotor instance
    ServoMotor motor('X', Serial1);  // Initialize with alias 'X' and Serial1 port

    // Test 1: Direct temperature reading in Celsius
    float celsius = motor.getTemperature();  // Default is Celsius
    char buf[100];
    snprintf(buf, sizeof(buf), "Temperature in Celsius: %.2f °C\n", celsius);
    Serial.print(buf);
    TEST_RESULT("Temperature Read", celsius > 20.0f && celsius < 100.0f);  // Basic sanity check

    // Test 2: Unit setting functionality
    motor.setTemperatureUnit(TemperatureUnit::FAHRENHEIT);
    float temp_f = motor.getTemperature();
    snprintf(buf, sizeof(buf), "Temperature in Fahrenheit: %.2f °F\n", temp_f);
    Serial.print(buf);
    // Verify F = (C * 9/5) + 32
    float expected_f = (celsius * 9.0f/5.0f) + 32.0f;
    TEST_RESULT("Fahrenheit Reading", approxEqual(temp_f, expected_f));

    motor.setTemperatureUnit(TemperatureUnit::KELVIN);
    float temp_k = motor.getTemperature();
    snprintf(buf, sizeof(buf), "Temperature in Kelvin: %.2f K\n", temp_k);
    Serial.print(buf);
    // Verify K = C + 273.15
    float expected_k = celsius + 273.15f;
    TEST_RESULT("Kelvin Reading", approxEqual(temp_k, expected_k));

    // Print test results
    TestRunner::printResults();

    // Exit with appropriate status
    exit(TestRunner::allTestsPassed() ? 0 : 1);
}

void loop() {
    // Not used in this test
}
