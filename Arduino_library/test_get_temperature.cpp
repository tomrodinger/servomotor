#include "ServomotorCommands.h"

void setup() {
    Serial.begin(115200);  // For debug output

    // Create a ServoMotor instance
    ServoMotor motor('X', Serial1);  // Initialize with alias 'X' and Serial1 port

    // Method 1: Using convertTemperature
    printf("Method 1: Using convertTemperature:\n");
    float celsius = motor.getTemperature();  // Default is Celsius
    float fahrenheit = convertTemperature(celsius, TemperatureUnit::CELSIUS, TemperatureUnit::FAHRENHEIT);
    float kelvin = convertTemperature(celsius, TemperatureUnit::CELSIUS, TemperatureUnit::KELVIN);
    printf("  %.2f°C\n", celsius);
    printf("  %.2f°F\n", fahrenheit);
    printf("  %.2f K\n", kelvin);

    // Method 2: Using setTemperatureUnit
    printf("\nMethod 2: Using setTemperatureUnit:\n");
    motor.setTemperatureUnit(TemperatureUnit::CELSIUS);
    printf("  %.2f°C\n", motor.getTemperature());
    motor.setTemperatureUnit(TemperatureUnit::FAHRENHEIT);
    printf("  %.2f°F\n", motor.getTemperature());
    motor.setTemperatureUnit(TemperatureUnit::KELVIN);
    printf("  %.2f K\n", motor.getTemperature());

    // Exit after reading temperature
    exit(0);
}

void loop() {
    // Not used in this test
}
