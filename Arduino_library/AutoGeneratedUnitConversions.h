#ifndef AUTO_GENERATED_UNIT_CONVERSIONS_H
#define AUTO_GENERATED_UNIT_CONVERSIONS_H

// This file is auto-generated. Do not edit manually.

enum class TimeUnit {
    SECONDS,
    MILLISECONDS,
    MINUTES,
    TIMESTEPS
};

float convertTime(float value, TimeUnit fromUnit, TimeUnit toUnit);

enum class PositionUnit {
    SHAFT_ROTATIONS,
    DEGREES,
    RADIANS,
    ENCODER_COUNTS
};

float convertPosition(float value, PositionUnit fromUnit, PositionUnit toUnit);

enum class VelocityUnit {
    ROTATIONS_PER_SECOND,
    RPM,
    DEGREES_PER_SECOND,
    RADIANS_PER_SECOND,
    COUNTS_PER_SECOND
};

float convertVelocity(float value, VelocityUnit fromUnit, VelocityUnit toUnit);

enum class AccelerationUnit {
    ROTATIONS_PER_SECOND_SQUARED,
    RPM_PER_SECOND,
    DEGREES_PER_SECOND_SQUARED,
    RADIANS_PER_SECOND_SQUARED,
    COUNTS_PER_SECOND_SQUARED
};

float convertAcceleration(float value, AccelerationUnit fromUnit, AccelerationUnit toUnit);

enum class CurrentUnit {
    MILLIAMPS,
    AMPS
};

float convertCurrent(float value, CurrentUnit fromUnit, CurrentUnit toUnit);

enum class VoltageUnit {
    MILLIVOLTS,
    VOLTS
};

float convertVoltage(float value, VoltageUnit fromUnit, VoltageUnit toUnit);

enum class TemperatureUnit {
    CELSIUS,
    FAHRENHEIT,
    KELVIN
};

float convertTemperature(float value, TemperatureUnit fromUnit, TemperatureUnit toUnit);

#endif // AUTO_GENERATED_UNIT_CONVERSIONS_H
