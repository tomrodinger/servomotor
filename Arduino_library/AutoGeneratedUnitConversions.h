#ifndef AUTO_GENERATED_UNIT_CONVERSIONS_H
#define AUTO_GENERATED_UNIT_CONVERSIONS_H

// This file is auto-generated. Do not edit manually.

// Enum to specify the direction of unit conversion
enum class ConversionDirection {
    TO_INTERNAL,    // Convert from user unit to internal unit
    FROM_INTERNAL   // Convert from internal unit to user unit
};

enum class TimeUnit {
    TIMESTEPS,
    SECONDS,
    MILLISECONDS,
    MINUTES
};

float convertTime(float value, TimeUnit unit, ConversionDirection direction);

enum class PositionUnit {
    SHAFT_ROTATIONS,
    DEGREES,
    RADIANS,
    ENCODER_COUNTS
};

float convertPosition(float value, PositionUnit unit, ConversionDirection direction);

enum class VelocityUnit {
    ROTATIONS_PER_SECOND,
    RPM,
    DEGREES_PER_SECOND,
    RADIANS_PER_SECOND,
    COUNTS_PER_SECOND,
    COUNTS_PER_TIMESTEP
};

float convertVelocity(float value, VelocityUnit unit, ConversionDirection direction);

enum class AccelerationUnit {
    ROTATIONS_PER_SECOND_SQUARED,
    RPM_PER_SECOND,
    DEGREES_PER_SECOND_SQUARED,
    RADIANS_PER_SECOND_SQUARED,
    COUNTS_PER_SECOND_SQUARED,
    COUNTS_PER_TIMESTEP_SQUARED
};

float convertAcceleration(float value, AccelerationUnit unit, ConversionDirection direction);

enum class CurrentUnit {
    ARBITRARY_UNITS,
    MILLIAMPS,
    AMPS
};

float convertCurrent(float value, CurrentUnit unit, ConversionDirection direction);

enum class VoltageUnit {
    MILLIVOLTS,
    VOLTS
};

float convertVoltage(float value, VoltageUnit unit, ConversionDirection direction);

enum class TemperatureUnit {
    CELSIUS,
    FAHRENHEIT,
    KELVIN
};

float convertTemperature(float value, TemperatureUnit unit, ConversionDirection direction);

#endif // AUTO_GENERATED_UNIT_CONVERSIONS_H
