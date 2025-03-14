#ifndef AUTO_GENERATED_UNIT_CONVERSIONS_H
#define AUTO_GENERATED_UNIT_CONVERSIONS_H

// This file was auto-generated by generate_unit_conversion_code.py on Mar 11 2025 20:27:03
// Do not edit manually. If changes are needed, modify the generator program instead.

// Define the number of encoder counts per revolution
#define COUNTS_PER_REVOLUTION 3276800

// Conversion factors for all units
// Time conversion factors
#define CONVERSION_FACTOR_TIMESTEPS 1.000000000f
#define CONVERSION_FACTOR_SECONDS 31250.000000000f
#define CONVERSION_FACTOR_MILLISECONDS 31.250000000f
#define CONVERSION_FACTOR_MINUTES 1875000.000000000f

// Position conversion factors
#define CONVERSION_FACTOR_SHAFT_ROTATIONS 3276800.000000000f
#define CONVERSION_FACTOR_DEGREES 9102.222222222f
#define CONVERSION_FACTOR_RADIANS 521518.917523523f
#define CONVERSION_FACTOR_ENCODER_COUNTS 1.000000000f

// Velocity conversion factors
#define CONVERSION_FACTOR_ROTATIONS_PER_SECOND 109951162.777600005f
#define CONVERSION_FACTOR_RPM 1832519.379626667f
#define CONVERSION_FACTOR_DEGREES_PER_SECOND 305419.896604444f
#define CONVERSION_FACTOR_RADIANS_PER_SECOND 17499271.054756649f
#define CONVERSION_FACTOR_COUNTS_PER_SECOND 33.554432000f
#define CONVERSION_FACTOR_COUNTS_PER_TIMESTEP 109951162.777600005f

// Acceleration conversion factors
#define CONVERSION_FACTOR_ROTATIONS_PER_SECOND_SQUARED 56294.995342131f
#define CONVERSION_FACTOR_RPM_PER_SECOND 938.249922369f
#define CONVERSION_FACTOR_DEGREES_PER_SECOND_SQUARED 156.374987061f
#define CONVERSION_FACTOR_RADIANS_PER_SECOND_SQUARED 8959.626780035f
#define CONVERSION_FACTOR_COUNTS_PER_SECOND_SQUARED 0.017179869f
#define CONVERSION_FACTOR_COUNTS_PER_TIMESTEP_SQUARED 16777216.000000000f

// Current conversion factors
#define CONVERSION_FACTOR_INTERNAL_CURRENT_UNITS 1.000000000f
#define CONVERSION_FACTOR_MILLIAMPS 0.195000000f
#define CONVERSION_FACTOR_AMPS 195.000000000f

// Voltage conversion factors
#define CONVERSION_FACTOR_MILLIVOLTS 0.010000000f
#define CONVERSION_FACTOR_VOLTS 10.000000000f

// Temperature conversion factors
#define CONVERSION_FACTOR_CELSIUS 1.000000000f
#define CONVERSION_FACTOR_FAHRENHEIT 0.555555556f
#define CONVERSION_FACTOR_KELVIN 1.000000000f

// Conversion offsets for complex conversions
#define CONVERSION_OFFSET_FAHRENHEIT_TO_CELSIUS -32.000000000f
#define CONVERSION_OFFSET_CELSIUS_TO_FAHRENHEIT 32.000000000f
#define CONVERSION_OFFSET_KELVIN_TO_CELSIUS -273.150000000f
#define CONVERSION_OFFSET_CELSIUS_TO_KELVIN 273.150000000f

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
    INTERNAL_CURRENT_UNITS,
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
