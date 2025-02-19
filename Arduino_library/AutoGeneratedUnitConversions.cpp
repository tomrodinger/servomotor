// Auto-generated. Do not edit manually.

#include "AutoGeneratedUnitConversions.h"
#include <math.h>

float convertTime(float value, TimeUnit fromUnit, TimeUnit toUnit)
{
    // Convert from 'fromUnit' to reference unit
    float inRef = 0.0f;
    switch (fromUnit) {
      case TimeUnit::SECONDS:
        inRef = value * 31250.000000000;
        break;
      case TimeUnit::MILLISECONDS:
        inRef = value * 31.250000000;
        break;
      case TimeUnit::MINUTES:
        inRef = value * 1875000.000000000;
        break;
      case TimeUnit::TIMESTEPS:
        inRef = value;
        break;
    }

    // Convert from reference unit to 'toUnit'
    float outVal = 0.0f;
    switch (toUnit) {
      case TimeUnit::SECONDS:
        {
            float factor = 31250.000000000;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
      case TimeUnit::MILLISECONDS:
        {
            float factor = 31.250000000;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
      case TimeUnit::MINUTES:
        {
            float factor = 1875000.000000000;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
      case TimeUnit::TIMESTEPS:
        {
            float factor = 1.0;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
    }

    return outVal;
}

float convertPosition(float value, PositionUnit fromUnit, PositionUnit toUnit)
{
    // Convert from 'fromUnit' to reference unit
    float inRef = 0.0f;
    switch (fromUnit) {
      case PositionUnit::SHAFT_ROTATIONS:
        inRef = value * 3276800.000000000;
        break;
      case PositionUnit::DEGREES:
        inRef = value * 9102.222222222;
        break;
      case PositionUnit::RADIANS:
        inRef = value * 521518.917523523;
        break;
      case PositionUnit::ENCODER_COUNTS:
        inRef = value * 1.000000000;
        break;
    }

    // Convert from reference unit to 'toUnit'
    float outVal = 0.0f;
    switch (toUnit) {
      case PositionUnit::SHAFT_ROTATIONS:
        {
            float factor = 3276800.000000000;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
      case PositionUnit::DEGREES:
        {
            float factor = 9102.222222222;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
      case PositionUnit::RADIANS:
        {
            float factor = 521518.917523523;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
      case PositionUnit::ENCODER_COUNTS:
        {
            float factor = 1.000000000;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
    }

    return outVal;
}

float convertVelocity(float value, VelocityUnit fromUnit, VelocityUnit toUnit)
{
    // Convert from 'fromUnit' to reference unit
    float inRef = 0.0f;
    switch (fromUnit) {
      case VelocityUnit::ROTATIONS_PER_SECOND:
        inRef = value * 3276800.000000000;
        break;
      case VelocityUnit::RPM:
        inRef = value * 54613.333333333;
        break;
      case VelocityUnit::DEGREES_PER_SECOND:
        inRef = value * 9102.222222222;
        break;
      case VelocityUnit::RADIANS_PER_SECOND:
        inRef = value * 521518.917523523;
        break;
      case VelocityUnit::COUNTS_PER_SECOND:
        inRef = value * 1.000000000;
        break;
    }

    // Convert from reference unit to 'toUnit'
    float outVal = 0.0f;
    switch (toUnit) {
      case VelocityUnit::ROTATIONS_PER_SECOND:
        {
            float factor = 3276800.000000000;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
      case VelocityUnit::RPM:
        {
            float factor = 54613.333333333;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
      case VelocityUnit::DEGREES_PER_SECOND:
        {
            float factor = 9102.222222222;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
      case VelocityUnit::RADIANS_PER_SECOND:
        {
            float factor = 521518.917523523;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
      case VelocityUnit::COUNTS_PER_SECOND:
        {
            float factor = 1.000000000;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
    }

    return outVal;
}

float convertAcceleration(float value, AccelerationUnit fromUnit, AccelerationUnit toUnit)
{
    // Convert from 'fromUnit' to reference unit
    float inRef = 0.0f;
    switch (fromUnit) {
      case AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED:
        inRef = value * 3276800.000000000;
        break;
      case AccelerationUnit::RPM_PER_SECOND:
        inRef = value * 54613.333333333;
        break;
      case AccelerationUnit::DEGREES_PER_SECOND_SQUARED:
        inRef = value * 9102.222222222;
        break;
      case AccelerationUnit::RADIANS_PER_SECOND_SQUARED:
        inRef = value * 521518.917523523;
        break;
      case AccelerationUnit::COUNTS_PER_SECOND_SQUARED:
        inRef = value * 1.000000000;
        break;
    }

    // Convert from reference unit to 'toUnit'
    float outVal = 0.0f;
    switch (toUnit) {
      case AccelerationUnit::ROTATIONS_PER_SECOND_SQUARED:
        {
            float factor = 3276800.000000000;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
      case AccelerationUnit::RPM_PER_SECOND:
        {
            float factor = 54613.333333333;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
      case AccelerationUnit::DEGREES_PER_SECOND_SQUARED:
        {
            float factor = 9102.222222222;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
      case AccelerationUnit::RADIANS_PER_SECOND_SQUARED:
        {
            float factor = 521518.917523523;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
      case AccelerationUnit::COUNTS_PER_SECOND_SQUARED:
        {
            float factor = 1.000000000;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
    }

    return outVal;
}

float convertCurrent(float value, CurrentUnit fromUnit, CurrentUnit toUnit)
{
    // Convert from 'fromUnit' to reference unit
    float inRef = 0.0f;
    switch (fromUnit) {
      case CurrentUnit::MILLIAMPS:
        inRef = value * 1.000000000;
        break;
      case CurrentUnit::AMPS:
        inRef = value * 1.000000000;
        break;
    }

    // Convert from reference unit to 'toUnit'
    float outVal = 0.0f;
    switch (toUnit) {
      case CurrentUnit::MILLIAMPS:
        {
            float factor = 1.000000000;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
      case CurrentUnit::AMPS:
        {
            float factor = 1.000000000;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
    }

    return outVal;
}

float convertVoltage(float value, VoltageUnit fromUnit, VoltageUnit toUnit)
{
    // Convert from 'fromUnit' to reference unit
    float inRef = 0.0f;
    switch (fromUnit) {
      case VoltageUnit::MILLIVOLTS:
        inRef = value * 1.000000000;
        break;
      case VoltageUnit::VOLTS:
        inRef = value * 1.000000000;
        break;
    }

    // Convert from reference unit to 'toUnit'
    float outVal = 0.0f;
    switch (toUnit) {
      case VoltageUnit::MILLIVOLTS:
        {
            float factor = 1.000000000;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
      case VoltageUnit::VOLTS:
        {
            float factor = 1.000000000;
            // Avoid divide-by-zero
            if (fabs(factor) < 1e-15) {
                outVal = 0.0f;
            } else {
                outVal = inRef / factor;
            }
        }
        break;
    }

    return outVal;
}

float convertTemperature(float value, TemperatureUnit fromUnit, TemperatureUnit toUnit)
{
    // Temperature conversions: celsius <-> fahrenheit <-> kelvin

    // Convert to celsius first
    float inCelsius = value;
    switch (fromUnit) {
      case TemperatureUnit::CELSIUS:
        inCelsius = value;
        break;
      case TemperatureUnit::FAHRENHEIT:
        inCelsius = (value - 32.0f) * (5.0f/9.0f);
        break;
      case TemperatureUnit::KELVIN:
        inCelsius = value - 273.15f;
        break;
    }

    float outValue = inCelsius;
    switch (toUnit) {
      case TemperatureUnit::CELSIUS:
        outValue = inCelsius;
        break;
      case TemperatureUnit::FAHRENHEIT:
        outValue = (inCelsius * 9.0f/5.0f) + 32.0f;
        break;
      case TemperatureUnit::KELVIN:
        outValue = inCelsius + 273.15f;
        break;
    }

    return outValue;
}

