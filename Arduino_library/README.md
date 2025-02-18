# ServoMotor Arduino Library

A comprehensive Arduino library for controlling servomotors using UART communication. This library provides an extensive API for precise motor control, position tracking, and advanced motion features, with built-in unit conversion capabilities for intuitive usage.

## Overview

The ServoMotor library is designed to interface with Gearotons servomotors through UART communication. It offers precise control over motor movements, including position, velocity, and acceleration control, along with advanced features like motion profiling, safety limits, and diagnostic capabilities. The library includes sophisticated unit conversion functionality, allowing users to work with their preferred units of measurement while handling all internal conversions automatically.

### Supported Units

- **Position Units**: Shaft rotations, degrees, radians, encoder counts
- **Time Units**: Seconds, milliseconds, minutes
- **Velocity Units**: Rotations per second, RPM, degrees/second, radians/second, counts/second
- **Acceleration Units**: Rotations/s², RPM/s, degrees/s², radians/s², counts/s²
- **Current Units**: Milliamps, amps
- **Voltage Units**: Millivolts, volts
- **Temperature Units**: Celsius, Fahrenheit, Kelvin

## Features

- **Motion Control**
  - Trapezoidal motion profiles
  - Position control with duration
  - Velocity control
  - Acceleration control
  - Multi-move sequences
  - Safety limits enforcement

- **Motor Configuration**
  - Maximum velocity settings
  - Maximum acceleration settings
  - PID constants configuration
  - Motor current limits
  - Position deviation limits

- **Sensor Integration**
  - Hall sensor position tracking
  - External encoder support
  - Temperature monitoring
  - Supply voltage monitoring

- **Diagnostics & Debugging**
  - Comprehensive position reporting
  - Debug value access
  - Performance profiling
  - Hall sensor statistics
  - PID error monitoring

- **System Management**
  - Device detection and identification
  - Firmware upgrade support
  - Time synchronization
  - Emergency stop functionality
  - System reset capability

## Installation

1. Download the library
2. Import it into the Arduino IDE
3. Include the library in your project

## Basic Usage

```cpp
#include <ServoMotor.h>

// Initialize servo motor with device alias and serial port
ServoMotor motor(1, Serial1);

void setup() {
    // Open serial communication
    motor.openSerialPort();
    
    // Enable the motor
    motor.enableMosfets();
    
    // Set preferred units for intuitive control
    motor.setPositionUnit(PositionUnit::DEGREES);
    motor.setTimeUnit(TimeUnit::SECONDS);
    
    // Set basic parameters (in the specified units)
    motor.setMaximumVelocity(360);     // 360 degrees per second
    motor.setMaximumAcceleration(720); // 720 degrees per second²
}

void loop() {
    // Move to position with trapezoidal profile
    // Will rotate 90 degrees over 0.5 seconds
    motor.trapezoidMove(90, 0.5);
    
    // Get current position (will be returned in degrees)
    auto position = motor.getHallSensorPosition();
    
    // Wait before next movement
    delay(3000);
    
    // Example of different unit usage
    motor.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
    motor.trapezoidMove(0.25, 0.5);  // Rotate 1/4 turn in 0.5 seconds
}
```

The library handles all unit conversions internally, allowing you to work with familiar units while maintaining precise control over the motor.

## Advanced Features

### Safety Limits
```cpp
// Set position limits
motor.setSafetyLimits(-10000, 10000);  // Set lower and upper bounds
```

### Motion Profiling
```cpp
// Execute complex motion sequence
uint8_t moveList[] = {...};  // Define move sequence
motor.multiMove(3, MOVE_TYPES, moveList);
```

### Diagnostics
```cpp
// Get comprehensive position data
auto posData = motor.getComprehensivePosition();
// Access commanded position, hall sensor position, and external encoder position

// Monitor temperature
auto temp = motor.getTemperature();
```

## Development and Contributing

### Development Tools

The library includes several Python scripts for code generation and maintenance:

- `generate_unit_conversion_code.py`: Generates unit conversion code from a JSON configuration
- `generate_command_code.py`: Generates command interface code
- `create_context.py`: Creates development context
- `apply_changes.py`: Applies changes to the codebase

### Testing

The library includes an Arduino emulator (`ArduinoEmulator.h`) that allows testing on desktop machines without actual hardware. Test files include:
- `test_motor.cpp`: Tests basic motor functionality
- `test_unit_conversions.cpp`: Validates unit conversion accuracy

### Contributing

Contributions to improve the library are welcome. Please follow these steps:

1. Fork the repository
2. Create a feature branch
3. Make your changes:
   - Use the provided Python scripts for generating code
   - Test your changes using the emulator
   - Ensure all tests pass
4. Submit a pull request

### Adding New Units

To add new units or modify existing ones:

1. Modify the unit conversion JSON configuration
2. Run `generate_unit_conversion_code.py` to regenerate the conversion code
3. Add appropriate tests to validate the new units
4. Test thoroughly using the Arduino emulator

## License

This library is released under the MIT License. See the LICENSE file for details.

## Author

Tom Rodinger (tom.rodinger@alumni.utoronto.ca)

## Version History

- 0.1.0: Initial release
  - Basic motor control functionality
  - Position and velocity control
  - Safety features
  - Diagnostic capabilities