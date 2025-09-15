# Servomotor Arduino API Documentation

Generated: 2025-09-15 12:21:15

## Latest Firmware Versions

At the time of generating this API reference, the latest released firmware versions for the servomotors are:

- **Model M17**: `servomotor_M17_fw0.13.0.0_scc3_hw1.3.firmware`


If you are experiencing problems, you can try to set the firmware of your product to this version and try again.

## Table of Contents

1. [Getting Started](#getting-started)
2. [Data Types](#data-types)
3. [Command Reference](#command-reference)
4. [Basic Control](#basic-control)
5. [Configuration](#configuration)
6. [Device Management](#device-management)
7. [Motion Control](#motion-control)
8. [Other](#other)
9. [Status & Monitoring](#status-and-monitoring)
10. [Error Handling](#error-handling)
11. [Error Codes](#error-codes)

## Getting Started

This section provides a complete example showing how to control a servomotor with Arduino.

### Trapezoid Move Example

```cpp
// Minimal Arduino example: Trapezoid move using built‑in unit conversions
// Goal: spin the motor exactly 1 rotation in 1 second, then stop.
// Sequence:
//  enable MOSFETs -> trapezoidMove(1.0 rotations, 1.0 seconds) -> wait 1.1s -> disable MOSFETs.
//
// Notes:
// - This uses the library's unit conversion (no raw counts/timesteps).
// - Configure Serial1 pins for your board (ESP32 example pins below).
// - Motor is created AFTER Serial1.begin(...) so hardware UART pins are set first.

#include <Servomotor.h>

#define ALIAS 'X'                   // Device alias
#define BAUD 230400                 // RS485 UART baud rate
#define DISPLACEMENT_ROTATIONS 1.0f // 1 rotation
#define DURATION_SECONDS 1.0f       // 1 second
#define TOLERANCE_PERCENT 10        // +10% wait margin because the motor's clock is not
                                    //  perfectly accurate
#define WAIT_MS ((unsigned long)(DURATION_SECONDS * 1000.0f * (100 + TOLERANCE_PERCENT) / 100))

// Example RS485 pins for ESP32 DevKit (change as needed for your board)
#if defined(ESP32)
#define RS485_TXD 4              // TX pin to RS485 transceiver
#define RS485_RXD 5              // RX pin from RS485 transceiver
#endif

void setup() {
  Serial.begin(115200); // Console serial for debugging

  // Create the motor; serial port opens on first instantiation.
#if defined(ESP32)
  Servomotor motor(ALIAS, Serial1, RS485_RXD, RS485_TXD);
#else
  Servomotor motor(ALIAS, Serial1);
#endif

  // Use units: rotations for position, seconds for time
  motor.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
  motor.setTimeUnit(TimeUnit::SECONDS);

  motor.enableMosfets();
  motor.trapezoidMove(DISPLACEMENT_ROTATIONS, DURATION_SECONDS);
  delay(WAIT_MS);
  motor.disableMosfets();
}

void loop() {
}
```

## Data Types

This section describes the various data types used in the Servomotor Arduino API.

### Integer Data Types

| Type | Size (bytes) | Range | Description |
|------|--------------|-------|-------------|
| i16 | 2 | -32,768 to 32,767 | 16-bit signed integer |
| i24 | 3 | -8,388,608 to 8,388,607 | 24-bit signed integer |
| i32 | 4 | -2,147,483,648 to 2,147,483,647 | 32-bit signed integer |
| i48 | 6 | -549,755,813,888 to 549,755,813,887 | 48-bit signed integer |
| i64 | 8 | -9,223,372,036,854,775,808 to 9,223,372,036,854,775,807 | 64-bit signed integer |
| i8 | 1 | -128 to 127 | 8-bit signed integer |
| u16 | 2 | 0 to 65,535 | 16-bit unsigned integer |
| u24 | 3 | 0 to 16,777,215 | 24-bit unsigned integer |
| u32 | 4 | 0 to 4,294,967,295 | 32-bit unsigned integer |
| u48 | 6 | 0 to 1,099,511,627,775 | 48-bit unsigned integer |
| u64 | 8 | 0 to 18,446,744,073,709,551,615 | 64-bit unsigned integer |
| u8 | 1 | 0 to 255 | 8-bit unsigned integer |

### Special Data Types

| Type | Size (bytes) | Description |
|------|--------------|-------------|
| buf10 | 10 | 10 byte long buffer containing any binary data |
| crc32 | 4 | 32-bit CRC |
| firmware_page | 2058 | This is the data to upgrade one page of flash memory. Contents includes the product model code (8 bytes), firmware compatibility code (1 byte), page number (1 byte), and the page data itself (2048 bytes). |
| general_data | Variable | This is some data. You will need to look elsewhere at some documentation or into the source code to find out what this data is. |
| list_2d | Variable | A two dimensional list in a Python style format, for example: [[1, 2], [3, 4]] |
| string8 | 8 | 8 byte long string with null termination if it is shorter than 8 bytes |
| string_null_term | Variable | This is a string with a variable length and must be null terminated |
| success_response | Variable | Indicates that the command was received successfully and is being executed. The next command can be immediately transmitted without causing a command overflow situation. |
| u24_version_number | 3 | 3 byte version number. the order is patch, minor, major |
| u32_version_number | 4 | 4 byte version number. the order is development number, patch, minor, major |
| u64_unique_id | 8 | The unique ID of the device (8-bytes long) |
| u8_alias | 1 | This can hold an ASCII character where the value is represented as an ASCII character if it is in the range 33 to 126, otherwise it is represented as a number from 0 to 255 |
| unknown_data | Variable | This is an unknown data type (work in progress; will be corrected and documented later) |

## Command Reference

This section documents all available commands organized by category.

### Basic Control

#### Disable MOSFETs

**Description:** Disables the MOSFETS (note that MOSFETs are disabled after initial power on).

**Example:**
```cpp
// Disable MOSFETs
motor.disableMosfets();
```

#### Enable MOSFETs

**Description:** Enables the MOSFETS.

**Example:**
```cpp
// Enable MOSFETs
motor.enableMosfets();
```

#### Reset time

**Description:** Resets the absolute time to zero (call this first before issuing any movement commands)

**Example:**
```cpp
// Reset time
motor.resetTime();
```

#### Emergency stop

**Description:** Emergency stop (stop all movement, disable MOSFETS, clear the queue)

**Example:**
```cpp
// Emergency stop
motor.emergencyStop();
```

#### Zero position

**Description:** Make the current position the position zero (origin)

**Example:**
```cpp
// Zero position
motor.zeroPosition();
```

#### System reset

**Description:** System reset / go to the bootloader. The motor will reset immediately and will enter the bootloader. If there is no command sent within a short time, the motor will exit the bootloader and run the application from the beginning.

**Example:**
```cpp
// System reset
motor.systemReset();
```

### Configuration

#### Set maximum velocity

**Description:** Sets maximum velocity (this is not used at this time)

**Parameters:**
- `maximumVelocity`: u32: Maximum velocity.

**Example:**
```cpp
// Set maximum velocity
uint32_t maximumVelocity = 1000;

motor.setMaximumVelocity(maximumVelocity);
```

#### Set maximum acceleration

**Description:** Sets max acceleration

**Parameters:**
- `maximumAcceleration`: u32: The maximum acceleration.

**Example:**
```cpp
// Set maximum acceleration
uint32_t maximumAcceleration = 1000;

motor.setMaximumAcceleration(maximumAcceleration);
```

#### Start calibration

**Description:** Starts a calibration, which will determine the average values of the hall sensors and will determine if they are working correctly

**Example:**
```cpp
// Start calibration
motor.startCalibration();
```

#### Set maximum motor current

**Description:** Set the maximum motor current and maximum regeneration current. The values are stored in non-volatile memory and survive a reset.

**Parameters:**
- `motorCurrent`: u16: The motor current. The units are some arbitrary units and not amps. A value of 150 or 200 is suitable.
- `regenerationCurrent`: u16: The motor regeneration current (while it is braking). This parameter is currently not used for anything.

**Example:**
```cpp
// Set maximum motor current
uint16_t motorCurrent = 100;
uint16_t regenerationCurrent = 100;

motor.setMaximumMotorCurrent(motorCurrent, regenerationCurrent);
```

#### Set safety limits

**Description:** Set safety limits (to prevent motion from exceeding set bounds)

**Parameters:**
- `lowerLimit`: i64: The lower limit in microsteps.
- `upperLimit`: i64: The upper limit in microsteps.

**Example:**
```cpp
// Set safety limits
float lowerLimit = 0;
float upperLimit = 0;

motor.setSafetyLimits(lowerLimit, upperLimit);
```

#### Test mode

**Description:** Set or trigger a certain test mode. This is a bit undocumented at the moment. Don't use this unless you are a developer working on test cases.

**Parameters:**
- `testMode`: u8: The test mode to use or trigger

**Example:**
```cpp
// Test mode
uint8_t testMode = 1;

motor.testMode(testMode);
```

#### Set PID constants

**Description:** Set PID constants for the control loop that will try to maintain the motion trajectory.

**Parameters:**
- `kP`: u32: The proportional term constant (P)
- `kI`: u32: The integral term constant (I)
- `kD`: u32: The differential term constant (D)

**Example:**
```cpp
// Set PID constants
uint32_t kP = 1000;
uint32_t kI = 1000;
uint32_t kD = 1000;

motor.setPidConstants(kP, kI, kD);
```

#### Set max allowable position deviation

**Description:** Set the amount of microsteps that the actual motor position (as measured by the hall sensors) is allowed to deviate from the desired position. Throw a fatal error if this is exceeded.

**Parameters:**
- `maxAllowablePositionDeviation`: i64: The new maximum allowable position deviation setting

**Example:**
```cpp
// Set max allowable position deviation
float maxAllowablePositionDeviation = 0;

motor.setMaxAllowablePositionDeviation(maxAllowablePositionDeviation);
```

#### CRC32 control

**Description:** Enable or disable CRC32 checking for commands

**Parameters:**
- `enableCrc32`: u8: Control value (1 to enable, 0 to disable CRC32 checking)

**Example:**
```cpp
// CRC32 control
uint8_t enableCrc32 = 1;

motor.crc32Control(enableCrc32);
```

### Device Management

#### Time sync

**Description:** Sends the master time to the motor so that it can sync its own clock (do this 10 times per second).

**Parameters:**
- `masterTime`: u48: The motor absolute time that the motor should sync to (in microseconds).

**Returns:**
- `timeError`: i32: The error in the motor's time compared to the master time.
- `rccIcscr`: u16: The contents of the RCC-ICSCR register (holds the HSICAL and HSITRIM settings).

**Example:**
```cpp
// Time sync
float masterTime = 0;

timeSyncResponse response = motor.timeSync(masterTime);
```

#### Get product specs

**Description:** Get the update frequency (reciprocal of the time step)

**Returns:**
- `updateFrequency`: u32: Update frequency in Hz. This is how often the motor executes all calculations for hall sensor position, movement, PID loop, safety, etc.
- `countsPerRotation`: u32: Counts per rotation. When commanding the motor or when reading back position, this is the number of counts per one shaft rotation.

**Example:**
```cpp
// Get product specs
getProductSpecsResponse response = motor.getProductSpecs();
```

#### Detect devices

**Description:** Detect all of the devices that are connected on the RS485 interface. Devices will identify themselves at a random time within one seconde. Chance of collision is possible but unlikely. You can repeat this if you suspect a collision (like if you have devices connected but they were not discovered within one to two seconds).

**Returns:**
- `uniqueId`: u64_unique_id: A unique ID (unique among all devices manufactured). The response is sent after a random delay of between 0 and 1 seconds.
- `alias`: u8_alias: The alias of the device that has this unique ID.

**Example:**
```cpp
// Detect devices
detectDevicesResponse response = motor.detectDevices();
```

#### Set device alias

**Description:** Sets device alias

**Parameters:**
- `alias`: u8_alias: The alias (which is a one byte ID) ranging from 0 to 251. It cannot be 252 to 254 because those are reserved. You can set it to 255, which will remove the alias.

**Example:**
```cpp
// Set device alias
uint8_t alias = 1;

motor.setDeviceAlias(alias);
```

#### Get product info

**Description:** Get product information

**Returns:**
- `productCode`: string8: The product code / model number (when doing a firmware upgrade, this must match between the firmware file and the target device).
- `firmwareCompatibility`: u8: A firmware compatibility code (when doing a firmware upgrade, this must match between the firmware file and the target device).
- `hardwareVersion`: u24_version_number: The hardware version stored as 3 bytes. The first byte is the patch version, followed by the minor and major versions.
- `serialNumber`: u32: The serial number.
- `uniqueId`: u64_unique_id: The unique ID for the product.
- `reserved`: u32: Not currently used.

**Example:**
```cpp
// Get product info
getProductInfoResponse response = motor.getProductInfo();
```

#### Firmware upgrade

**Description:** This command will upgrade the flash memory of the servo motor. Before issuing a firmware upgrade command, you must do some calculations as shown in the examples.

**Parameters:**
- `firmwarePage`: firmware_page: The data to upgrade one page of flash memory. Contents includes the product model code (8 bytes), firmware compatibility code (1 byte), page number (1 byte), and the page data itself (2048 bytes).

**Example:**
```cpp
// Firmware upgrade
float firmwarePage = 0;

motor.firmwareUpgrade(firmwarePage);
```

#### Get product description

**Description:** Get the product description.

**Returns:**
- `productDescription`: string_null_term: This is a brief description of the product.

**Example:**
```cpp
// Get product description
getProductDescriptionResponse response = motor.getProductDescription();
```

#### Get firmware version

**Description:** Get the firmware version or the bootloader version depending on what mode we are in. This command also returns the status bits, where the least significan bit teels us if we are currently in the bootloader (=1) or the main firmware (=0)

**Returns:**
- `firmwareVersion`: u32_version_number: The firmware version stored as 4 bytes. The first byte is the development number, then patch version, followed by the minor and major versions.
- `inBootloader`: u8: A flag that tells us if we are in the bootloader (=1) or in the reguslar firmware (=0)

**Example:**
```cpp
// Get firmware version
getFirmwareVersionResponse response = motor.getFirmwareVersion();
```

#### Ping

**Description:** Send a payload containing any data and the device will respond with the same data back

**Parameters:**
- `pingData`: buf10: Any binary data payload to send to the device.

**Returns:**
- `responsePayload`: buf10: The same data that was sent to the device will be returned if all went well.

**Example:**
```cpp
// Ping
float pingData = 0;

pingResponse response = motor.ping(pingData);
```

#### Vibrate

**Description:** Cause the motor to start to vary the voltage quickly and therefore to vibrate (or stop).

**Parameters:**
- `vibrationLevel`: u8: Vibration level (0 = turn off, 1 = turn on).

**Example:**
```cpp
// Vibrate
uint8_t vibrationLevel = 1;

motor.vibrate(vibrationLevel);
```

#### Identify

**Description:** Identify your motor by sending this command. The motor's green LED will flash rapidly for 3 seconds.

**Example:**
```cpp
// Identify
motor.identify();
```

### Motion Control

#### Trapezoid move

**Description:** Move immediately to the given position using the currently set speed (the speed is set by a separate command)

**Parameters:**
- `displacement`: i32: The displacement to travel. Can be positive or negative.
- `duration`: u32: The time over which to do the move.

**Example:**
```cpp
// Trapezoid move
int32_t displacement = 1000;
uint32_t duration = 1000;

motor.trapezoidMove(displacement, duration);
```

#### Go to position

**Description:** Move to this new given position in the amount of time specified. Acceleration and deceleration will be applied to make the move smooth.

**Parameters:**
- `position`: i32: New absolute position value.
- `duration`: u32: Time allowed for executing the move.

**Example:**
```cpp
// Go to position
int32_t position = 1000;
uint32_t duration = 1000;

motor.goToPosition(position, duration);
```

#### Homing

**Description:** Homing (or in other words, move until a crash and then stop immediately)

**Parameters:**
- `maxDistance`: i32: The maximum distance to move (if a crash does not occur). This can be positive or negative. the sign determines the direction of movement.
- `maxDuration`: u32: The maximum time to allow for homing. Make sure to give enough time for the motor to cover the maximum distance or the motor may move too fast or throw a fatal error.

**Example:**
```cpp
// Homing
int32_t maxDistance = 1000;
uint32_t maxDuration = 1000;

motor.homing(maxDistance, maxDuration);
```

#### Go to closed loop

**Description:** Go to closed loop position control mode

**Example:**
```cpp
// Go to closed loop
motor.goToClosedLoop();
```

#### Move with acceleration

**Description:** Rotates the motor with the specified acceleration

**Parameters:**
- `acceleration`: i32: The acceleration (the unit is microsteps per time step per time step * 2^24).
- `timeSteps`: u32: The number of time steps to apply this acceleration. Use command 18 to get the frequency of the time steps. After this many time steps, the acceleration will go to zero and velocity will be maintained.

**Example:**
```cpp
// Move with acceleration
int32_t acceleration = 1000;
uint32_t timeSteps = 1000;

motor.moveWithAcceleration(acceleration, timeSteps);
```

#### Move with velocity

**Description:** Rotates the motor with the specified velocity.

**Parameters:**
- `velocity`: i32: The velocity (the unit is microsteps per time step * 2^20).
- `duration`: u32: The time to maintain this velocity.

**Example:**
```cpp
// Move with velocity
int32_t velocity = 1000;
uint32_t duration = 1000;

motor.moveWithVelocity(velocity, duration);
```

#### Multimove

**Description:** The multimove command allows you to compose multiple moves one after another. Please note that when the queue becomes empty after all the moves are executed and the motor is not at a standstill then a fatal error will be triggered.

**Parameters:**
- `moveCount`: u8: Specify how many moves are being communicated in this one shot.
- `moveTypes`: u32: Each bit specifies if the move is a (bit = 0) MOVE_WITH_ACCELERATION_COMMAND or a (bit = 1) MOVE_WITH_VELOCITY_COMMAND.
- `moveList`: list_2d: A 2D list in Python format (list of lists). Each item in the list is of type [i32, u32] representing a series of move commands. Each move command specifies the acceleration to move at or the velocity to instantly change to (according to the bits above) and the number of time steps over which this command is to be executed. For example: '[[100, 30000], [-200, 60000]]'. There is a limit of 32 move commands that can be listed in this one multi-move command. Each of the moves takes up one queue spot, so make sure there is enough space in the queue to store all of the commands.

**Example:**
```cpp
// Multimove
uint8_t moveCount = 1;
uint32_t moveTypes = 1000;
uint32_t moveList = 1000;

motor.multimove(moveCount, moveTypes, moveList);
```

### Other

#### Capture hall sensor data

**Description:** Start sending hall sensor data (work in progress; don't send this command)

**Parameters:**
- `captureType`: u8: Indicates the type of data to capture. Currently 1 to 3 are valid.
- `nPointsToRead`: u32: Number of points to read back from the device
- `channelsToCaptureBitmask`: u8: Channels to capture bitmask. The first three bits are valid, which will turn on (0) or turn off (0) that hall sensor channel
- `timeStepsPerSample`: u16: Aquire a sample every this number of time steps. Time steps happen at the update frequency, which can be read with the Get product specs command
- `nSamplesToSum`: u16: Number of samples to sum together to make one point to transmit back
- `divisionFactor`: u16: Division factor to apply to the sum of the samples to scale it down before transmitting it so that it fits into the returned data type, which is a 16-bit number per each hall sensor

**Returns:**
- `data`: general_data: The data of the hall sensors after suming and averaging

**Example:**
```cpp
// Capture hall sensor data
uint8_t captureType = 1;
uint32_t nPointsToRead = 1000;
uint8_t channelsToCaptureBitmask = 1;
uint16_t timeStepsPerSample = 100;
uint16_t nSamplesToSum = 100;
uint16_t divisionFactor = 100;

captureHallSensorDataResponse response = motor.captureHallSensorData(captureType, nPointsToRead, channelsToCaptureBitmask, timeStepsPerSample, nSamplesToSum, divisionFactor);
```

#### Read multipurpose buffer

**Description:** Read whatever is in the multipurpose buffer (the buffer is used for data generated during calibration, going to closed loop mode, and when capturing hall sensor data)

**Returns:**
- `bufferData`: general_data: The data in the buffer (the format and length of the data depends on what was put in the buffer)

**Example:**
```cpp
// Read multipurpose buffer
readMultipurposeBufferResponse response = motor.readMultipurposeBuffer();
```

### Status & Monitoring

#### Get current time

**Description:** Gets the current absolute time

**Returns:**
- `currentTime`: u48: The current absolute time

**Example:**
```cpp
// Get current time
getCurrentTimeResponse response = motor.getCurrentTime();
```

#### Get n queued items

**Description:** Get the number of items currently in the movement queue (if this gets too large, don't queue any more movement commands)

**Returns:**
- `queueSize`: u8: The number of items in the movement queue. This command will return between 0 and 32. If less than 32, you can add more items to the queue to continue the movements in order without stopping.

**Example:**
```cpp
// Get n queued items
getNQueuedItemsResponse response = motor.getNQueuedItems();
```

#### Get hall sensor position

**Description:** Get the position as measured by the hall sensors (this should be the actual position of the motor and if everything is ok then it will be about the same as the desired position)

**Returns:**
- `hallSensorPosition`: i64: The current position as determined by the hall sensors

**Example:**
```cpp
// Get hall sensor position
getHallSensorPositionResponse response = motor.getHallSensorPosition();
```

#### Get status

**Description:** Gets the status of the motor

**Returns:**
- `statusFlags`: u16: A series of flags which are 1 bit each
- `fatalErrorCode`: u8: The fatal error code. If 0 then there is no fatal error. Once a fatal error happens, the motor becomes disabled and cannot do much anymore until reset. You can press the reset button on the motor or you can execute the System reset command to get out of the fatal error state.

**Example:**
```cpp
// Get status
getStatusResponse response = motor.getStatus();
```

#### Control hall sensor statistics

**Description:** Turn on or off the gathering of statistics for the hall sensors and reset the statistics

**Parameters:**
- `control`: u8: 0 = turn off statistics gathering, 1 = reset statistics and turn on gathering.

**Example:**
```cpp
// Control hall sensor statistics
uint8_t control = 1;

motor.controlHallSensorStatistics(control);
```

#### Get hall sensor statistics

**Description:** Read back the statistics gathered from the hall sensors. Useful for checking the hall sensor health and noise in the system.

**Returns:**
- `maxHall1`: u16: The maximum value of hall sensor 1 encoutered since the last statistics reset.
- `maxHall2`: u16: The maximum value of hall sensor 2 encoutered since the last statistics reset.
- `maxHall3`: u16: The maximum value of hall sensor 3 encoutered since the last statistics reset.
- `minHall1`: u16: The minimum value of hall sensor 1 encoutered since the last statistics reset.
- `minHall2`: u16: The minimum value of hall sensor 2 encoutered since the last statistics reset.
- `minHall3`: u16: The minimum value of hall sensor 3 encoutered since the last statistics reset.
- `sumHall1`: u64: The sum of hall sensor 1 values collected since the last statistics reset.
- `sumHall2`: u64: The sum of hall sensor 2 values collected since the last statistics reset.
- `sumHall3`: u64: The sum of hall sensor 3 values collected since the last statistics reset.
- `measurementCount`: u32: The number of times the hall sensors were measured since the last statistics reset.

**Example:**
```cpp
// Get hall sensor statistics
getHallSensorStatisticsResponse response = motor.getHallSensorStatistics();
```

#### Get position

**Description:** Get the current desired position (which may differ a bit from the actual position as measured by the hall sensors)

**Returns:**
- `position`: i64: The current desired position

**Example:**
```cpp
// Get position
getPositionResponse response = motor.getPosition();
```

#### Get comprehensive position

**Description:** Get the desired motor position, hall sensor position, and external encoder position all in one shot

**Returns:**
- `commandedPosition`: i64: The commanded position (which may differ from actual)
- `hallSensorPosition`: i64: The hall sensor position (or you could say the actual measured position)
- `externalEncoderPosition`: i32: The external encoder position. This needs special hardware attached to the motor to work

**Example:**
```cpp
// Get comprehensive position
getComprehensivePositionResponse response = motor.getComprehensivePosition();
```

#### Get supply voltage

**Description:** Get the measured voltage of the power supply.

**Returns:**
- `supplyVoltage`: u16: The voltage. Divide this number by 10 to get the actual voltage in volts.

**Example:**
```cpp
// Get supply voltage
getSupplyVoltageResponse response = motor.getSupplyVoltage();
```

#### Get max PID error

**Description:** Get the minimum and maximum error value ovserved in the PID control loop since the last read.

**Returns:**
- `minPidError`: i32: The minimum PID error value.
- `maxPidError`: i32: The maximum PID error value.

**Example:**
```cpp
// Get max PID error
getMaxPidErrorResponse response = motor.getMaxPidError();
```

#### Get temperature

**Description:** Get the measured temperature of the motor.

**Returns:**
- `temperature`: i16: The temperature in degrees celcius. The accuracy is about +/- 3 degrees celcius and is measured at the motor driver PCB.

**Example:**
```cpp
// Get temperature
getTemperatureResponse response = motor.getTemperature();
```

#### Get debug values

**Description:** Get debug values including motor control parameters, profiler times, hall sensor data, and other diagnostic information.

**Returns:**
- `maxAcceleration`: i64: Maximum acceleration setting
- `maxVelocity`: i64: Maximum velocity setting
- `currentVelocity`: i64: Current velocity
- `measuredVelocity`: i32: Measured velocity
- `nTimeSteps`: u32: Number of time steps left in the current move
- `debugValue1`: i64: Debug value 1
- `debugValue2`: i64: Debug value 2
- `debugValue3`: i64: Debug value 3
- `debugValue4`: i64: Debug value 4
- `allMotorControlCalculationsProfilerTime`: u16: All motor control calculations profiler time
- `allMotorControlCalculationsProfilerMaxTime`: u16: All motor control calculations profiler maximum time
- `getSensorPositionProfilerTime`: u16: Get sensor position profiler time
- `getSensorPositionProfilerMaxTime`: u16: Get sensor position profiler maximum time
- `computeVelocityProfilerTime`: u16: Compute velocity profiler time
- `computeVelocityProfilerMaxTime`: u16: Compute velocity profiler maximum time
- `motorMovementCalculationsProfilerTime`: u16: Motor movement calculations profiler time
- `motorMovementCalculationsProfilerMaxTime`: u16: Motor movement calculations profiler maximum time
- `motorPhaseCalculationsProfilerTime`: u16: Motor phase calculations profiler time
- `motorPhaseCalculationsProfilerMaxTime`: u16: Motor phase calculations profiler maximum time
- `motorControlLoopPeriodProfilerTime`: u16: Motor control loop period profiler time
- `motorControlLoopPeriodProfilerMaxTime`: u16: Motor control loop period profiler maximum time
- `hallSensor1Voltage`: u16: Hall sensor 1 voltage
- `hallSensor2Voltage`: u16: Hall sensor 2 voltage
- `hallSensor3Voltage`: u16: Hall sensor 3 voltage
- `commutationPositionOffset`: u32: Commutation position offset
- `motorPhasesReversed`: u8: Motor phases reversed flag
- `maxHallPositionDelta`: i32: Maximum hall position delta
- `minHallPositionDelta`: i32: Minimum hall position delta
- `averageHallPositionDelta`: i32: Average hall position delta
- `motorPwmVoltage`: u8: Motor PWM voltage

**Example:**
```cpp
// Get debug values
getDebugValuesResponse response = motor.getDebugValues();
```

#### Get communication statistics

**Description:** Get and optionally reset the CRC32 error counter

**Parameters:**
- `resetCounter`: u8: Reset flag (1 to reset the counter after reading, 0 to just read)

**Returns:**
- `crc32ErrorCount`: u32: Number of CRC32 errors detected
- `packetDecodeErrorCount`: u32: Number of packet decode errors detected
- `firstBitErrorCount`: u32: Number of times that the first bit in the first byte of a packet was not 1 as expected
- `framingErrorCount`: u32: Number of framing errors detected during reception from the RS485 interface
- `overrunErrorCount`: u32: Number of overrun errors detected during reception from the RS485 interface
- `noiseErrorCount`: u32: Number of noise errors detected during reception from the RS485 interface

**Example:**
```cpp
// Get communication statistics
uint8_t resetCounter = 1;

getCommunicationStatisticsResponse response = motor.getCommunicationStatistics(resetCounter);
```

## Error Handling

The servomotor has comprehensive error detection and handling. If an error condition is detected then a fatal error condition will result. When this happens, the motor will immediately disable itself and the red LED will flash. The flashing LED will indicate the error code. You can count the pulses. You can also retrieve the error using the "Get Status" command. Once you know the error code, you can look it up in the section below to understand the root reason. The servomotor will not respond to most commands when it is in a fatal error state, only "Get Status" and "System Reset". You will need to reset the servomotor to get it back into a functional state. With careful programming, a fatal error should not be triggered. Nearly in all cases, if a fatal error occurs, it is for a good reason and most likely you will need to improve the way you try to use the motor.

## Error Codes

This section lists all possible error codes that can be returned by the servomotor.

| Code | Enum | Description |
|------|------|-------------|
| 1 | ERROR_TIME_WENT_BACKWARDS | The system detected that time appeared to move backwards, which indicates a problem with the microsecond clock |
| 2 | ERROR_FLASH_UNLOCK_FAIL | Failed to unlock the flash memory for writing |
| 3 | ERROR_FLASH_WRITE_FAIL | Failed to write data to flash memory |
| 4 | ERROR_TOO_MANY_BYTES | Data size exceeds buffer or memory limits |
| 5 | ERROR_COMMAND_OVERFLOW | Command buffer overflow detected |
| 6 | ERROR_COMMAND_TOO_LONG | Individual command exceeds maximum allowed length |
| 7 | ERROR_NOT_IN_OPEN_LOOP | An operation that requires open loop control mode was attempted while the motor was in a different control mode |
| 8 | ERROR_QUEUE_NOT_EMPTY | An operation that requires an empty movement queue was attempted while the queue still had pending movements |
| 9 | ERROR_HALL_SENSOR | Hall sensor readings are invalid or out of expected range |
| 10 | ERROR_CALIBRATION_OVERFLOW | Calibration data buffer overflow during motor calibration |
| 11 | ERROR_NOT_ENOUGH_MINIMA_OR_MAXIMA | Insufficient number of peaks detected during hall sensor calibration |
| 12 | ERROR_VIBRATION_FOUR_STEP | Error in four-step vibration sequence during motor testing |
| 13 | ERROR_NOT_IN_CLOSED_LOOP | An operation that requires closed loop control mode was attempted while in a different mode |
| 14 | ERROR_OVERVOLTAGE | Motor supply voltage exceeds maximum allowed limit |
| 15 | ERROR_ACCEL_TOO_HIGH | The requested acceleration exceeds the maximum allowed acceleration limit |
| 16 | ERROR_VEL_TOO_HIGH | The requested velocity exceeds the maximum allowed velocity limit |
| 17 | ERROR_QUEUE_IS_FULL | Movement queue is full and cannot accept more commands |
| 18 | ERROR_RUN_OUT_OF_QUEUE_ITEMS | Movement queue unexpectedly empty during operation |
| 19 | ERROR_MOTOR_BUSY | An operation was attempted while the motor was busy with another operation like calibration or homing |
| 20 | ERROR_CAPTURE_PAYLOAD_TOO_BIG | The protocol used to communication over RS485 supports a total packet size that cannot be larger than 65535 bytes. The maximum payload size is just a bit less than this (about 65530 bytes). We have requested to capture hall sensor data that is larger than this maximum payload size. |
| 21 | ERROR_CAPTURE_OVERFLOW | The data could not be transmitted back over RS485 fast enough to keep up with the rate of capturing the data |
| 22 | ERROR_CURRENT_SENSOR_FAILED | The motor current sensor readings are outside expected baseline range |
| 23 | ERROR_MAX_PWM_VOLTAGE_TOO_HIGH | Configured maximum PWM voltage exceeds hardware limits |
| 24 | ERROR_MULTIMOVE_MORE_THAN_32_MOVES | Multi-move sequence exceeds maximum allowed number of moves |
| 25 | ERROR_SAFETY_LIMIT_EXCEEDED | The motor position has exceeded the configured safety limits |
| 26 | ERROR_TURN_POINT_OUT_OF_SAFETY_ZONE | Calculated movement turn point exceeds safety zone limits |
| 27 | ERROR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE | Movement trajectory would exceed safety zone limits |
| 28 | ERROR_PREDICTED_VELOCITY_TOO_HIGH | Movement would result in velocity exceeding limits |
| 29 | ERROR_DEBUG1 | Debug error used for development testing |
| 30 | ERROR_CONTROL_LOOP_TOOK_TOO_LONG | The motor control loop calculations exceeded the maximum allowed time |
| 31 | ERROR_INDEX_OUT_OF_RANGE | Array or buffer index exceeds valid range |
| 32 | ERROR_CANT_PULSE_WHEN_INTERVALS_ACTIVE | Pulse command attempted while interval timing active |
| 33 | ERROR_INVALID_RUN_MODE | Attempted to set invalid motor run mode |
| 34 | ERROR_PARAMETER_OUT_OF_RANGE | Configuration parameter exceeds valid range |
| 35 | ERROR_DISABLE_MOSFETS_FIRST | Operation requires MOSFETs to be disabled first |
| 36 | ERROR_FRAMING | Communication framing error detected |
| 37 | ERROR_OVERRUN | Communication buffer overrun detected |
| 38 | ERROR_NOISE | Communication noise detected |
| 39 | ERROR_GO_TO_CLOSED_LOOP_FAILED | Failed to transition from open loop to closed loop control mode |
| 40 | ERROR_OVERHEAT | Motor or controller temperature too high |
| 41 | ERROR_TEST_MODE_ACTIVE | Operation not allowed while test mode is active |
| 42 | ERROR_POSITION_DISCREPANCY | Mismatch between expected and actual position after movement |
| 43 | ERROR_OVERCURRENT | Motor current exceeds maximum allowed limit |
| 44 | ERROR_PWM_TOO_HIGH | The calculated PWM duty cycle exceeds the maximum allowed value |
| 45 | ERROR_POSITION_DEVIATION_TOO_LARGE | The difference between commanded position and actual position exceeds allowed limits |
| 46 | ERROR_MOVE_TOO_FAR | Requested movement distance exceeds system limits |
| 47 | ERROR_HALL_POSITION_DELTA_TOO_LARGE | The change in hall sensor position between updates is larger than expected |
| 48 | ERROR_INVALID_FIRST_BYTE | The received first byte does not have the least significant bit set to 1 |
| 49 | ERROR_CAPTURE_BAD_PARAMETERS | Some invalid parameters were given when invoking the Capture hall sensor data command |
| 50 | ERROR_BAD_ALIAS | You tried to set the device to one of the reserved aliases such as 254, 253, or 252 |
| 51 | ERROR_COMMAND_SIZE_WRONG | The payload sent with the command does not match the expected size of the input parameters |
| 52 | ERROR_INVALID_FLASH_PAGE | You are sending a firmware packet and the specified firmware page to be written is not valid and out of range |
| 53 | ERROR_INVALID_TEST_MODE | You tried to trigger a certain test mode but it is not a valid one |
