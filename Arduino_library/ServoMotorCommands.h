#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#ifdef ARDUINO
#include <Arduino.h>
#endif

#include "Communication.h"
#include "Commands.h"
#include "DataTypes.h"
#include "Utils.h"

// Payload and Response Structures

// Structure for Trapezoid move command payload
typedef struct __attribute__((__packed__)) {
    int32_t displacement;
    uint32_t duration;
} trapezoidMovePayload;

// Structure for Set maximum velocity command payload
typedef struct __attribute__((__packed__)) {
    uint32_t maximumVelocity;
} setMaximumVelocityPayload;

// Structure for Go to position command payload
typedef struct __attribute__((__packed__)) {
    int32_t position;
    uint32_t duration;
} goToPositionPayload;

// Structure for Set maximum acceleration command payload
typedef struct __attribute__((__packed__)) {
    uint32_t maximumAcceleration;
} setMaximumAccelerationPayload;

// Structure for Capture hall sensor data command payload
typedef struct __attribute__((__packed__)) {
    uint8_t dataType;
} captureHallSensorDataPayload;

// Structure for Capture hall sensor data command response
typedef struct __attribute__((__packed__)) {
    uint8_t* data;
} captureHallSensorDataResponse;

// Structure for Get current time command response
typedef struct __attribute__((__packed__)) {
    uint64_t currentTime;
} getCurrentTimeResponse;

// Structure for Time sync command payload
typedef struct __attribute__((__packed__)) {
    uint64_t masterTime;
} timeSyncPayload;

// Structure for Time sync command response
typedef struct __attribute__((__packed__)) {
    int32_t timeError;
    uint16_t rccIcscr;
} timeSyncResponse;

// Structure for Homing command payload
typedef struct __attribute__((__packed__)) {
    int32_t maxDistance;
    uint32_t maxDuration;
} homingPayload;

// Structure for Get hall sensor position command response
typedef struct __attribute__((__packed__)) {
    int64_t hallSensorPosition;
} getHallSensorPositionResponse;

// Structure for Get status command response
typedef struct __attribute__((__packed__)) {
    uint8_t statusFlags;
    uint8_t fatalErrorCode;
} StatusResponse;

// Structure for Get update frequency command response
typedef struct __attribute__((__packed__)) {
    uint32_t updateFrequency;
} getUpdateFrequencyResponse;

// Structure for Move with acceleration command payload
typedef struct __attribute__((__packed__)) {
    int32_t acceleration;
    uint32_t timeSteps;
} moveWithAccelerationPayload;

// Structure for Detect devices command response
typedef struct __attribute__((__packed__)) {
    uint64_t uniqueId;
    uint8_t alias;
    uint32_t crc;
} detectDevicesResponse;

// Structure for Set device alias command payload
typedef struct __attribute__((__packed__)) {
    uint64_t uniqueId;
    uint8_t alias;
} setDeviceAliasPayload;

// Structure for Get product info command response
typedef struct __attribute__((__packed__)) {
    char productCode[8];
    uint8_t firmwareCompatibility;
    uint32_t hardwareVersion;
    uint32_t serialNumber;
    uint64_t uniqueId;
    uint32_t reserved;
} getProductInfoResponse;

// Structure for Firmware upgrade command payload
typedef struct __attribute__((__packed__)) {
    uint8_t firmwarePage[2058];
} firmwareUpgradePayload;

// Structure for Get product description command response
typedef struct __attribute__((__packed__)) {
    char productDescription[32];
} getProductDescriptionResponse;

// Structure for Get firmware version command response
typedef struct __attribute__((__packed__)) {
    uint32_t firmwareVersion;
} getFirmwareVersionResponse;

// Structure for Move with velocity command payload
typedef struct __attribute__((__packed__)) {
    int32_t velocity;
    uint32_t duration;
} moveWithVelocityPayload;

// Structure for Set maximum motor current command payload
typedef struct __attribute__((__packed__)) {
    uint16_t motorCurrent;
    uint16_t regenerationCurrent;
} setMaximumMotorCurrentPayload;

// Structure for Multi-move command payload
typedef struct __attribute__((__packed__)) {
    uint8_t moveCount;
    uint32_t moveTypes;
    uint8_t* moveList;
} multiMovePayload;

// Structure for Set safety limits command payload
typedef struct __attribute__((__packed__)) {
    int64_t lowerLimit;
    int64_t upperLimit;
} setSafetyLimitsPayload;

// Structure for Ping command payload
typedef struct __attribute__((__packed__)) {
    uint8_t pingData[10];
} pingPayload;

// Structure for Ping command response
typedef struct __attribute__((__packed__)) {
    uint8_t responsePayload[10];
} pingResponse;

// Structure for Control hall sensor statistics command payload
typedef struct __attribute__((__packed__)) {
    uint8_t control;
} controlHallSensorStatisticsPayload;

// Structure for Get hall sensor statistics command response
typedef struct __attribute__((__packed__)) {
    uint16_t maxHall1;
    uint16_t maxHall2;
    uint16_t maxHall3;
    uint16_t minHall1;
    uint16_t minHall2;
    uint16_t minHall3;
    uint64_t sumHall1;
    uint64_t sumHall2;
    uint64_t sumHall3;
    uint32_t measurementCount;
} getHallSensorStatisticsResponse;

// Structure for Read multipurpose buffer command response
typedef struct __attribute__((__packed__)) {
    uint8_t* bufferData;
} readMultipurposeBufferResponse;

// Structure for Test mode command payload
typedef struct __attribute__((__packed__)) {
    uint8_t testMode;
} testModePayload;

// Structure for Get comprehensive position command response
typedef struct __attribute__((__packed__)) {
    int64_t commandedPosition;
    int64_t hallSensorPosition;
    int32_t externalEncoderPosition;
} getComprehensivePositionResponse;

// Structure for Get supply voltage command response
typedef struct __attribute__((__packed__)) {
    uint16_t supplyVoltage;
} getSupplyVoltageResponse;

// Structure for Get max PID error command response
typedef struct __attribute__((__packed__)) {
    int32_t minPidError;
    int32_t maxPidError;
} getMaxPidErrorResponse;

// Structure for Vibrate command payload
typedef struct __attribute__((__packed__)) {
    uint8_t vibrationLevel;
} vibratePayload;

// Structure for Identify command payload
typedef struct __attribute__((__packed__)) {
    uint64_t uniqueId;
} identifyPayload;

// Structure for Get temperature command response
typedef struct __attribute__((__packed__)) {
    int16_t temperature;
} getTemperatureResponse;

// Structure for Set PID constants command payload
typedef struct __attribute__((__packed__)) {
    uint32_t kP;
    uint32_t kI;
    uint32_t kD;
} setPIDConstantsPayload;

// Structure for Set max allowable position deviation command payload
typedef struct __attribute__((__packed__)) {
    int64_t maxAllowablePositionDeviation;
} setMaxAllowablePositionDeviationPayload;

// Structure for Get debug values command response
typedef struct __attribute__((__packed__)) {
    int64_t maxAcceleration;
    int64_t maxVelocity;
    int64_t currentVelocity;
    int32_t measuredVelocity;
    uint32_t nTimeSteps;
    int64_t debugValue1;
    int64_t debugValue2;
    int64_t debugValue3;
    int64_t debugValue4;
    uint16_t allMotorControlCalculationsProfilerTime;
    uint16_t allMotorControlCalculationsProfilerMaxTime;
    uint16_t getSensorPositionProfilerTime;
    uint16_t getSensorPositionProfilerMaxTime;
    uint16_t computeVelocityProfilerTime;
    uint16_t computeVelocityProfilerMaxTime;
    uint16_t motorMovementCalculationsProfilerTime;
    uint16_t motorMovementCalculationsProfilerMaxTime;
    uint16_t motorPhaseCalculationsProfilerTime;
    uint16_t motorPhaseCalculationsProfilerMaxTime;
    uint16_t motorControlLoopPeriodProfilerTime;
    uint16_t motorControlLoopPeriodProfilerMaxTime;
    uint16_t hallSensor1Voltage;
    uint16_t hallSensor2Voltage;
    uint16_t hallSensor3Voltage;
    uint32_t commutationPositionOffset;
    uint8_t motorPhasesReversed;
    int32_t maxHallPositionDelta;
    int32_t minHallPositionDelta;
    int32_t averageHallPositionDelta;
    uint8_t motorPwmVoltage;
} getDebugValuesResponse;

class ServoMotor {
public:
    ServoMotor(uint8_t alias, HardwareSerial& serialPort);
    void setAlias(uint8_t new_alias);
    uint8_t getAlias();
    void openSerialPort();
    void disableMosfets();
    void enableMosfets();
    void trapezoidMove(int32_t displacement, uint32_t duration);
    void setMaximumVelocity(uint32_t maximumVelocity);
    void goToPosition(int32_t position, uint32_t duration);
    void setMaximumAcceleration(uint32_t maximumAcceleration);
    void startCalibration();
    captureHallSensorDataResponse captureHallSensorData(uint8_t dataType);
    void resetTime();
    getCurrentTimeResponse getCurrentTime();
    timeSyncResponse timeSync(uint64_t masterTime);
    uint8_t getNumberOfQueuedItems();
    void emergencyStop();
    void zeroPosition();
    void homing(int32_t maxDistance, uint32_t maxDuration);
    getHallSensorPositionResponse getHallSensorPosition();
    StatusResponse getStatus();
    void goToClosedLoop();
    getUpdateFrequencyResponse getUpdateFrequency();
    void moveWithAcceleration(int32_t acceleration, uint32_t timeSteps);
    detectDevicesResponse detectDevices();
    detectDevicesResponse detectDevicesGetAnotherResponse();
    void setDeviceAlias(uint64_t uniqueId, uint8_t alias);
    getProductInfoResponse getProductInfo();
    void firmwareUpgrade(uint8_t firmwarePage[2058]);
    getProductDescriptionResponse getProductDescription();
    getFirmwareVersionResponse getFirmwareVersion();
    void moveWithVelocity(int32_t velocity, uint32_t duration);
    void systemReset();
    void setMaximumMotorCurrent(uint16_t motorCurrent, uint16_t regenerationCurrent);
    void multiMove(uint8_t moveCount, uint32_t moveTypes, uint8_t* moveList);
    void setSafetyLimits(int64_t lowerLimit, int64_t upperLimit);
    pingResponse ping(uint8_t pingData[10]);
    void controlHallSensorStatistics(uint8_t control);
    getHallSensorStatisticsResponse getHallSensorStatistics();
    int64_t getPosition();
    readMultipurposeBufferResponse readMultipurposeBuffer();
    void testMode(uint8_t testMode);
    getComprehensivePositionResponse getComprehensivePosition();
    getSupplyVoltageResponse getSupplyVoltage();
    getMaxPidErrorResponse getMaxPidError();
    void vibrate(uint8_t vibrationLevel);
    void identify(uint64_t uniqueId);
    getTemperatureResponse getTemperature();
    void setPIDConstants(uint32_t kP, uint32_t kI, uint32_t kD);
    void setMaxAllowablePositionDeviation(int64_t maxAllowablePositionDeviation);
    getDebugValuesResponse getDebugValues();
    int getError() const;

private:
    uint8_t _alias;
    Communication _comm;
    int _errno;
};

#endif // SERVOMOTOR_H
