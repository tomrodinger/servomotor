#include "motor_hal.h"
#include "stm32g0xx_hal.h"

// Internal state
static struct {
    double currentAngleDeg;
    uint16_t motorCurrent;
    uint16_t errorCode;
} gState = {0};

// Initialize the HAL layer
MotorHAL_StatusTypeDef MotorHAL_Init(void) {
    // Initialize state
    gState.currentAngleDeg = 0.0;
    gState.motorCurrent = 0;
    gState.errorCode = 0;
    
    return MOTOR_HAL_OK;
}

// Motor control functions
void MotorHAL_SetCurrent(uint16_t current) {
    gState.motorCurrent = current;
}

void MotorHAL_SetPosition(double angleDeg) {
    gState.currentAngleDeg = angleDeg;
}

void MotorHAL_SetError(uint16_t error) {
    gState.errorCode = error;
}

// Query functions
uint16_t MotorHAL_GetCurrent(void) {
    return gState.motorCurrent;
}

double MotorHAL_GetPosition(void) {
    return gState.currentAngleDeg;
}

uint16_t MotorHAL_GetError(void) {
    return gState.errorCode;
}
