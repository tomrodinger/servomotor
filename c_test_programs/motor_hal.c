#include "motor_hal.h"
#include "stm32g0xx_hal.h"

// Internal state
static struct {
    uint16_t motorCurrent;
    uint16_t errorCode;
} gState = {0};

// Initialize the HAL layer
MotorHAL_StatusTypeDef MotorHAL_Init(void) {
    gState.motorCurrent = 0;
    gState.errorCode = 0;
    return MOTOR_HAL_OK;
}

// Motor control functions
void MotorHAL_SetCurrent(uint16_t current) {
    gState.motorCurrent = current;
}

void MotorHAL_SetPosition(double angleDeg) {
    // Position is controlled by the firmware through current_position_i96
    // This function is a no-op in the simulator
}

void MotorHAL_SetError(uint16_t error) {
    gState.errorCode = error;
}

// Query functions
uint16_t MotorHAL_GetCurrent(void) {
    return gState.motorCurrent;
}

// External declaration of get_motor_position from motor_control.c
extern int64_t get_motor_position(void);

double MotorHAL_GetPosition(void) {
    int64_t current_position_i64 = get_motor_position();
    return (double)current_position_i64 * (360.0 / COUNTS_PER_ROTATION);
}

uint16_t MotorHAL_GetError(void) {
    return gState.errorCode;
}
