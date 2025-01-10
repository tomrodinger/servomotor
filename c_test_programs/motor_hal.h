#ifndef MOTOR_HAL_H
#define MOTOR_HAL_H

#include <stdint.h>

// Status type for HAL operations
typedef enum {
    MOTOR_HAL_OK = 0,
    MOTOR_HAL_ERROR = 1
} MotorHAL_StatusTypeDef;

// Initialize the HAL layer
MotorHAL_StatusTypeDef MotorHAL_Init(void);

// Motor control functions
void MotorHAL_SetCurrent(uint16_t current);
void MotorHAL_SetPosition(double angleDeg);
void MotorHAL_SetError(uint16_t error);

// Query functions
uint16_t MotorHAL_GetCurrent(void);
double MotorHAL_GetPosition(void);
uint16_t MotorHAL_GetError(void);

#endif // MOTOR_HAL_H
