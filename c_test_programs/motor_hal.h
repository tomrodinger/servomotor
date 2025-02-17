#ifndef MOTOR_HAL_H
#define MOTOR_HAL_H

#include <stdint.h>
#include "motor_control.h"

#ifdef PRODUCT_NAME_M1
#include "commutation_table_M1.h"
#endif
#ifdef PRODUCT_NAME_M2
#include "commutation_table_M2.h"
#endif
#ifdef PRODUCT_NAME_M3
#include "commutation_table_M3.h"
#endif
#ifdef PRODUCT_NAME_M4
#include "commutation_table_M4.h"
#endif

// Constants
#define COUNTS_PER_ROTATION (N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS * ONE_REVOLUTION_ELECTRICAL_CYCLES)

// Types
typedef struct {
    uint32_t low;
    uint32_t mid;
    int32_t high;
} int96_t;

// External declarations
extern int96_t current_position_i96;

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
