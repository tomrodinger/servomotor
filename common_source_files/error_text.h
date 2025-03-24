#ifndef SRC_ERROR_TEXT_H_
#define SRC_ERROR_TEXT_H_

#include <stdint.h>

typedef enum {
    ERROR_NONE = 0,
    ERROR_TIME_WENT_BACKWARDS = 1,
    ERROR_FLASH_UNLOCK_FAIL = 2,
    ERROR_FLASH_WRITE_FAIL = 3,
    ERROR_TOO_MANY_BYTES = 4,
    ERROR_COMMAND_OVERFLOW = 5,
    ERROR_COMMAND_TOO_LONG = 6,
    ERROR_NOT_IN_OPEN_LOOP = 7,
    ERROR_QUEUE_NOT_EMPTY = 8,
    ERROR_HALL_SENSOR = 9,
    ERROR_CALIBRATION_OVERFLOW = 10,
    ERROR_NOT_ENOUGH_MINIMA_OR_MAXIMA = 11,
    ERROR_VIBRATION_FOUR_STEP = 12,
    ERROR_NOT_IN_CLOSED_LOOP = 13,
    ERROR_OVERVOLTAGE = 14,
    ERROR_ACCEL_TOO_HIGH = 15,
    ERROR_VEL_TOO_HIGH = 16,
    ERROR_QUEUE_IS_FULL = 17,
    ERROR_RUN_OUT_OF_QUEUE_ITEMS = 18,
    ERROR_MOTOR_BUSY = 19,
    ERROR_POSITION_OUT_OF_RANGE = 20,        // TO BE DEPRECATED
    ERROR_HALL_POSITION_OUT_OF_RANGE = 21,        // TO BE DEPRECATED
    ERROR_CURRENT_SENSOR_FAILED = 22,
    ERROR_MAX_PWM_VOLTAGE_TOO_HIGH = 23,
    ERROR_MULTI_MOVE_MORE_THAN_32_MOVES = 24,
    ERROR_SAFETY_LIMIT_EXCEEDED = 25,
    ERROR_TURN_POINT_OUT_OF_SAFETY_ZONE = 26,
    ERROR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE = 27,
    ERROR_PREDICTED_VELOCITY_TOO_HIGH = 28,
    ERROR_DEBUG1 = 29,
    ERROR_CONTROL_LOOP_TOOK_TOO_LONG = 30,
    ERROR_INDEX_OUT_OF_RANGE = 31,
    ERROR_CANT_PULSE_WHEN_INTERVALS_ACTIVE = 32,
    ERROR_INVALID_RUN_MODE = 33,
    ERROR_PARAMETER_OUT_OF_RANGE = 34,
    ERROR_DISABLE_MOSFETS_FIRST = 35,
    ERROR_FRAMING = 36,
    ERROR_OVERRUN = 37,
    ERROR_NOISE = 38,
    ERROR_GO_TO_CLOSED_LOOP_FAILED = 39,
    ERROR_OVERHEAT = 40,
    ERROR_TEST_MODE_ACTIVE = 41,
    ERROR_POSITION_DISCREPANCY = 42,
    ERROR_OVERCURRENT = 43,
    ERROR_PWM_TOO_HIGH = 44,
    ERROR_POSITION_DEVIATION_TOO_LARGE = 45,
    ERROR_MOVE_TOO_FAR = 46,
    ERROR_HALL_POSITION_DELTA_TOO_LARGE = 47,
    ERROR_INVALID_FIRST_BYTE = 48
} error_code_t;

#define ERROR_TEXT_INITIALIZER \
    "\0"                                      /* ERROR_NONE */ \
    "time went backwards\0"                   /* ERROR_TIME_WENT_BACKWARDS */ \
    "flash unlock fail\0"                     /* ERROR_FLASH_UNLOCK_FAIL */ \
    "flash write fail\0"                      /* ERROR_FLASH_WRITE_FAIL */ \
    "unused\0"                                /* ERROR_THIS_IS_UNUSED_AND_CAN_BE_REPLACED */                    /* THIS IS NOW OBSOLETE AND CAN BE REPLACED */ \
    "command overflow\0"                      /* ERROR_COMMAND_OVERFLOW */ \
    "command too long\0"                      /* ERROR_COMMAND_TOO_LONG */ \
    "not in open loop\0"                      /* ERROR_NOT_IN_OPEN_LOOP */ \
    "queue not empty\0"                       /* ERROR_QUEUE_NOT_EMPTY */ \
    "hall sensor error\0"                     /* ERROR_HALL_SENSOR */ \
    "calibration overflow\0"                  /* ERROR_CALIBRATION_OVERFLOW */ \
    "not enough minima or maxima\0"           /* ERROR_NOT_ENOUGH_MINIMA_OR_MAXIMA */ \
    "vibration four step\0"                   /* ERROR_VIBRATION_FOUR_STEP */ \
    "not in closed loop\0"                    /* ERROR_NOT_IN_CLOSED_LOOP */ \
    "overvoltage\0"                           /* ERROR_OVERVOLTAGE */ \
    "accel too high\0"                        /* ERROR_ACCEL_TOO_HIGH */ \
    "vel too high\0"                          /* ERROR_VEL_TOO_HIGH */ \
    "queue is full\0"                         /* ERROR_QUEUE_IS_FULL */ \
    "run out of queue items\0"                /* ERROR_RUN_OUT_OF_QUEUE_ITEMS */ \
    "motor busy\0"                            /* ERROR_MOTOR_BUSY */ \
    "position out of range\0"                 /* ERROR_POSITION_OUT_OF_RANGE (TO BE DEPRECATED) */ \
    "hall position out of range\0"            /* ERROR_HALL_POSITION_OUT_OF_RANGE (TO BE DEPRECATED) */ \
    "current sensor failed\0"                 /* ERROR_CURRENT_SENSOR_FAILED */ \
    "max pwm voltage too high\0"              /* ERROR_MAX_PWM_VOLTAGE_TOO_HIGH */ \
    "multi-move more than 32 moves\0"         /* ERROR_MULTI_MOVE_MORE_THAN_32_MOVES */ \
    "safety limit exceeded\0"                 /* ERROR_SAFETY_LIMIT_EXCEEDED */ \
    "turn point out of safety zone\0"         /* ERROR_TURN_POINT_OUT_OF_SAFETY_ZONE */ \
    "predicted position out of safety zone\0" /* ERROR_PREDICTED_POSITION_OUT_OF_SAFETY_ZONE */ \
    "predicted velocity too high\0"           /* ERROR_PREDICTED_VELOCITY_TOO_HIGH */ \
    "debug1\0"                                /* ERROR_DEBUG1 */ \
    "control loop took too long\0"            /* ERROR_CONTROL_LOOP_TOOK_TOO_LONG */ \
    "index out of range\0"                    /* ERROR_INDEX_OUT_OF_RANGE */ \
    "can't pulse when intervals are active\0" /* ERROR_CANT_PULSE_WHEN_INTERVALS_ACTIVE */ \
    "invalid run mode\0"                      /* ERROR_INVALID_RUN_MODE */ \
    "parameter out of range\0"                /* ERROR_PARAMETER_OUT_OF_RANGE */ \
    "disable MOSFETs first\0"                 /* ERROR_DISABLE_MOSFETS_FIRST */ \
    "framing error\0"                         /* ERROR_FRAMING */ \
    "overrun error\0"                         /* ERROR_OVERRUN */ \
    "noise error\0"                           /* ERROR_NOISE */ \
    "go to closed loop failed\0"              /* ERROR_GO_TO_CLOSED_LOOP_FAILED */ \
    "overheat\0"                              /* ERROR_OVERHEAT */ \
    "test mode active\0"                      /* ERROR_TEST_MODE_ACTIVE */ \
    "position discrepancy\0"                  /* ERROR_POSITION_DISCREPANCY */ \
    "overcurrent\0"                           /* ERROR_OVERCURRENT */ \
    "PWM too high\0"                          /* ERROR_PWM_TOO_HIGH */ \
    "position deviation too large\0"          /* ERROR_POSITION_DEVIATION_TOO_LARGE */ \
    "move too far\0"                          /* ERROR_MOVE_TOO_FAR */ \
    "hall position delta too large\0"         /* ERROR_HALL_POSITION_DELTA_TOO_LARGE */ \
    "invalid first byte\0"                    /* ERROR_INVALID_FIRST_BYTE */ \
    "\0"                                      /* this marks the end of the error messages */

const char *get_error_text(uint16_t error_code);

#endif /* SRC_ERROR_TEXT_H_ */
