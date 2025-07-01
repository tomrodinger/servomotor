#ifndef SRC_ERROR_HANDLING_H_
#define SRC_ERROR_HANDLING_H_

#include <stdint.h>
#include "error_text.h"

void if_fatal_error_then_respond(void);
void if_fatal_error_then_dont_respond(void);
void fatal_error(uint16_t error_code);

#ifdef MOTOR_SIMULATION
/**
 * Initialize all static variables in error_handling.c for simulator use
 * This function should be called when the simulator starts to ensure
 * all static variables are properly initialized
 */
void error_handling_simulator_init(void);
#endif

#endif /* SRC_ERROR_HANDLING_H_ */
