#include "error_text.h"

void fatal_error(uint16_t error_code);

#ifdef MOTOR_SIMULATION
/**
 * Initialize all static variables in error_handling.c for simulator use
 * This function should be called when the simulator starts to ensure
 * all static variables are properly initialized
 */
void error_handling_simulator_init(void);
#endif
