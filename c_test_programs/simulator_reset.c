/**
 * simulator_reset.c
 *
 * This file contains implementations for functions that reset static variables
 * in specific modules when running in the simulator environment.
 */

#include "simulator_reset.h"
#include <stdio.h>
#include <stdint.h>


#ifdef MOTOR_SIMULATION

// External declarations for module-specific reset functions
extern void motor_control_simulator_init(void);
extern void error_handling_simulator_init(void);
extern void RS485_simulator_init(void);
extern void main_simulation_init(void);

/**
 * Reset all modules
 * This function calls all the individual module reset functions
 */
void reset_all_modules(void)
{
    // Reset error handling first to ensure we can handle any errors during other resets
    error_handling_simulator_init();
    // Reset RS485 module
    RS485_simulator_init();
    // Now reset motor control
    motor_control_simulator_init();
    // Reset static variables in main.c
    main_simulation_init();
}

#endif // MOTOR_SIMULATION