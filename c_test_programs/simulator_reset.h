/**
 * simulator_reset.h
 * 
 * This file contains declarations for functions that reset static variables
 * in specific modules when running in the simulator environment.
 */

#ifndef SIMULATOR_RESET_H
#define SIMULATOR_RESET_H

#ifdef MOTOR_SIMULATION

/**
 * Reset all static variables in the motor_control module
 */
void motor_control_reset(void);

/**
 * Reset all static variables in the error_handling module
 */
void error_handling_reset(void);

/**
 * Reset all static variables in other modules as needed
 * Add more reset functions here as needed
 */
// void other_module_reset(void);

/**
 * Reset all modules
 * This function calls all the individual module reset functions
 */
void reset_all_modules(void);

#endif // MOTOR_SIMULATION

#endif // SIMULATOR_RESET_H