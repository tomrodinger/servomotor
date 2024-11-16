#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include "motor_control.h"
#include "mosfets.h"
#include "error_handling.h"
#include "stm32g0xx_hal.h"


#define MAX_MOTOR_PWM_VOLTAGE_DISCREPANCY 100

static double integral_term_double = 0;
static double previous_error_double = 0;
static double max_integral_term_double = 0;
static double min_PID_error_double = 2147483647;
static double max_PID_error_double = -2147483648;
static double max_error_change_double = 0;
static double max_error_double = 0;
static double proportional_constant_pid_double = 0;
static double integral_constant_pid_double = 0;
static double derivative_constant_pid_scaled_for_averaging_double = 0;
static double low_pass_filtered_error_change_double = 0;
static double max_motor_pwm_voltage_double = DEFAULT_MAX_MOTOR_PWM_VOLTAGE;
static double max_motor_regen_pwm_voltage_double = DEFAULT_MAX_MOTOR_PWM_VOLTAGE;
static double pid_p_double = 0;
static double pid_i_double = 0;
static double pid_d_double = 0;


void recompute_pid_parameters_and_set_pwm_voltage_double(double new_max_motor_pwm_voltage_double, double new_max_motor_regen_pwm_voltage_double)
{
	double new_max_error = (new_max_motor_pwm_voltage_double * (1 << (PID_SHIFT_RIGHT + PWM_VOLTAGE_VS_COMMUTATION_POSITION_FUDGE_SHIFT)));
	if (pid_p_double != 0) {
		new_max_error /= pid_p_double;
	}
	double new_derivative_constant_pid_scaled_for_averaging_double = (pid_d_double / (1 << DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT));
	double new_max_integral_term_double = (new_max_motor_pwm_voltage_double * (1 << (PID_SHIFT_RIGHT + PWM_VOLTAGE_VS_COMMUTATION_POSITION_FUDGE_SHIFT)));
//	double max_pd_terms = ((MAX_INT32 - MAX_I_TERM) / 2); // there are maximum values for the three terms. we do this so that we can calculate everything within a int32_t and don't overflow the math
	double new_max_error_change_double = 0;
	if (pid_d_double != 0) {
		new_max_error_change_double = (new_max_motor_pwm_voltage_double * (1 << (PID_SHIFT_RIGHT + PWM_VOLTAGE_VS_COMMUTATION_POSITION_FUDGE_SHIFT))) / pid_d_double;
	}
	__disable_irq();
	proportional_constant_pid_double = (double)pid_p_double;
	integral_constant_pid_double = (double)pid_i_double;
	derivative_constant_pid_scaled_for_averaging_double = new_derivative_constant_pid_scaled_for_averaging_double;
	max_error_double = new_max_error;
	max_integral_term_double = new_max_integral_term_double;
	max_error_change_double = new_max_error_change_double;
	max_motor_pwm_voltage_double = new_max_motor_pwm_voltage_double;
	max_motor_regen_pwm_voltage_double = new_max_motor_regen_pwm_voltage_double;
	__enable_irq();
}


void set_pid_constants_double(double p, double i, double d)
{
	pid_p_double = p;
	pid_i_double = i;
	pid_d_double = d;
	recompute_pid_parameters_and_set_pwm_voltage_double(max_motor_pwm_voltage_double, max_motor_regen_pwm_voltage_double);
}


double PID_controller_double(double error)
{
    double output_value;
    double proportional_term;
    double derivative_term;

	printf("PID_controller_double: error = %0.0lf\n", error);

	if(error < min_PID_error_double) {
		min_PID_error_double = error;
	}
	if(error > max_PID_error_double) {
		max_PID_error_double = error;
	}

	printf("PID_controller_double: error = %0.0lf\n", error);

    if (derivative_constant_pid_scaled_for_averaging_double != 0) {
		double error_change = error - previous_error_double;
		if(error_change > max_error_change_double) {
			error_change = max_error_change_double;
		}
		else if(error_change < -max_error_change_double) {
			error_change = -max_error_change_double;
		}
		low_pass_filtered_error_change_double = (low_pass_filtered_error_change_double * (DERIVATIVE_CONSTANT_AVERAGING_SCALAR - 1)) / (1 << DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT);
		low_pass_filtered_error_change_double += error_change;
		derivative_term = low_pass_filtered_error_change_double * derivative_constant_pid_scaled_for_averaging_double;
	}
	else {
		derivative_term = 0;
	}
    previous_error_double = error;

    if(error < -max_error_double) {
        error = -max_error_double;
    }
    else if(error > max_error_double) {
        error = max_error_double;
    }
	if (integral_constant_pid_double != 0) {
	    integral_term_double += error * integral_constant_pid_double;
		if(integral_term_double > max_integral_term_double) {
			integral_term_double = max_integral_term_double;
		}
		else if(integral_term_double < -max_integral_term_double) {
			integral_term_double = -max_integral_term_double;
		}
	}
	else {
		integral_term_double = 0;
	}
    proportional_term = error * proportional_constant_pid_double;

    output_value = (integral_term_double + proportional_term + derivative_term) / (1 << PID_SHIFT_RIGHT);

	return output_value;
}


int32_t PID_controller(int32_t error);

int main(void)
{
    uint32_t iteration = 0;
    double current_position_i64_double = 0;
    double hall_position_i64_double = 0;
    int64_t current_position_i64 = 0;
    int64_t hall_position_i64 = 0;

    // Seed the random number generator
    srand48(time(NULL));

    set_pid_constants_double(PROPORTIONAL_CONSTANT_PID, INTEGRAL_CONSTANT_PID, DERIVATIVE_CONSTANT_PID);
    set_pid_constants(PROPORTIONAL_CONSTANT_PID, INTEGRAL_CONSTANT_PID, DERIVATIVE_CONSTANT_PID);

    while (1) {
        iteration++;
        printf("--- Starting iteration %u -----------------------------------------------------\n", iteration);
        // Let's choose a random number for the current_position_i64_double and hall_position_i64_double between -(1 << 34) and (1 << 34)
        current_position_i64_double = (drand48() * (float)(((int64_t)1 << 60)) - ((int64_t)1 << 59));
        hall_position_i64_double = current_position_i64_double + (drand48() * (float)(((int64_t)1 << 31)) - ((int64_t)1 << 30));
        current_position_i64 = (int64_t)current_position_i64_double;
        hall_position_i64 = (int64_t)hall_position_i64_double;
        double error_double = current_position_i64_double - hall_position_i64_double;
        double motor_pwm_voltage_double = PID_controller_double(error_double);
        int32_t motor_pwm_voltage = PID_controller(current_position_i64 - hall_position_i64);
        printf("current_position_i64_double = %0.0lf, hall_position_i64_double = %0.0lf, error_double = %0.0lf\n", current_position_i64_double, hall_position_i64_double, error_double);
        double discrepancy = (double)motor_pwm_voltage - motor_pwm_voltage_double;
        printf("motor_pwm_voltage_double = %0.1lf, motor_pwm_voltage = %d, discrepancy = %0.0lf\n", motor_pwm_voltage_double, motor_pwm_voltage, discrepancy);
        // We need to make sure that motor_pwm_voltage_double does not exceed the limits, which are the limits of int32_t. If they are exceeded, the test fails
        if (motor_pwm_voltage_double > INT32_MAX) {
            printf("Error: motor_pwm_voltage_double is too large\n");
            printf("motor_pwm_voltage_double = %0.1lf\n", motor_pwm_voltage_double);
            printf("FAILED\n");
            exit(1);
        }
        else if (motor_pwm_voltage_double < INT32_MIN) {
            printf("Error: motor_pwm_voltage_double is too small\n");
            printf("motor_pwm_voltage_double = %0.1lf\n", motor_pwm_voltage_double);
            printf("FAILED\n");
            exit(1);
        }
        // And We also need to check that the sign of error_double matches the sign of motor_pwm_voltage_double. If they do not match, the test fails
        if ((error_double > 0 && motor_pwm_voltage_double < 0) || (error_double < 0 && motor_pwm_voltage_double > 0)) {
            if (fabs(motor_pwm_voltage_double) > 30000) {
                printf("Error: error_double and motor_pwm_voltage_double have different signs\n");
                printf("error_double = %0.0lf, motor_pwm_voltage_double = %0.1lf\n", error_double, motor_pwm_voltage_double);
                printf("FAILED\n");
                exit(1);
            }
        }
        // If the the absolute value of discrepancy is greater than MAX_MOTOR_PWM_VOLTAGE_DISCREPANCY, the test fails
        if (fabs(discrepancy) > MAX_MOTOR_PWM_VOLTAGE_DISCREPANCY) {
            printf("Error: discrepancy is too large\n");
            printf("discrepancy = %0.0lf\n", discrepancy);
            printf("FAILED\n");
            exit(1);
        }
    }
}