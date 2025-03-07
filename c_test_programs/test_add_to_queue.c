#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include "motor_control.h"
#include "mosfets.h"
#include "error_handling.h"
#include "stm32g0xx_hal.h"


#define N_RANDOM_MOVES 32000
#define EXTRA_TIME_STEPS_AFTER_QUEUE_EMPTY 100
#define MIN_MOVE_TIME_SECONDS 0.01
#define MAX_MOVE_TIME_SECONDS 100.0
#define LOG_FILENAME "test_add_to_queue.log"
#define QUEUE_SIZE 32
#define N_QUEUE_ITEMS_PER_TRAPEZOID_MOVE 6
#define MAX_DISPLACEMENT 8000000
#define MAX_TIME 320000
#define MAX_POSITION_DISCREPANCY 100000000
#define LOG_INTERVAL_TIME_SECONDS 1
//#define VERBOSE

void TIM16_IRQHandler(void);

int64_t max_velocity = MAX_VELOCITY;
int64_t max_acceleration = MAX_ACCELERATION;
uint32_t time_step = 0;

uint32_t n_items_in_queue_n_items_in_queue_double_discrepancy_count = 0;
uint32_t n_items_in_queue_n_items_in_queue_int128_discrepancy_count = 0;

double max_current_position_deviation_double = 0;
double max_current_velocity_deviation_double = 0;
__int128_t max_current_position_deviation_int128 = 0;
__int128_t max_current_velocity_deviation_int128 = 0;
__int128_t min_simulated_current_position_int128 = 0;
__int128_t max_simulated_current_position_int128 = 0;
__int128_t min_simulated_current_velocity_int128 = 0;
__int128_t max_simulated_current_velocity_int128 = 0;

typedef struct {
	movement_type_t movement_type;
	union {
	    double acceleration; // we use this variable if the movement_type == MOVE_WITH_ACCELERATION
	    double velocity;     // we use this variable if the movement_type == MOVE_WITH_VELOCITY
	};
    uint32_t n_time_steps;
} movement_queue_double_t;
static movement_queue_double_t movement_queue_double[MOVEMENT_QUEUE_SIZE];
static uint8_t n_items_in_queue_double = 0;
static uint8_t queue_write_position_double = 0;
static uint8_t queue_read_position_double = 0;
static double current_velocity_double = 0;
static double current_position_double = 0;
static double velocity_after_last_queue_item_double = 0;
static double position_after_last_queue_item_double = 0;

typedef struct {
	movement_type_t movement_type;
	union {
	    __int128_t acceleration; // we use this variable if the movement_type == MOVE_WITH_ACCELERATION
	    __int128_t velocity;     // we use this variable if the movement_type == MOVE_WITH_VELOCITY
	};
    uint32_t n_time_steps;
} movement_queue_int128_t;
static movement_queue_int128_t movement_queue_int128[MOVEMENT_QUEUE_SIZE];
static uint8_t n_items_in_queue_int128 = 0;
static uint8_t queue_write_position_int128 = 0;
static uint8_t queue_read_position_int128 = 0;
static __int128_t current_velocity_int128 = 0;
static __int128_t current_position_int128 = 0;
static __int128_t velocity_after_last_queue_item_int128 = 0;
static __int128_t position_after_last_queue_item_int128 = 0;


__int128_t llabs_int128(__int128_t x)
{
    if (x < 0) {
        return -x;
    }
    return x;
}


void add_to_queue_simulated_double(double parameter, double n_time_steps, movement_type_t movement_type)
{
    double predicted_final_velocity_double;
    double predicted_final_position_double;

    if (n_time_steps == 0) {
        return;
    }
    if(n_items_in_queue_double < MOVEMENT_QUEUE_SIZE) {
		movement_queue_double[queue_write_position_double].movement_type = movement_type;
		if(movement_type == MOVE_WITH_ACCELERATION) {
            if(fabs(movement_queue_double[queue_write_position_double].acceleration) > (double)max_acceleration) {
	            fatal_error(ERROR_ACCEL_TOO_HIGH); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
            }
	        movement_queue_double[queue_write_position_double].acceleration = parameter;
    	    movement_queue_double[queue_write_position_double].acceleration *= (1 << ACCELERATION_SHIFT_LEFT);
			predicted_final_velocity_double = velocity_after_last_queue_item_double + movement_queue_double[queue_write_position_double].acceleration * n_time_steps;
			if(fabs(predicted_final_velocity_double) > (double)max_velocity) {
				fatal_error(ERROR_PREDICTED_VELOCITY_TOO_HIGH); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
			}
			double position_change_due_to_queue_item_double = velocity_after_last_queue_item_double * n_time_steps + movement_queue_double[queue_write_position_double].acceleration * (((double)n_time_steps * (n_time_steps + 1)) / 2);
			predicted_final_position_double = position_after_last_queue_item_double + position_change_due_to_queue_item_double;
        }
        else {
	        movement_queue_double[queue_write_position_double].velocity = parameter;
    	    movement_queue_double[queue_write_position_double].velocity *= (1 << VELOCITY_SHIFT_LEFT);
            predicted_final_velocity_double = movement_queue_double[queue_write_position_double].velocity;
            if(fabs(movement_queue_double[queue_write_position_double].velocity) > (double)max_velocity) {
	            fatal_error(ERROR_VEL_TOO_HIGH); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
            }
			double position_change_due_to_queue_item = movement_queue_double[queue_write_position_double].velocity * n_time_steps;
			predicted_final_position_double = position_after_last_queue_item_double + position_change_due_to_queue_item;
        }
    }
    else {
        printf("Error: Queue is full\n");
        exit(1);
    }
    movement_queue_double[queue_write_position_double].n_time_steps = n_time_steps;
    queue_write_position_double = (queue_write_position_double + 1) & (MOVEMENT_QUEUE_SIZE - 1);
    n_items_in_queue_double++;
    position_after_last_queue_item_double = predicted_final_position_double;
    velocity_after_last_queue_item_double = predicted_final_velocity_double;
}


void add_to_queue_simulated_int128(__int128_t parameter, __int128_t n_time_steps, movement_type_t movement_type)
{
    __int128_t predicted_final_velocity_int128;
    __int128_t predicted_final_position_int128;

    if (n_time_steps == 0) {
        return;
    }
    if(n_items_in_queue_int128 < MOVEMENT_QUEUE_SIZE) {
		movement_queue_int128[queue_write_position_int128].movement_type = movement_type;
		if(movement_type == MOVE_WITH_ACCELERATION) {
            if(llabs_int128(movement_queue_int128[queue_write_position_int128].acceleration) > (__int128_t)max_acceleration) {
	            fatal_error(ERROR_ACCEL_TOO_HIGH); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
            }
	        movement_queue_int128[queue_write_position_int128].acceleration = parameter;
    	    movement_queue_int128[queue_write_position_int128].acceleration *= (1 << ACCELERATION_SHIFT_LEFT);
			predicted_final_velocity_int128 = velocity_after_last_queue_item_int128 + movement_queue_int128[queue_write_position_int128].acceleration * n_time_steps;
			if(llabs_int128(predicted_final_velocity_int128) > (__int128_t)max_velocity) {
				fatal_error(ERROR_PREDICTED_VELOCITY_TOO_HIGH); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
			}
			__int128_t position_change_due_to_queue_item_int128 = velocity_after_last_queue_item_int128 * n_time_steps + movement_queue_int128[queue_write_position_int128].acceleration * (((__int128_t)n_time_steps * (n_time_steps + 1)) / 2);
			predicted_final_position_int128 = position_after_last_queue_item_int128 + position_change_due_to_queue_item_int128;
        }
        else {
	        movement_queue_int128[queue_write_position_int128].velocity = parameter;
    	    movement_queue_int128[queue_write_position_int128].velocity *= (1 << VELOCITY_SHIFT_LEFT);
            predicted_final_velocity_int128 = movement_queue_int128[queue_write_position_int128].velocity;
            if(llabs_int128(movement_queue_int128[queue_write_position_int128].velocity) > (__int128_t)max_velocity) {
	            fatal_error(ERROR_VEL_TOO_HIGH); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
            }
			__int128_t position_change_due_to_queue_item = movement_queue_int128[queue_write_position_int128].velocity * n_time_steps;
			predicted_final_position_int128 = position_after_last_queue_item_int128 + position_change_due_to_queue_item;
        }
    }
    else {
        printf("Error: Queue is full\n");
        exit(1);
    }
    movement_queue_int128[queue_write_position_int128].n_time_steps = n_time_steps;
    queue_write_position_int128 = (queue_write_position_int128 + 1) & (MOVEMENT_QUEUE_SIZE - 1);
    n_items_in_queue_int128++;
    position_after_last_queue_item_int128 = predicted_final_position_int128;
    velocity_after_last_queue_item_int128 = predicted_final_velocity_int128;
}


void compute_trapezoid_move_simulated_double(double total_displacement, double total_time, double *acceleration_returned, double *delta_t1_returned, double *delta_t2_returned)
{
	double delta_d = total_displacement * (double)(1 << 24);
	double delta_t1 = (double)max_velocity / (double)max_acceleration; // calculating detal_t1 without rounding
	if((delta_t1 * 2) > total_time) {
	    delta_t1 = total_time / 2;
	}
	double delta_t2 = total_time - (delta_t1 * 2);
	double numerator = delta_d;
	double denominator = ((delta_t1 + delta_t2) * delta_t1);
	double acceleration = numerator / denominator;

#ifdef VERBOSE
    printf("SIMULATED (double):\n");
	printf("   max_velocity: %0.0lf\n", (double)max_velocity);
	printf("   max_acceleration: %0.0lf\n", (double)max_acceleration);
	printf("   delta_d: %0.0lf\n", delta_d);
	printf("   delta_t: %0.0lf\n", total_time);
	printf("   delta_t1: %0.0lf\n", delta_t1);
	printf("   delta_t2: %0.0lf\n", delta_t2);
	printf("   numerator: %0.0lf\n", numerator);
	printf("   denominator: %0.0lf\n", denominator);
	printf("   acceleration: %0.0lf\n", acceleration);
#endif

//	acceleration >>= 8;
	*acceleration_returned = acceleration;
	*delta_t1_returned = (uint32_t)delta_t1;
	*delta_t2_returned = (uint32_t)delta_t2;
}


void compute_trapezoid_move_simulated_int128(__int128_t total_displacement, __int128_t total_time, __int128_t *acceleration_returned, __int128_t *delta_t1_returned, __int128_t *delta_t2_returned)
{
	__int128_t delta_d = total_displacement * (double)(1 << 24);
	__int128_t delta_t1 = (__int128_t)max_velocity / (__int128_t)max_acceleration; // calculating detal_t1 without rounding
	if((delta_t1 * 2) > total_time) {
	    delta_t1 = total_time / 2;
	}
	__int128_t delta_t2 = total_time - (delta_t1 * 2);
	__int128_t numerator = delta_d;
	__int128_t denominator = ((delta_t1 + delta_t2) * delta_t1);
	__int128_t acceleration = numerator / denominator;

#ifdef VERBOSE
    printf("SIMULATED (int128):\n");
	printf("   max_velocity: %lld\n", max_velocity);
	printf("   max_acceleration: %lld\n", max_acceleration);
	printf("   delta_d: %lld\n", (int64_t)delta_d);
	printf("   delta_t: %lld\n", (int64_t)total_time);
	printf("   delta_t1: %lld\n", (int64_t)delta_t1);
	printf("   delta_t2: %lld\n", (int64_t)delta_t2);
	printf("   numerator: %lld\n", (int64_t)numerator);
	printf("   denominator: %lld\n", (int64_t)denominator);
	printf("   acceleration: %lld\n", (int64_t)acceleration);
#endif

    if ( (delta_d > INT64_MAX) || (delta_d < INT64_MIN) ) {
        printf("Error: delta_d is too large (positive or negative)\n");
        exit(1);
    }
    if ( (delta_t1 > INT64_MAX) || (delta_t1 < INT64_MIN) ) {
        printf("Error: delta_t1 is too large (positive or negative)\n");
        exit(1);
    }
    if ( (delta_t2 > INT64_MAX) || (delta_t2 < INT64_MIN) ) {
        printf("Error: delta_t2 is too large (positive or negative)\n");
        exit(1);
    }
    if ( (total_time > INT64_MAX) || (total_time < INT64_MIN) ) {
        printf("Error: total_time is too large (positive or negative)\n");
        exit(1);
    }
    if ( (numerator > INT64_MAX) || (numerator < INT64_MIN) ) {
        printf("Error: numerator is too large (positive or negative)\n");
        exit(1);
    }
    if ( (denominator > INT64_MAX) || (denominator < INT64_MIN) ) {
        printf("Error: denominator is too large (positive or negative)\n");
        exit(1);
    }

	*acceleration_returned = acceleration;
	*delta_t1_returned = delta_t1;
	*delta_t2_returned = delta_t2;
}


void add_trapezoid_move_to_queue_simulated(double total_displacement, double total_time)
{
	double acceleration_double;
	double delta_t1_double;
	double delta_t2_double;

	__int128_t acceleration_int128;
	__int128_t delta_t1_int128;
	__int128_t delta_t2_int128;

	compute_trapezoid_move_simulated_double(total_displacement, total_time, &acceleration_double, &delta_t1_double, &delta_t2_double);
	compute_trapezoid_move_simulated_int128(total_displacement, total_time, &acceleration_int128, &delta_t1_int128, &delta_t2_int128);

	add_to_queue_simulated_double(acceleration_double,  delta_t1_double, MOVE_WITH_ACCELERATION);
	add_to_queue_simulated_double(0,                    delta_t2_double, MOVE_WITH_ACCELERATION);
	add_to_queue_simulated_double(-acceleration_double, delta_t1_double, MOVE_WITH_ACCELERATION);

	add_to_queue_simulated_int128(acceleration_int128,  delta_t1_int128, MOVE_WITH_ACCELERATION);
	add_to_queue_simulated_int128(0,                    delta_t2_int128, MOVE_WITH_ACCELERATION);
	add_to_queue_simulated_int128(-acceleration_int128, delta_t1_int128, MOVE_WITH_ACCELERATION);
}


void simulated_time_step_double(void)
{
    if(n_items_in_queue_double > 0) {
        // there is an assumption here that any item in the queue will always have one or more time steps
        // see the add_to_queue function where we make sure to never add an item to the queue with zero time steps
        if(movement_queue_double[queue_read_position_double].movement_type == MOVE_WITH_ACCELERATION) {
            current_velocity_double += movement_queue_double[queue_read_position_double].acceleration; // consume one time step worth of acceleration
            movement_queue_double[queue_read_position_double].n_time_steps--;
            if(movement_queue_double[queue_read_position_double].n_time_steps == 0) {
                queue_read_position_double = (queue_read_position_double + 1) & (MOVEMENT_QUEUE_SIZE - 1);
                n_items_in_queue_double--;
            }
        }
        else {
            current_velocity_double = movement_queue_double[queue_read_position_double].velocity; // velocity is constant during this time step
            movement_queue_double[queue_read_position_double].n_time_steps--;
            if(movement_queue_double[queue_read_position_double].n_time_steps == 0) {
                queue_read_position_double = (queue_read_position_double + 1) & (MOVEMENT_QUEUE_SIZE - 1);
                n_items_in_queue_double--;
            }
        }
    }
    else {
        if(fabs(position_after_last_queue_item_double - current_position_double) > MAX_POSITION_DISCREPANCY) {
            double position_discrepancy = position_after_last_queue_item_double - current_position_double;
            printf("position_after_last_queue_item_double = %0.0lf\n", position_after_last_queue_item_double);
            printf("current_position_double = %0.0lf\n", current_position_double);
            printf("position_discrepancy = %0.0lf\n", position_discrepancy);
            printf("time_step = %u\n", time_step);
            fatal_error(ERROR_POSITION_DISCREPANCY); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
        }
    }

	if((current_velocity_double > (double)max_velocity) || (current_velocity_double < -(double)max_velocity)) {
		fatal_error(ERROR_VEL_TOO_HIGH); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
	}
	current_position_double += current_velocity_double;
}


void simulated_time_step_int128(void)
{
    if(n_items_in_queue_int128 > 0) {
        // there is an assumption here that any item in the queue will always have one or more time steps
        // see the add_to_queue function where we make sure to never add an item to the queue with zero time steps
        if(movement_queue_int128[queue_read_position_int128].movement_type == MOVE_WITH_ACCELERATION) {
            current_velocity_int128 += movement_queue_int128[queue_read_position_int128].acceleration; // consume one time step worth of acceleration
            movement_queue_int128[queue_read_position_int128].n_time_steps--;
            if(movement_queue_int128[queue_read_position_int128].n_time_steps == 0) {
                queue_read_position_int128 = (queue_read_position_int128 + 1) & (MOVEMENT_QUEUE_SIZE - 1);
                n_items_in_queue_int128--;
            }
        }
        else {
            current_velocity_int128 = movement_queue_int128[queue_read_position_int128].velocity; // velocity is constant during this time step
            movement_queue_int128[queue_read_position_int128].n_time_steps--;
            if(movement_queue_int128[queue_read_position_int128].n_time_steps == 0) {
                queue_read_position_int128 = (queue_read_position_int128 + 1) & (MOVEMENT_QUEUE_SIZE - 1);
                n_items_in_queue_int128--;
            }
        }
    }
    else {
        if (llabs_int128(position_after_last_queue_item_int128 - current_position_int128) > MAX_POSITION_DISCREPANCY) {
            __int128_t position_discrepancy = position_after_last_queue_item_int128 - current_position_int128;
            printf("position_after_last_queue_item_int128 = %lld\n", (int64_t)position_after_last_queue_item_int128);
            printf("current_position_int128 = %lld\n", (int64_t)current_position_int128);
            printf("position_discrepancy = %lld\n", (int64_t)position_discrepancy);
            printf("time_step = %u\n", time_step);
            printf("position_after_last_queue_item_int128 uper 64 bits = %lld\n", (int64_t)(position_after_last_queue_item_int128 >> 64));
            printf("current_position_int128 uper 64 bits = %lld\n", (int64_t)(current_position_int128 >> 64));
            printf("position_discrepancy uper 64 bits = %lld\n", (int64_t)(position_discrepancy >> 64));
            fatal_error(ERROR_POSITION_DISCREPANCY); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
        }
    }

	if((current_velocity_int128 > (__int128_t)max_velocity) || (current_velocity_int128 < -(__int128_t)max_velocity)) {
		fatal_error(ERROR_VEL_TOO_HIGH); // All error messages are defined in error_text.h, which is an autogenerated file based on error_codes.json in the servomotor Python module (<repo root>/python_programs/servomotor/error_codes.json)
	}
	current_position_int128 += current_velocity_int128;
}


void print_summary(void)
{
    printf("min_simulated_current_position_int128 = %lld\n", (int64_t)min_simulated_current_position_int128);
    printf("max_simulated_current_position_int128 = %lld\n", (int64_t)max_simulated_current_position_int128);
    printf("min_simulated_current_velocity_int128 = %lld\n", (int64_t)min_simulated_current_velocity_int128);
    printf("max_simulated_current_velocity_int128 = %lld\n", (int64_t)max_simulated_current_velocity_int128);
    if ((min_simulated_current_position_int128 > INT64_MAX) || (min_simulated_current_position_int128 < INT64_MIN)) {
        printf("Error: min_simulated_current_position_int128 is too large (positive or negative)\n");
        exit(1);
    }
    if ((max_simulated_current_position_int128 > INT64_MAX) || (max_simulated_current_position_int128 < INT64_MIN)) {
        printf("Error: max_simulated_current_position_int128 is too large (positive or negative)\n");
        exit(1);
    }
    if ((min_simulated_current_velocity_int128 > INT64_MAX) || (min_simulated_current_velocity_int128 < INT64_MIN)) {
        printf("Error: min_simulated_current_velocity_int128 is too large (positive or negative)\n");
        exit(1);
    }
    if ((max_simulated_current_velocity_int128 > INT64_MAX) || (max_simulated_current_velocity_int128 < INT64_MIN)) {
        printf("Error: max_simulated_current_velocity_int128 is too large (positive or negative)\n");
        exit(1);
    }
    printf("n_items_in_queue_n_items_in_queue_double_discrepancy_count = %u\n", n_items_in_queue_n_items_in_queue_double_discrepancy_count);
    printf("n_items_in_queue_n_items_in_queue_int128_discrepancy_count = %u\n", n_items_in_queue_n_items_in_queue_int128_discrepancy_count);
    printf("max_current_position_deviation_double = %0.0lf\n", max_current_position_deviation_double);
    printf("max_current_velocity_deviation_double = %0.0lf\n", max_current_velocity_deviation_double);
    printf("max_current_position_deviation_int128 = %lld\n", (int64_t)max_current_position_deviation_int128);
    printf("max_current_velocity_deviation_int128 = %lld\n", (int64_t)max_current_velocity_deviation_int128);
    if ((max_current_position_deviation_int128 > INT64_MAX) || (max_current_position_deviation_int128 < INT64_MIN)) {
        printf("Error: max_current_position_deviation_int128 is too large (positive or negative)\n");
        exit(1);
    }
    if ((max_current_velocity_deviation_int128 > INT64_MAX) || (max_current_velocity_deviation_int128 < INT64_MIN)) {
        printf("Error: max_current_velocity_deviation_int128 is too large (positive or negative)\n");
        exit(1);
    }
    printf("The number of time steps executed is %u\n", time_step);
    printf("This is equivalent to %0.1lf seconds (%0.2lf days)\n", (double)time_step / get_update_frequency(), (double)time_step / get_update_frequency() / (3600 * 24));
}

int main(void)
{
    uint32_t min_time_steps = 1000; // at least this many time steps will be executed but more may be executed if there are moves in the queue
    FILE *fp;
//    uint32_t n_items_in_queue_double_n_items_in_queue_int128_discrepancy_count = 0;

    printf("Simulation Start\n");
    printf("The update frequency is %u\n", get_update_frequency());

    // Set the random seed based on the current time
    srand48(time(NULL));
    


    // Open file for writing
    fp = fopen(LOG_FILENAME, "w");
    if (fp == NULL) {
        printf("Error opening file %s for writing!\n", LOG_FILENAME);
        return 1;
    }

    // Write header to file
    fprintf(fp, "TimeStep#   CurrentPosition   CurrentVelocity   SimulatedPositionDouble   SimulatedVelocityDouble   SimulatedPositionInt128   SimulatedVelocityInt128   PositionDeviationDouble   VelocityDeviationDouble   PositionDeviationInt128   VelocityDeviationInt128\n");

    set_max_motor_current(390, 390);
    enable_mosfets();

    uint32_t n_random_moves = 0;
    // let's get the start time (as a floating point number in units seconds)
    double start_time = (float)clock() / CLOCKS_PER_SEC;
    double next_log_time = start_time + LOG_INTERVAL_TIME_SECONDS;
    while(time_step < min_time_steps) {
        uint32_t n_items_in_queue = get_n_items_in_queue();
//        if (n_items_in_queue_double != n_items_in_queue_int128) {
//            printf("Error: disagreement at step %u: n_items_in_queue_double = %u, n_items_in_queue_int128 = %u\n", time_step, n_items_in_queue_double, n_items_in_queue_int128);
//            n_items_in_queue_double_n_items_in_queue_int128_discrepancy_count++;
//            exit(1);
//        }
        if (n_items_in_queue != n_items_in_queue_double) {
//            printf("Error: disagreement: at step %u: n_items_in_queue = %u, n_items_in_queue_double = %u\n", time_step, n_items_in_queue, n_items_in_queue_double);
            n_items_in_queue_n_items_in_queue_double_discrepancy_count++;
//            exit(1);
        }
        if (n_items_in_queue != n_items_in_queue_int128) {
//            printf("Error: disagreement: at step %u: n_items_in_queue = %u, n_items_in_queue_int128 = %u\n", time_step, n_items_in_queue, n_items_in_queue_int128);
            n_items_in_queue_n_items_in_queue_int128_discrepancy_count++;
//            exit(1);
        }
        if ((n_items_in_queue <= QUEUE_SIZE - N_QUEUE_ITEMS_PER_TRAPEZOID_MOVE) && (n_random_moves < N_RANDOM_MOVES) && (time_step >= 10)) {
            uint32_t move_time_motor_units;
            double total_displacement_double;
            while (1) {
                // Let's choose a velocity (in units rotations per second) at random between -MAX_RPS and MAX_RPS
                double random_rps = (drand48() * 2 * MAX_RPS) - MAX_RPS;
                // Let's choose a random time in seconds between MIN_MOVE_TIME_SECONDS and MAX_MOVE_TIME_SECONDS seconds. These are double values.
                double move_time_seconds = (drand48() * (MAX_MOVE_TIME_SECONDS - MIN_MOVE_TIME_SECONDS)) + MIN_MOVE_TIME_SECONDS;
                // convert the time to the motor's units
                move_time_motor_units = (uint32_t)(move_time_seconds * get_update_frequency());
                // We will compute the displacement (in units rotations) based on the velocity (in rotations per second) and time (in seconds)
                double displacement_rotations = (random_rps * move_time_seconds);
                printf("random_rps = %0.2lf, move_time_seconds = %0.2lf, displacement_rotations = %0.2lf\n", random_rps, move_time_seconds, displacement_rotations);
                // convert the displacement to the motor's units
                total_displacement_double = displacement_rotations * (double)(MICROSTEPS_PER_ROTATION >> 32);
                if ( (total_displacement_double > INT32_MAX) || (total_displacement_double < INT32_MIN) ) {
                    printf("Error: magnitude of total_displacement_double is too large\n");
                    printf("total_displacement_double = %0.0lf\n", total_displacement_double);
                    continue;
                }
                else {
                    printf("total_displacement_double = %0.0lf\n", total_displacement_double);
                    break;
                }
            };
            add_trapezoid_move_to_queue_simulated(total_displacement_double, move_time_motor_units);
            add_trapezoid_move_to_queue((int32_t)total_displacement_double, move_time_motor_units);
            n_random_moves++;
        }
        simulated_time_step_double();
        simulated_time_step_int128();
        TIM16_IRQHandler();

        int64_t current_position = get_motor_position();
        int32_t current_velocity = get_current_velocity();
        int64_t simulated_current_position_double = (int64_t)(current_position_double / ((uint64_t)1 << 32));
        int32_t simulated_current_velocity_double = (int32_t)(current_velocity_double / ((uint64_t)1 << 32));
        int64_t simulated_current_position_int128 = (int64_t)(current_position_int128 / ((uint64_t)1 << 32));
        int32_t simulated_current_velocity_int128 = (int32_t)(current_velocity_int128 / ((uint64_t)1 << 32));

        if (simulated_current_position_int128 < min_simulated_current_position_int128) {
            min_simulated_current_position_int128 = simulated_current_position_int128;
        }
        if (simulated_current_position_int128 > max_simulated_current_position_int128) {
            max_simulated_current_position_int128 = simulated_current_position_int128;
        }
        if (simulated_current_velocity_int128 < min_simulated_current_velocity_int128) {
            min_simulated_current_velocity_int128 = simulated_current_velocity_int128;
        }
        if (simulated_current_velocity_int128 > max_simulated_current_velocity_int128) {
            max_simulated_current_velocity_int128 = simulated_current_velocity_int128;
        }

        double current_position_deviation_double = fabs((double)current_position - simulated_current_position_double);
        double current_velocity_deviation_double = fabs((double)current_velocity - simulated_current_velocity_double);
        if (current_position_deviation_double > max_current_position_deviation_double) {
            max_current_position_deviation_double = current_position_deviation_double;
        }
        if (current_velocity_deviation_double > max_current_velocity_deviation_double) {
            max_current_velocity_deviation_double = current_velocity_deviation_double;
        }
        __int128_t current_position_deviation_int128 = llabs_int128(current_position - simulated_current_position_int128);
        __int128_t current_velocity_deviation_int128 = llabs_int128(current_velocity - simulated_current_velocity_int128);
        if (current_position_deviation_int128 > max_current_position_deviation_int128) {
            max_current_position_deviation_int128 = current_position_deviation_int128;
        }
        if (current_velocity_deviation_int128 > max_current_velocity_deviation_int128) {
            max_current_velocity_deviation_int128 = current_velocity_deviation_int128;
        }

        // Is it time to log yet?
        double current_time = (float)clock() / CLOCKS_PER_SEC;
        if (current_time >= next_log_time) {
            next_log_time += LOG_INTERVAL_TIME_SECONDS;
            // Log data to file
            fprintf(fp, "%u %lld %d %lld %d %lld %d %lf %lf %lld %lld\n",
                    time_step, current_position, current_velocity,
                    simulated_current_position_double, simulated_current_velocity_double,
                    simulated_current_position_int128, simulated_current_velocity_int128,
                    current_position_deviation_double, current_velocity_deviation_double,
                    (int64_t)current_position_deviation_int128, (int64_t)current_velocity_deviation_int128);
            print_summary();
        }

        time_step++;
        if (n_items_in_queue != 0) {
            min_time_steps = time_step + EXTRA_TIME_STEPS_AFTER_QUEUE_EMPTY;
        }
    }

    // Close file
    fclose(fp);

    printf("Simulation finished successfully\n");
//    printf("n_items_in_queue_double_n_items_in_queue_int128_discrepancy_count = %u\n", n_items_in_queue_double_n_items_in_queue_int128_discrepancy_count);
    print_summary();

    return 0;
}
