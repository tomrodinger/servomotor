#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "motor_control.h"
#include "mosfets.h"
#include "stm32g0xx_hal.h"


#define N_RANDOM_MOVES 1
#define EXTRA_TIME_STEPS_AFTER_QUEUE_EMPTY 100
#define LOG_FILENAME "motor_control.log"
#define QUEUE_SIZE 32
#define N_QUEUE_ITEMS_PER_TRAPEZOID_MOVE 6
#define MAX_DISPLACEMENT 8000000
#define MAX_TIME 320000

void TIM16_IRQHandler(void);
extern uint16_t hysteretic_motor_current;


double max_velocity = 21532835718365;
double max_acceleration = 10133099161;

void add_to_queue_simulated(double parameter, double n_time_steps, movement_type_t movement_type)
{

}


void compute_trapezoid_move_simulated(double total_displacement, double total_time, double *acceleration_returned, double *delta_t1_returned, double *delta_t2_returned)
{
	double delta_d = total_displacement * (double)(1 << 24);
	double delta_t1 = max_velocity / max_acceleration; // calculating detal_t1 without rounding
	if((delta_t1 * 2) > total_time) {
	    delta_t1 = total_time / 2;
	}
	double delta_t2 = total_time - (delta_t1 * 2);
	double numerator = delta_d;
	double denominator = ((delta_t1 + delta_t2) * delta_t1);
	double acceleration = numerator / denominator;

	printf("SIMULATED: max_velocity: %0.0lf\n", max_velocity);
	printf("SIMULATED: max_acceleration: %0.0lf\n", max_acceleration);
	printf("SIMULATED: delta_d: %0.0lf\n", delta_d);
	printf("SIMULATED: delta_t: %0.0lf\n", total_time);
	printf("SIMULATED: delta_t1: %0.0lf\n", delta_t1);
	printf("SIMULATED: delta_t2: %0.0lf\n", delta_t2);
	printf("SIMULATED: numerator: %0.0lf\n", numerator);
	printf("SIMULATED: denominator: %0.0lf\n", denominator);
	printf("SIMULATED: acceleration: %0.0lf\n", acceleration);

//	acceleration >>= 8;
	*acceleration_returned = acceleration;
	*delta_t1_returned = (uint32_t)delta_t1;
	*delta_t2_returned = (uint32_t)delta_t2;
}


void add_trapezoid_move_to_queue_simulated(double total_displacement, double total_time)
{
	double acceleration;
	double delta_t1;
	double delta_t2;

	compute_trapezoid_move_simulated(total_displacement, total_time, &acceleration, &delta_t1, &delta_t2);

	add_to_queue_simulated(acceleration, delta_t1, MOVE_WITH_ACCELERATION);
	add_to_queue_simulated(0, delta_t2, MOVE_WITH_ACCELERATION);
	add_to_queue_simulated(-acceleration, delta_t1, MOVE_WITH_ACCELERATION);
}


int main(void)
{
    uint32_t time_step = 0;
    uint32_t min_time_steps = 1000; // at least this many time steps will be executed but more may be executed if there are moves in the queue
    FILE *fp;

    printf("Simulation Start\n");
    printf("The update frequency is %u\n", get_update_frequency());

    // Set the random seed based on the current time
    srand(time(NULL));
    


    // Open file for writing
    fp = fopen(LOG_FILENAME, "w");
    if (fp == NULL) {
        printf("Error opening file %s for writing!\n", LOG_FILENAME);
        return 1;
    }

    // Write header to file
    fprintf(fp, "# TimeStep CurrentPosition CurrentVelocity TIM1_CH1 TIM1_CH2 TIM3_CH1 TIM3_CH2\n");

    set_max_motor_current(200, 200);
    enable_mosfets();

    uint32_t n_random_moves = 0;
    while(time_step < min_time_steps) {
        uint32_t n_items_in_queue = get_n_items_in_queue();
        if ((n_items_in_queue <= QUEUE_SIZE - N_QUEUE_ITEMS_PER_TRAPEZOID_MOVE) && (n_random_moves < N_RANDOM_MOVES) && (time_step >= 10)) {
            // For the displacement, choose a random number between -MAX_DISPLACEMENT and MAX_DISPLACEMENT
            int32_t total_displacement = (rand() % (2 * MAX_DISPLACEMENT + 1)) - MAX_DISPLACEMENT;
            // For the time, choose a random number between 0 and MAX_TIME
            uint32_t time_for_move = rand() % (MAX_TIME + 1);
            total_displacement = 100000;
            time_for_move = 64000;
            add_trapezoid_move_to_queue_simulated(total_displacement, time_for_move);
            add_trapezoid_move_to_queue(total_displacement, time_for_move);
            n_random_moves++;
        }
        TIM16_IRQHandler();
        int32_t current_position = get_current_position();
        int32_t current_velocity = get_current_velocity();
        // Log data to file
        fprintf(fp, "%u %d %d %u\n", time_step, current_position, current_velocity, hysteretic_motor_current);
        time_step++;
        if (n_items_in_queue != 0) {
            min_time_steps = time_step + EXTRA_TIME_STEPS_AFTER_QUEUE_EMPTY;
        }
    }

    // Close file
    fclose(fp);

    return 0;
}
