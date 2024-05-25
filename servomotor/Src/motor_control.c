#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32g0xx_hal.h"
#include "microsecond_clock.h"
#include "debug_uart.h"
#include "PWM.h"
#include "ADC.h"
#include "hall_sensor_calculations.h"
#include "mosfets.h"
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
#include "goertzel_algorithm_constants.h"
#include "error_handling.h"
#include "RS485.h"
#include "motor_control.h"
#include "leds.h"
#include "GPIO_interrupts.h"
#include "device_status.h"
#include "global_variables.h"
#include "profiler.h"

#define MAX_HOMING_ERROR 50000

#if defined(PRODUCT_NAME_M3) || defined(PRODUCT_NAME_M4)
#define MAX_PWM_VOLTAGE_AT_ZERO_VELOCITY PWM_PERIOD
#else
#define MAX_PWM_VOLTAGE_AT_ZERO_VELOCITY (300)
#endif
#define ANALOG_WATCHDOG_LIMIT_MULTIPLIER (200)
#ifdef PRODUCT_NAME_M1
#define VOLTS_PER_ROTATIONAL_VELOCITY 300
#endif
#ifdef PRODUCT_NAME_M2
#define VOLTS_PER_ROTATIONAL_VELOCITY 300
#endif
//#define DO_DETAILED_PROFILING
#define UINT32_MIDPOINT 2147483648
#define COMMUTATION_POSITION_OFFSET_DEFAULT (UINT32_MIDPOINT / (N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS) * (N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS)) // a number that when added to the zero position will yield a lookup at index 0 in the commutation table (see commutation_table_XX.h, where XX is the product name)
#define POSITION_OUT_OF_RANGE_FATAL_ERROR_THRESHOLD 2000000000  // a position in the range -2000000000 to 2000000000 is valid
// This is the number of microsteps to turn the motor through one quarter of one commutation cycle (not one revolution)
#define HALL_TO_POSITION_90_DEGREE_OFFSET ((N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS) >> 2)

#define EXPECTED_MOTOR_CURRENT_BASELINE 1152
#define MOTOR_CURRENT_BASELINE_TOLERANCE 200
#define MIN_MOTOR_CURRENT_BASELINE (EXPECTED_MOTOR_CURRENT_BASELINE - MOTOR_CURRENT_BASELINE_TOLERANCE)
#define MAX_MOTOR_CURRENT_BASELINE (EXPECTED_MOTOR_CURRENT_BASELINE + MOTOR_CURRENT_BASELINE_TOLERANCE)
#define MAX_MOTOR_CURRENT 300 // make sure this number is less than (EXPECTED_MOTOR_CURRENT_BASELINE - MOTOR_CURRENT_BASELINE_TOLERANCE)
//#define MAX_MOTOR_CONTROL_PERIOD_FATAL_ERROR_THRESHOLD (PWM_PERIOD_MICROSECONDS * 3 - 2) // we want to ensure that the update of the PWM happens every second PWM period (we are not able to do calculatons fast enough to updated it every peripd unfortunately)
#define MAX_MOTOR_CONTROL_PERIOD_FATAL_ERROR_THRESHOLD (100) // we want to ensure that the update of the PWM happens every second PWM period (we are not able to do calculatons fast enough to updated it every peripd unfortunately)
#define COMPUTED_VELOCITY_CALIBRATION_CONSTANT 3000
#define COMPUTED_VELOCITY_CALIBRATION_SHIFT 16
#if !defined (PRODUCT_NAME_M3) && !defined (PRODUCT_NAME_M4)
const struct three_phase_data_struct commutation_lookup_table[N_COMMUTATION_STEPS] = COMMUTATION_LOOKUP_TABLE_INITIALIZER;
#endif

#define ACCELERATION_SHIFT_LEFT 8
#define VELOCITY_SHIFT_LEFT 12
#define MOVEMENT_QUEUE_SIZE 32 // this has to be a power of 2
typedef struct __attribute__((__packed__)) {
	movement_type_t movement_type;
	union {
	    int64_t acceleration; // we use this variable if the movement_type == MOVE_WITH_ACCELERATION
	    int64_t velocity;     // we use this variable if the movement_type == MOVE_WITH_VELOCITY
	};
    uint32_t n_time_steps;
} movement_queue_t;
static movement_queue_t movement_queue[MOVEMENT_QUEUE_SIZE];
static uint8_t queue_write_position = 0;
static uint8_t queue_read_position = 0;
static uint8_t n_items_in_queue = 0;
static uint32_t commutation_position_offset = COMMUTATION_POSITION_OFFSET_DEFAULT;
static int32_t sensor_position = 0; // this is the rotational position of the magnetic ring and is used for electrical commutation
static int32_t hall_position = 0;
static int32_t hall_position_delta = 0;
//static int32_t velocity_moving_average = 0;
static uint16_t max_motor_pwm_voltage = DEFAULT_MAX_MOTOR_PWM_VOLTAGE;
static uint16_t max_motor_regen_pwm_voltage = DEFAULT_MAX_MOTOR_PWM_VOLTAGE;
static int32_t velocity = 0;
static int64_t current_velocity_i64 = 0;
typedef union {
	int64_t i64;
	int32_t i32[2];  // in some cases, we need to access the 64-bit position as two 32-bit integers
} position_union;
static position_union current_position;
#if defined(PRODUCT_NAME_M3) || defined(PRODUCT_NAME_M4)
#define DEFAULT_ACTUAL_STEP_POSITION (COMMUTATION_POSITION_OFFSET_DEFAULT / N_COMMUTATION_SUB_STEPS) & (N_COMMUTATION_STEPS - 1)
static uint32_t actual_step_position = DEFAULT_ACTUAL_STEP_POSITION;
#endif
static uint32_t commutation_position = 0;
static int64_t max_acceleration = MAX_ACCELERATION;
static int64_t max_velocity = MAX_VELOCITY;
static int32_t motor_pwm_voltage = 0;
static int32_t desired_motor_pwm_voltage = 0;
//#ifdef PRODUCT_NAME_M1
//static int32_t desired_motor_pwm_voltage_before_shifting = 0;
//#endif
static uint8_t motor_control_mode = OPEN_LOOP_POSITION_CONTROL;
#define GO_TO_CLOSED_LOOP_MODE_TEST_MODE 2
static uint8_t test_mode = 0;

// PID position control related variables:
static int32_t integral_term = 0;
static int32_t previous_error = 0;
static int32_t low_pass_filtered_error_change = 0;
// PID informatonal variables:
int32_t PID_error = 0;
int32_t min_PID_error = 2147483647;
int32_t max_PID_error = -2147483648;
int32_t PID_P = 0;
int32_t PID_I = 0;
int32_t PID_D = 0;
int32_t PID_output_value = 0;

static uint8_t go_to_closed_loop_step = 0;
#ifdef PRODUCT_NAME_M1
static uint16_t vibration_four_step;
#endif
static uint16_t motor_current_baseline = 1350;
static int32_t position_lower_safety_limit = -2000000000;
static int32_t position_upper_safety_limit = 2000000000;
static uint8_t homing_active = 0;
#ifdef PRODUCT_NAME_M1
static uint8_t vibration_active = 0;
#endif
//static int8_t homing_direction = 0; // 1 for positive, -1 for negative
static uint8_t calibration_step = 0;
static uint8_t calibration_print_output;
static uint8_t decelerate_to_stop_active = 0;
static uint8_t motor_busy = 0; // this will be set to 1 while the motor is doing a long action like calibration or homing. other movement commands cannot be executed during this time.

static int32_t max_hall_position_delta = -2000000000;
static int32_t min_hall_position_delta = 2000000000;
static int32_t average_hall_position_delta = 0;
static int32_t average_hall_position_delta_count = 0;
static int64_t position_after_last_queue_item = 0;
static int64_t velocity_after_last_queue_item = 0;
static uint8_t multipurpose_data_type = 0;
static uint16_t multipurpose_data_size = 0;


// for the following calibration movement calculations, the time unit is one motor calculation cycle. at the time of this
// writing, this is roughly 25 microseconds. it may be noted elsewhere what this is more accurately.

// this is the velocity used during coast phases of the calibration.
// CALIBRATION_MAX_VELOCITY is in units of motor microsteps per time unit.
// for example, 64 means that the motor will take 64 microsteps per time unit.
#define CALIBRATION_MAX_VELOCITY CALIBRATION_CAPTURE_STEP_SIZE

#define CALIBRATION_VELOCITY_SHIFT 7 // this is how many binary decimal places we are keeping for the calibration velocity value

#define CALIBRATION_ACCELERATION 1 // this is the acceleration used during calibration. the units are microsteps divided by
                                   // (1 << CALIBRATION_VELOCITY_SHIFT) per one unit of time squared.

// this is the time to accelerate or decelerate during calibration. careful that this number ends up being an integer.
#define CALIBRATION_ACCELERATION_TIME ((CALIBRATION_MAX_VELOCITY << CALIBRATION_VELOCITY_SHIFT) / CALIBRATION_ACCELERATION)

// this is the distance delta during one acceleration or deceleration move in the unit of microsteps
#define CALIBRATION_ACCELERATION_DISTANCE ((((((uint32_t)CALIBRATION_ACCELERATION_TIME * (uint32_t)(CALIBRATION_ACCELERATION_TIME + 1)) >> 1) * CALIBRATION_ACCELERATION) >> CALIBRATION_VELOCITY_SHIFT))

// this is the time to coast at constant velocity such that the distance covered is CALIBRATION_ACCELERATION_DISTANCE
#define CALIBRATION_SMALL_COAST_TIME ((uint16_t)((uint32_t)CALIBRATION_ACCELERATION_DISTANCE / CALIBRATION_MAX_VELOCITY))

// this is the movement distance to cover exactly one rotation of the hall sensor magnetic disc in the unit of microsteps
#define CALIBRATION_MOVEMENT_DISTANCE ((int32_t)ONE_REVOLUTION_MICROSTEPS)

// this is the time to coast at constant velocity while capturing hall sensor readings. this should be exactly such that after one
// coast phase, the motor will rotate through one hall sensor revolution (ie. moves CALIBRATION_MOVEMENT_DISTANCE).
#define CALIBRATION_COAST_TIME (CALIBRATION_MOVEMENT_DISTANCE / CALIBRATION_MAX_VELOCITY)

#if defined(PRODUCT_NAME_M3) || defined(PRODUCT_NAME_M4)
#define CALIBRATION_DESIRED_MOTOR_PWM_VOLTAGE_STEP1 400
#define CALIBRATION_DESIRED_MOTOR_PWM_VOLTAGE_STEP2 600
#else
#define CALIBRATION_DESIRED_MOTOR_PWM_VOLTAGE_STEP1 150
#define CALIBRATION_DESIRED_MOTOR_PWM_VOLTAGE_STEP2 250
#endif

#define HOMING_MAX_VELOCITY 32

#define HOMING_VELOCITY_SHIFT 8 // this is how many binary decimal places we are keeping for the homing velocity value

#define HOMING_ACCELERATION 1 // this is the acceleration used during calibration. the units are microsteps divided by
                                   // (1 << HOMING_VELOCITY_SHIFT) per one unit of time squared.

// this is the time to accelerate or decelerate during calibration. careful that this number ends up being an integer.
#define HOMING_ACCELERATION_TIME ((HOMING_MAX_VELOCITY << HOMING_VELOCITY_SHIFT) / HOMING_ACCELERATION)

// this is the distance delta during one acceleration or deceleration move in the unit of microsteps
#define HOMING_ACCELERATION_DISTANCE ((((((uint32_t)HOMING_ACCELERATION_TIME * (uint32_t)(HOMING_ACCELERATION_TIME + 1)) >> 1) * HOMING_ACCELERATION) >> HOMING_VELOCITY_SHIFT))

// this is the time to coast at constant velocity such that the distance covered is HOMING_ACCELERATION_DISTANCE
#define HOMING_SMALL_COAST_TIME ((uint16_t)((uint32_t)HOMING_ACCELERATION_DISTANCE / HOMING_MAX_VELOCITY))

#define HOMING_MAX_REVOLUTIONS 300

// this is the movement distance to cover exactly one rotation of the hall sensor magnetic disc in the unit of microsteps
#define HOMING_MOVEMENT_DISTANCE (N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS * ONE_REVOLUTION_STEPS * HOMING_MAX_REVOLUTIONS)

// this is the time to coast at constant velocity while capturing hall sensor readings. this should be exactly such that after one
// coast phase, the motor will rotate through one hall sensor revolution (ie. moves HOMING_MOVEMENT_DISTANCE).
#define HOMING_COAST_TIME (HOMING_MOVEMENT_DISTANCE / HOMING_MAX_VELOCITY)


/*
 * Example:
 *    CALIBRATION_MAX_VELOCITY 64
 *    CALIBRATION_VELOCITY_SHIFT 8
 *    CALIBRATION_ACCELERATION 1
 *    CALIBRATION_ACCELERATION_TIME 16384
 *    CALIBRATION_ACCELERATION_DISTANCE 524320
 *    CALIBRATION_SMALL_COAST_TIME 8192.5 rounded down to 8192
 *    CALIBRATION_COAST_TIME 10080
 *    CALIBRATION_MOVEMENT_DISTANCE 645120
 */

#define CALIBRATION_DATA_COLLECTION_SHIFT_RIGHT 8
#define CALIBRATION_DATA_COLLECTION_N_TURNS_TIMES_256 (384) // 1.5 turns * 256
#define CALIBRATION_DATA_N_ITEMS ((TOTAL_NUMBER_OF_SEGMENTS / 3 * CALIBRATION_DATA_COLLECTION_N_TURNS_TIMES_256) >> CALIBRATION_DATA_COLLECTION_SHIFT_RIGHT)
struct calibration_struct {
	uint16_t local_min_or_max;
	int32_t local_min_or_max_position;
};
struct calibration_data_struct {
	int64_t hall_position_vs_position_sum;
	int64_t hall_position_vs_position_sum_phases_reversed;
	uint32_t hall_position_vs_position_count;
	int32_t min_hall_position_vs_position;
	int32_t max_hall_position_vs_position;
	int32_t min_hall_position_vs_position_phases_reversed;
	int32_t max_hall_position_vs_position_phases_reversed;
};
static uint8_t calibration_data_queue_upper_threshold = 0;
static uint8_t calibration_data_queue_lower_threshold = 0;
static uint8_t calibration_data_index = 0;
struct calibration_struct calibration[3][CALIBRATION_DATA_N_ITEMS];
struct calibration_data_struct *calibration_data = (void*)&calibration; // calibration_data uses the same memory as calibration
#define CALIBRATION_DATA_SIZE_IN_BYTES sizeof(calibration)

struct __attribute__((__packed__)) fast_capture_data_struct {
	uint16_t hall1;
	uint16_t hall2;
	uint16_t hall3;
	uint16_t hall_position_16bit;
};
struct fast_capture_data_struct *fast_capture_data = (void*)&calibration; // fast_capture_data uses the same memory as calibration
uint16_t fast_capture_data_size = sizeof(calibration) / sizeof(struct fast_capture_data_struct);
uint16_t fast_capture_data_index;
uint8_t fast_capture_data_active = 0;
uint8_t fast_capture_data_result_ready = 0;
static uint32_t calibration_index[3];


struct capture_struct {
    uint8_t capture_type;
    uint8_t avg_counter;
    uint16_t hall1_sum;
    uint16_t hall2_sum;
    uint16_t hall3_sum;
};
struct capture_struct capture = {0};


struct homing_struct {
    uint8_t move_number;
    int8_t direction;
    uint32_t time;
    int16_t velocity;
    int16_t acceleration;
    int32_t start_position;
    int32_t start_six_step_hall_position;
};
struct homing_struct homing = {0};


struct closed_loop_struct {
    uint8_t move_number;
    uint32_t time;
    int16_t velocity;
    int16_t acceleration;
    uint8_t capturing_data;
    uint8_t avg_counter;
    uint32_t hall1_sum;
    uint32_t hall2_sum;
    uint32_t hall3_sum;
    uint16_t max_hall_reading;
    int32_t max_hall_position;
    int32_t max_hall_distance;
};
struct closed_loop_struct closed_loop = {0};


void clear_the_queue_and_stop_no_disable_interrupt(void)
{
    queue_read_position = 0;
    queue_write_position = 0;
    n_items_in_queue = 0;
	current_velocity_i64 = 0;
}


/*
delta_t = 3000000
delta_d = int(microsteps_per_rotation * 1.0 + 0.5)
delta_t1 = int(max_velocity / max_acceleration + 0.5)
if 2 * delta_t1 > delta_t:
    print("Doing the special case of the move where the time is very short but so is the distance")
    delta_t1 = delta_t // 2
delta_t2 = delta_t - 2 * delta_t1
numerator = delta_d
denominator = ((delta_t1 + delta_t2) * delta_t1)
acceleration = int(delta_d / ((delta_t1 + delta_t2) * delta_t1) + 0.5)
*/

void compute_trapezoid_move(int32_t total_displacement, uint32_t total_time, int32_t *acceleration_returned, uint32_t *delta_t1_returned, uint32_t *delta_t2_returned)
{
	int64_t delta_d = (int64_t)total_displacement << 24;
//	uint32_t delta_t1 = ((max_acceleration >> 1) + max_velocity) / max_acceleration; // calculating detal_t1 with rounding
	int64_t delta_t1 = max_velocity / max_acceleration; // calculating detal_t1 without rounding
	if((delta_t1 << 1) > total_time) {
	    delta_t1 = total_time >> 1;
	}
	uint32_t delta_t2 = total_time - (delta_t1 << 1);
	int64_t numerator = delta_d;
	int64_t denominator = ((delta_t1 + delta_t2) * delta_t1);
	int64_t acceleration = numerator / denominator;

	print_int64("max_velocity: ", max_velocity);
	print_int64("max_acceleration: ", max_acceleration);
	print_int64("delta_d: ", (int64_t)delta_d);
	print_int64("delta_t: ", (int64_t)total_time);
	print_int64("delta_t1: ", (int64_t)delta_t1);
	print_int64("delta_t2: ", (int64_t)delta_t2);
	print_int64("numerator: ", (int64_t)numerator);
	print_int64("denominator: ", (int64_t)denominator);
	print_int64("acceleration: ", (int64_t)acceleration);

//	acceleration >>= 8;
	*acceleration_returned = acceleration;
	*delta_t1_returned = (uint32_t)delta_t1;
	*delta_t2_returned = (uint32_t)delta_t2;
}


uint32_t hall1_sum;
uint32_t hall2_sum;
uint32_t hall3_sum;
uint8_t avg_counter;
uint16_t hall_data_buffer[3];

#ifdef PRODUCT_NAME_M1
#define CALIBRATION_TIME (get_update_frequency() * 1)
#else
#ifdef PRODUCT_NAME_M2
#define CALIBRATION_TIME (get_update_frequency() * 2)
#else
#ifdef PRODUCT_NAME_M3
#define CALIBRATION_TIME (get_update_frequency() * 4)
#else
#ifdef PRODUCT_NAME_M4
#define CALIBRATION_TIME (get_update_frequency() * 4)
// If no servomotor is defined then exit with a compile time error
#else
#error "PRODUCT_NAME_M1 or PRODUCT_NAME_M2 or PRODUCT_NAME_M3 or PRODUCT_NAME_M4 must be defined"
#endif
#endif
#endif
#endif

#define CALIBRATION_DISTANCE ((uint32_t)(((uint64_t)ONE_REVOLUTION_MICROSTEPS * (uint64_t)CALIBRATION_DATA_COLLECTION_N_TURNS_TIMES_256) >> CALIBRATION_DATA_COLLECTION_SHIFT_RIGHT))
#define HALL_PEAK_FIND_THREASHOLD 200
uint8_t hall_rising_flag[3];
uint16_t hall_local_maximum[3];
uint16_t hall_local_minimum[3];
int32_t hall_local_maximum_position[3];
int32_t hall_local_minimum_position[3];
#define N_QUEUED_CALIBRATION_CYCLES 6
#define N_QUEUED_CALIBRATION_MOVES_PER_CYCLE 5

void queue_up_calibration_moves(void)
{
	int32_t acceleration;
	uint32_t delta_t1;
	uint32_t delta_t2;
	uint32_t i;

	// we will do 1.5 rotations of the motor shaft and then capture the hall sensor readings
	// the number of hal sensors peaks and valleyes (ie. cycles should be aproximately 1.5 times the number of pole pairs)
	compute_trapezoid_move(CALIBRATION_DISTANCE, CALIBRATION_TIME / 2, &acceleration, &delta_t1, &delta_t2);

	for(i = 0; i < N_QUEUED_CALIBRATION_CYCLES; i++) {
		add_to_queue(-acceleration, delta_t1, MOVE_WITH_ACCELERATION);
		add_to_queue(0, delta_t2, MOVE_WITH_ACCELERATION);
		add_to_queue(acceleration, delta_t1 * 2, MOVE_WITH_ACCELERATION);
		add_to_queue(0, delta_t2, MOVE_WITH_ACCELERATION);
		add_to_queue(-acceleration, delta_t1, MOVE_WITH_ACCELERATION);
	}
}


void start_calibration(uint8_t print_output)
{
	uint8_t j;

	if(motor_control_mode != OPEN_LOOP_POSITION_CONTROL) {
		fatal_error(7); // "not in open loop" (all error text is defined in error_text.c)
	}
	if(n_items_in_queue != 0) {
		fatal_error(8); // "queue not empty" (all error text is defined in error_text.c)
	}
	if(motor_busy) {
		fatal_error(19); // "motor busy" (all error text is defined in error_text.c)
	}

    GPIOA->BSRR = ((1 << 15) << 16); // reset the stepper motor driver

	if(print_output) {
	   	rs485_transmit("Calibration start\n", 18);
	}

	calibration_print_output = print_output;
	desired_motor_pwm_voltage = CALIBRATION_DESIRED_MOTOR_PWM_VOLTAGE_STEP1;
	current_position.i64 = 0;
	global_settings.motor_phases_reversed = 0;
	global_settings.commutation_position_offset = COMMUTATION_POSITION_OFFSET_DEFAULT;
	commutation_position_offset = COMMUTATION_POSITION_OFFSET_DEFAULT;
#if defined (PRODUCT_NAME_M3) || defined (PRODUCT_NAME_M4)
	actual_step_position = DEFAULT_ACTUAL_STEP_POSITION;
#endif

    volatile uint32_t i;
    for(i = 0; i < 1000; i++); // make a delay
    GPIOA->BSRR = (1 << 15); // take the stepper motor driver out of reset by making the reset pin high

	check_current_sensor_and_enable_mosfets();
	queue_up_calibration_moves();
	motor_busy = 1;
	for(j = 0; j < 3; j++) {
		hall_rising_flag[j] = 1;
		hall_local_maximum[j] = 0;
		hall_local_minimum[j] = 65535;
		hall_local_maximum_position[j] = 0;
		hall_local_minimum_position[j] = 0;
		calibration_index[j] = 0;
	}
	hall1_sum = 0;
	hall2_sum = 0;
	hall3_sum = 0;
	avg_counter = 0;

	calibration_step = 1;
}


void handle_calibration_logic(void)
{
	uint16_t hall_reading;
	uint8_t j;

	if(calibration_step == 1) {
		if(n_items_in_queue == 2 + (N_QUEUED_CALIBRATION_CYCLES - 1) * N_QUEUED_CALIBRATION_MOVES_PER_CYCLE) {
			if(calibration_print_output) {
				hall1_sum += get_hall_sensor1_voltage();
				hall2_sum += get_hall_sensor2_voltage();
				hall3_sum += get_hall_sensor3_voltage();
	//			hall1_sum = get_hall_sensor1_voltage();
	//			hall2_sum = get_hall_sensor2_voltage();
	//			hall3_sum = get_hall_sensor3_voltage();
				avg_counter++;
				if(avg_counter == 16) {
					hall_data_buffer[0] = (hall1_sum >> 1) - HALL_SENSOR_SHIFT;
					hall_data_buffer[1] = (hall2_sum >> 1) - HALL_SENSOR_SHIFT;
					hall_data_buffer[2] = (hall3_sum >> 1) - HALL_SENSOR_SHIFT;
	//				hall_data_buffer[0] = (hall1_sum << 3) - HALL_SENSOR_SHIFT;
	//				hall_data_buffer[1] = (hall2_sum << 3) - HALL_SENSOR_SHIFT;
	//				hall_data_buffer[2] = (hall3_sum << 3) - HALL_SENSOR_SHIFT;
					rs485_transmit((void*)hall_data_buffer, 6);
					avg_counter = 0;
					hall1_sum = 0;
					hall2_sum = 0;
					hall3_sum = 0;
				}
			}
			else {
				for(j = 0; j < 3; j++) {
					if(calibration_index[j] < CALIBRATION_DATA_N_ITEMS) {
						switch(j) {
						case 0:
							hall_reading = get_hall_sensor1_voltage();
							break;
						case 1:
							hall_reading = get_hall_sensor2_voltage();
							break;
						case 2:
							hall_reading = get_hall_sensor3_voltage();
							break;
						}
			//			if((hall_reading > 10000) || (hall_reading < 7000)) {
			//				fatal_error(9); // "hall sensor error" (all error text is defined in error_text.c)
			//			}
						if(hall_rising_flag[j]) {
							if(hall_reading > hall_local_maximum[j]) {
								hall_local_maximum[j] = hall_reading;
								hall_local_maximum_position[j] = current_position.i32[1];
							}
							if(hall_local_maximum[j] - hall_reading > HALL_PEAK_FIND_THREASHOLD) {
								calibration[j][calibration_index[j]].local_min_or_max = hall_local_maximum[j];
								calibration[j][calibration_index[j]].local_min_or_max_position = hall_local_maximum_position[j];
								calibration_index[j]++;
								hall_local_minimum[j] = hall_reading;
								hall_local_minimum_position[j] = current_position.i32[1];
								hall_rising_flag[j] = 0;
							}
						}
						else {
							if(hall_reading < hall_local_minimum[j]) {
								hall_local_minimum[j] = hall_reading;
								hall_local_minimum_position[j] = current_position.i32[1];
							}
							if(hall_reading - hall_local_minimum[j] > HALL_PEAK_FIND_THREASHOLD) {
								calibration[j][calibration_index[j]].local_min_or_max = hall_local_minimum[j];
								calibration[j][calibration_index[j]].local_min_or_max_position = hall_local_minimum_position[j];
								calibration_index[j]++;
								hall_local_maximum[j] = hall_reading;
								hall_local_maximum_position[j] = current_position.i32[1];
								hall_rising_flag[j] = 1;
							}
						}
					}
					else {
						fatal_error(10); // "calibration overflow" (all error text is defined in error_text.c)
					}
				}
			}
		}
		else if(n_items_in_queue == 0 + (N_QUEUED_CALIBRATION_CYCLES - 1) * N_QUEUED_CALIBRATION_MOVES_PER_CYCLE) {
			if(calibration_print_output) {
				disable_mosfets();
				rs485_transmit("Calibration capture done\n", 25);
				calibration_step = 0;
				motor_busy = 0;
			}
			else {
				calibration_step = 2;
			}
		}
	}
	else if (calibration_step == 3) {
//		if(n_items_in_queue == (N_QUEUED_CALIBRATION_CYCLES - 3) * N_QUEUED_CALIBRATION_MOVES_PER_CYCLE) {
		if(current_position.i64 == 0) {
			zero_hall_position(1); // sero the hall sensor position but keep the offset (so it won't go to exactly zero but will be within one cycle of the sensor)
			hall_position = 0;
			calibration_data_queue_upper_threshold = (N_QUEUED_CALIBRATION_CYCLES - 3) * N_QUEUED_CALIBRATION_MOVES_PER_CYCLE;
			calibration_data_queue_lower_threshold = calibration_data_queue_upper_threshold - N_QUEUED_CALIBRATION_MOVES_PER_CYCLE;
			calibration_data_index = 0;
			desired_motor_pwm_voltage = CALIBRATION_DESIRED_MOTOR_PWM_VOLTAGE_STEP2;
			calibration_step = 4;
		}
	}
	else if (calibration_step == 4) {
		if(n_items_in_queue > 0) {
			if(n_items_in_queue <= calibration_data_queue_lower_threshold) {
				calibration_data_queue_upper_threshold -= N_QUEUED_CALIBRATION_MOVES_PER_CYCLE;
				calibration_data_queue_lower_threshold = calibration_data_queue_upper_threshold - N_QUEUED_CALIBRATION_MOVES_PER_CYCLE;
				calibration_data_index++;
			}
			if(n_items_in_queue <= calibration_data_queue_upper_threshold) {
				int32_t sp = sensor_position;
				int32_t cp = current_position.i32[1];
				int32_t difference = sp - cp;
				int32_t difference_phases_reversed = sp + cp;
				if(calibration_data[calibration_data_index].hall_position_vs_position_count == 0) {
					calibration_data[calibration_data_index].max_hall_position_vs_position = difference;
					calibration_data[calibration_data_index].min_hall_position_vs_position = difference;
					calibration_data[calibration_data_index].max_hall_position_vs_position_phases_reversed = difference_phases_reversed;
					calibration_data[calibration_data_index].min_hall_position_vs_position_phases_reversed = difference_phases_reversed;
				}
				else {
					if(difference > calibration_data[calibration_data_index].max_hall_position_vs_position) {
						calibration_data[calibration_data_index].max_hall_position_vs_position = difference;
					}
					else if(difference < calibration_data[calibration_data_index].min_hall_position_vs_position) {
						calibration_data[calibration_data_index].min_hall_position_vs_position = difference;
					}
					if(difference_phases_reversed > calibration_data[calibration_data_index].max_hall_position_vs_position_phases_reversed) {
						calibration_data[calibration_data_index].max_hall_position_vs_position_phases_reversed = difference_phases_reversed;
					}
					else if(difference_phases_reversed < calibration_data[calibration_data_index].min_hall_position_vs_position_phases_reversed) {
						calibration_data[calibration_data_index].min_hall_position_vs_position_phases_reversed = difference_phases_reversed;
					}
				}
				calibration_data[calibration_data_index].hall_position_vs_position_sum += difference;
				calibration_data[calibration_data_index].hall_position_vs_position_sum_phases_reversed += difference_phases_reversed;
				calibration_data[calibration_data_index].hall_position_vs_position_count++;
			}
		}
		else {
			disable_mosfets();
			calibration_step = 5;
		}
	}
}


void compute_midlines_from_calibration_data(void)
{
	int32_t position_delta;
	uint16_t peak_to_peak;
	int32_t min_position_delta;
	int32_t max_position_delta;
	uint16_t min_peak_to_peak;
	uint16_t max_peak_to_peak;
	uint8_t min_or_max;
	char buf[150];
	uint16_t i;
	uint8_t j;

	for(j = 0; j < 3; j++) {
		for(i = 0; i < calibration_index[j]; i++) {
			sprintf(buf, "index: %u  local_min_or_max: %u  position: %ld\n",
					i, calibration[j][i].local_min_or_max, calibration[j][i].local_min_or_max_position);
			print_debug_string(buf);
		}

		min_or_max = 0;
		min_position_delta = 2147483640;
		max_position_delta = -2147483640;
		min_peak_to_peak = 65535;
		max_peak_to_peak = 0;
		for(i = 1; i < calibration_index[j]; i++) {
			position_delta = calibration[j][i].local_min_or_max_position - calibration[j][i - 1].local_min_or_max_position;
			if(position_delta > max_position_delta) {
				max_position_delta = position_delta;
			}
			if(position_delta < min_position_delta) {
				min_position_delta = position_delta;
			}

			if(min_or_max == 0) {
				peak_to_peak = calibration[j][i - 1].local_min_or_max - calibration[j][i].local_min_or_max;
				min_or_max = 1;
			}
			else {
				peak_to_peak = calibration[j][i].local_min_or_max - calibration[j][i - 1].local_min_or_max;
				min_or_max = 0;
			}
			if(peak_to_peak > max_peak_to_peak) {
				max_peak_to_peak = peak_to_peak;
			}
			if(peak_to_peak < min_peak_to_peak) {
				min_peak_to_peak = peak_to_peak;
			}

			sprintf(buf, "i: %u  max to min delta: %u  position delta: %ld\n", i, peak_to_peak, position_delta);
			print_debug_string(buf);
		}

		sprintf(buf, "min_position_delta: %ld  max_position_delta: %ld\n", min_position_delta, max_position_delta);
		print_debug_string(buf);
		sprintf(buf, "min_peak_to_peak: %u  max_peak_to_peak: %u\n", min_peak_to_peak, max_peak_to_peak);
		print_debug_string(buf);
	}

	#define N_POLES (TOTAL_NUMBER_OF_SEGMENTS / 3)
	uint32_t minima_and_maxima_avg[3] = {0, 0, 0};
	uint16_t midline[3];
	for(j = 0; j < 3; j++) {
		if(calibration_index[j] < N_POLES) {
			fatal_error(11); // "not enough minima or maxima" (all error text is defined in error_text.c)
		}

		uint16_t start_calibration_index = (calibration_index[j] - N_POLES);
		for(i = start_calibration_index; i < calibration_index[j]; i++) {
			minima_and_maxima_avg[j] += (uint32_t)calibration[j][i].local_min_or_max;
			sprintf(buf, "Averaging: index: %u  local_min_or_max: %u\n", i, calibration[j][i].local_min_or_max);
			print_debug_string(buf);
		}
		minima_and_maxima_avg[j] /= N_POLES;
		midline[j] = (uint16_t)(((int32_t)minima_and_maxima_avg[j] << 3) - HALL_SENSOR_SHIFT);
	}

	global_settings.hall1_midline = midline[0];
	global_settings.hall2_midline = midline[1];
	global_settings.hall3_midline = midline[2];

	for(j = 0; j < 3; j++) {
		sprintf(buf, "The average and midline for hall sensor %hu are: %lu  %u\n", j + 1, minima_and_maxima_avg[j], midline[j]);
		print_debug_string(buf);
	}
}


void compute_commutation_offset_from_calibration_data(void)
{
	char buf[150];
	uint8_t i = 0;

	while(1) {
		if(calibration_data[i].hall_position_vs_position_count == 0) {
			break;
		}
		sprintf(buf, "calibration_data index %hu:\n", i);
		print_debug_string(buf);

		sprintf(buf, "   hall position vs. position sums (forward and reverse phase): %ld %ld\n", (int32_t)calibration_data[i].hall_position_vs_position_sum,
		                                                                                          (int32_t)calibration_data[i].hall_position_vs_position_sum_phases_reversed);
		print_debug_string(buf);
		sprintf(buf, "   hall position v. position count: %lu\n", calibration_data[i].hall_position_vs_position_count);
		print_debug_string(buf);

		sprintf(buf, "   min and max hall position vs. position: %ld %ld\n", calibration_data[i].min_hall_position_vs_position,
		                                                                     calibration_data[i].max_hall_position_vs_position);
		print_debug_string(buf);
		int32_t min_to_max_delta = calibration_data[i].max_hall_position_vs_position - calibration_data[i].min_hall_position_vs_position;
		sprintf(buf, "   min to max delta: %ld\n", min_to_max_delta);
		print_debug_string(buf);

		sprintf(buf, "   min to max hall position v. position (phases reversed): %ld %ld\n", calibration_data[i].min_hall_position_vs_position_phases_reversed,
		                                                                                     calibration_data[i].max_hall_position_vs_position_phases_reversed);
		print_debug_string(buf);
		int32_t min_to_max_delta_phases_reversed = calibration_data[i].max_hall_position_vs_position_phases_reversed - calibration_data[i].min_hall_position_vs_position_phases_reversed;
		sprintf(buf, "   min to max delta (phases reversed): %ld\n", min_to_max_delta_phases_reversed);
		print_debug_string(buf);

		int64_t average_hall_position_vs_position = calibration_data[i].hall_position_vs_position_sum / calibration_data[i].hall_position_vs_position_count;
		int64_t average_hall_position_vs_position_reversed = calibration_data[i].hall_position_vs_position_sum_phases_reversed / calibration_data[i].hall_position_vs_position_count;
		if(min_to_max_delta < min_to_max_delta_phases_reversed) {
			global_settings.motor_phases_reversed = 0;
			global_settings.commutation_position_offset = COMMUTATION_POSITION_OFFSET_DEFAULT - (int32_t)average_hall_position_vs_position;
			commutation_position_offset = global_settings.commutation_position_offset;
			sprintf(buf, "   average hall position vs. position: %ld\n", (int32_t)average_hall_position_vs_position);
			print_debug_string(buf);
			sprintf(buf, "   average hall position vs. position (not used): %ld\n", (int32_t)average_hall_position_vs_position_reversed);
			print_debug_string(buf);
		}
		else {
			global_settings.motor_phases_reversed = 1;
			global_settings.commutation_position_offset = COMMUTATION_POSITION_OFFSET_DEFAULT - (int32_t)average_hall_position_vs_position_reversed;
			commutation_position_offset = global_settings.commutation_position_offset;
			sprintf(buf, "   average hall position vs. position: %ld\n", (int32_t)average_hall_position_vs_position_reversed);
			print_debug_string(buf);
			sprintf(buf, "   average hall position vs. position (not used): %ld\n", (int32_t)average_hall_position_vs_position);
			print_debug_string(buf);
		}
		i++;
	}
}

uint8_t process_calibration_data(void)
{
	char buf[150];

	if(calibration_step == 2) {
		compute_midlines_from_calibration_data();
		sprintf(buf, "n_items_in_queue after finishing calculating the midlines: %u\n", (unsigned int)n_items_in_queue);
		print_debug_string(buf);
		memset(calibration_data, 0, CALIBRATION_DATA_SIZE_IN_BYTES);
		calibration_step = 3;
	}
	else if(calibration_step == 5) {
		compute_commutation_offset_from_calibration_data();
		calibration_step = 0;
		return 1;
	}
	return 0;
}

#ifdef PRODUCT_NAME_M1

#define GO_TO_CLOSED_LOOP_N_DATA_ITEMS ((VIBRATION_QUARTER_CYCLE_DURATION * 4) >> GO_TO_CLOSED_LOOP_SHIFT_RIGHT)

static volatile int32_t *go_to_closed_loop_data = (void*)&calibration; // go_to_closed_loop_data uses the same data as calibration (the ligic that takes it to closed loop never runs at the same time as the calibration logic)
static volatile uint16_t go_to_closed_loop_max_data_items = sizeof(calibration) / sizeof(int32_t);
static volatile uint16_t go_to_closed_loop_avg_counter = 0;
static volatile uint16_t vibration_duration_counter = 0;
static volatile uint16_t data_index = 0;

void start_go_to_closed_loop_mode(void)
{
//	if(motor_control_mode != OPEN_LOOP_POSITION_CONTROL) {
//		fatal_error(7); // "not in open loop" (all error text is defined in error_text.c)
//	}
	if(n_items_in_queue != 0) {
		fatal_error(8); // "queue not empty" (all error text is defined in error_text.c)
	}
	if(motor_busy) {
		fatal_error(19); // "motor busy" (all error text is defined in error_text.c)
	}
	if(GO_TO_CLOSED_LOOP_N_DATA_ITEMS + 2 > sizeof(calibration) / sizeof(int32_t)) { // sanity check here. make sure we don't overflow the available memory.
		fatal_error(29); // "debug1" (all error text is defined in error_text.c)
	}

	motor_busy = 1;

	memset((void*)go_to_closed_loop_data, 0, GO_TO_CLOSED_LOOP_N_DATA_ITEMS * sizeof(int32_t));
	
	print_debug_string("Go to closed loop mode start\n");
	check_current_sensor_and_enable_mosfets();

	TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
	zero_hall_position(1);
	commutation_position_offset = global_settings.commutation_position_offset;
	set_motor_control_mode(OPEN_LOOP_PWM_VOLTAGE_CONTROL);
	desired_motor_pwm_voltage = GO_TO_CLOSED_LOOP_MOTOR_PWM_VOLTAGE;
	vibration_duration_counter = 0;
	vibration_four_step = 0;
	go_to_closed_loop_avg_counter = 0;
	data_index = 0;
	go_to_closed_loop_step = 1;
    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
}

void go_to_closed_loop_mode_logic(void)
{
	if(go_to_closed_loop_step == 1) {
		if(go_to_closed_loop_avg_counter < GO_TO_CLOSED_LOOP_AVERAGE_SAMPLES) {
			if(go_to_closed_loop_avg_counter >= (GO_TO_CLOSED_LOOP_AVERAGE_SAMPLES >> 3)) {
				go_to_closed_loop_data[data_index >> GO_TO_CLOSED_LOOP_SHIFT_RIGHT] += hall_position_delta;
			}
			data_index++;

			vibration_duration_counter++;
			if(vibration_duration_counter >= VIBRATION_QUARTER_CYCLE_DURATION) {
				vibration_duration_counter = 0;
				vibration_four_step++;
				if(vibration_four_step < 4) {	
					commutation_position_offset += HALL_TO_POSITION_90_DEGREE_OFFSET;
				}
				else {
					commutation_position_offset -= (HALL_TO_POSITION_90_DEGREE_OFFSET * 3);
					vibration_four_step = 0;
					data_index = 0;
					go_to_closed_loop_avg_counter++;
				}
			}
		}
		else {
			desired_motor_pwm_voltage = 0;
			go_to_closed_loop_step = 2;
		}
	}
}


struct goertzel_algorithm_result_t {
    int32_t real;
    int32_t imag;
};

struct goertzel_algorithm_result_t goertzel_algorithm(volatile int32_t samples[], uint16_t n_samples)
{
    struct goertzel_algorithm_result_t result;
    int32_t d1 = 0;
	int32_t d2 = 0;
	uint16_t n;
	uint16_t n2;
	char buf[150];

    sprintf(buf, "Running the Goertzel algorithm on %hu samples\n", n_samples);
	print_debug_string(buf);
    sprintf(buf, "The multipliers are: %u %u\n", W_REAL_MULTIPLIER, W_IMAG_MULTIPLIER);
	print_debug_string(buf);
    sprintf(buf, "The shifts are: %u %u\n", W_REAL_SHIFT, W_IMAG_SHIFT);
	print_debug_string(buf);

	n2 = GO_TO_CLOSED_LOOP_PHASE_ADJUSTMENT_SAMPLES;
    for(n = 0; n < n_samples; n++) {
        int64_t d1_times_w_real_multiplier_64bit = (int64_t)d1 * W_REAL_MULTIPLIER;
		if(n2 < 0) {
			n2 += n_samples;
		}
		else if(n2 >= n_samples) {
			n2 -= n_samples;
		}
        int32_t y = samples[n2] + (d1_times_w_real_multiplier_64bit >> W_REAL_SHIFT) - d2;
		n2++;
        d2 = d1;
        d1 = y;
		sprintf(buf, "n: %hu, d1_times_w_real_multiplier_64bit: %ld, y: %ld, d1: %ld, d2: %ld\n",
                 n, (int32_t)d1_times_w_real_multiplier_64bit, y, d1, d2);
		print_debug_string(buf);
    }

    int64_t d1_times_w_real_multiplier_64bit = (int64_t)d1 * W_REAL_MULTIPLIER;
    int64_t d1_times_w_imag_multiplier_64bit = (int64_t)d1 * W_IMAG_MULTIPLIER;
 
    int32_t d1_times_w_real_multiplier_32bit = (int32_t)(d1_times_w_real_multiplier_64bit >> W_REAL_SHIFT);
    int32_t d1_times_w_imag_multiplier_32bit = (int32_t)(d1_times_w_imag_multiplier_64bit >> W_IMAG_SHIFT);

    result.real = (d1_times_w_real_multiplier_32bit >> 1) - d2;
    result.imag = d1_times_w_imag_multiplier_32bit;
    return result;
}


void process_go_to_closed_loop_data(void)
{
	char buf[100];
//	uint16_t i;
	int32_t ratio;

	if(go_to_closed_loop_step == 2) {
		multipurpose_data_size = GO_TO_CLOSED_LOOP_N_DATA_ITEMS * sizeof(int32_t) + 2 * sizeof(int32_t); // we will send the input data plus the result of the Goertzel algorithm
//		print_debug_string("Go to closed loop data:\n");
//		for(i = 0; i < GO_TO_CLOSED_LOOP_N_DATA_ITEMS; i++) {
//			sprintf(buf, "   %hu: %ld\n", i, go_to_closed_loop_data[i]);
//			print_debug_string(buf);
//		}
		struct goertzel_algorithm_result_t result = goertzel_algorithm(go_to_closed_loop_data, (uint16_t)GO_TO_CLOSED_LOOP_N_DATA_ITEMS);
		go_to_closed_loop_data[GO_TO_CLOSED_LOOP_N_DATA_ITEMS] = result.real;
		go_to_closed_loop_data[GO_TO_CLOSED_LOOP_N_DATA_ITEMS + 1] = result.imag;
		multipurpose_data_type = MULTIPURPOSE_DATA_TYPE_GO_TO_CLOSED_LOOP;
		sprintf(buf, "Goertzel algorithm result: real: %ld, imag: %ld\n", result.real, result.imag);
		print_debug_string(buf);
		int32_t abs_result_real = abs(result.real);
		int32_t abs_result_imag = abs(result.imag);
		if(abs_result_real >= abs_result_imag) {
			if(abs_result_imag == 0) {
				ratio = 32000;
			}
			else {
				ratio = abs_result_real / abs_result_imag;
			}
			if(result.real >= 0) {
				print_debug_string("Angle determined to be very roughly 0 degrees\n");  //                |
				commutation_position_offset += (HALL_TO_POSITION_90_DEGREE_OFFSET * 3); // 3 2 1 0  3 0 1 2
			}
			else {
				print_debug_string("Angle determined to be very roughly 180 degrees\n");
				commutation_position_offset += (HALL_TO_POSITION_90_DEGREE_OFFSET * 1); // 1 0 3 2  1 2 3 0
			}
		}
		else {
			if(abs_result_real == 0) {
				ratio = 32000;
			}
			else {
				ratio = abs_result_imag / abs_result_real;
			}
			if(result.imag >= 0) {
				print_debug_string("Angle determined to be very roughly 90 degrees\n");
				commutation_position_offset += (HALL_TO_POSITION_90_DEGREE_OFFSET * 2); // 0 3 2 1  2 3 0 1
			}
			else {
				print_debug_string("Angle determined to be very roughly 270 degrees\n");
				commutation_position_offset += (HALL_TO_POSITION_90_DEGREE_OFFSET * 0); // 2 1 0 3  0 1 2 3
			}
		}
		sprintf(buf, "Ratio: %ld\n", ratio);
		print_debug_string(buf);

		if(ratio < 3) {
			print_debug_string("Ratio is too small\n");
			if(test_mode != GO_TO_CLOSED_LOOP_MODE_TEST_MODE) {
				fatal_error(39); // "go to closed loop failed" (all error text is defined in error_text.c)
			}
			disable_mosfets();
			set_motor_control_mode(OPEN_LOOP_POSITION_CONTROL);
		}
		else {
			set_motor_control_mode(CLOSED_LOOP_POSITION_CONTROL);
		}

		hall_position = current_position.i32[1]; // this is so that the motor does not move when we go into closed loop mode
		go_to_closed_loop_step = 0;
		motor_busy = 0;
	}
}

#else

void start_go_to_closed_loop_mode(void)
{
//	if(motor_control_mode != OPEN_LOOP_POSITION_CONTROL) {
//		fatal_error(7); // "not in open loop" (all error text is defined in error_text.c)
//	}
	if(n_items_in_queue != 0) {
		fatal_error(8); // "queue not empty" (all error text is defined in error_text.c)
	}
	if(motor_busy) {
		fatal_error(19); // "motor busy" (all error text is defined in error_text.c)
	}

	print_debug_string("Go to closed loop mode start\n");
	check_current_sensor_and_enable_mosfets();

	commutation_position_offset = global_settings.commutation_position_offset;

	set_motor_control_mode(CLOSED_LOOP_POSITION_CONTROL);
}

#endif


#define VIBRATE_STEP_SIZE 1
#define VIBRATE_MAGNITUDE 50
#define VIBRATE_LOOP_COUNTER 200
#define VIBRATE_DATA_INDEX_SHIFT_RIGHT 3
#define VIBRATE_N_DATA_ITEMS ((VIBRATE_LOOP_COUNTER * 2) >> VIBRATE_DATA_INDEX_SHIFT_RIGHT)
#ifdef PRODUCT_NAME_M1
static volatile int32_t *vibrate_data = (void*)&calibration; // vibrate_data uses the same data as calibration (the ligic that takes it to closed loop never runs at the same time as the calibration logic)
#endif

void vibrate(uint8_t vibration_level)
{
#ifdef PRODUCT_NAME_M1
	if(n_items_in_queue != 0) {
		fatal_error(8); // "queue not empty" (all error text is defined in error_text.c)
	}

	if(vibration_level == 0) {
		vibration_active = 0;
		desired_motor_pwm_voltage = 0;
		motor_busy = 0;
		return;
	}

	if(motor_busy) {
		fatal_error(19); // "motor busy" (all error text is defined in error_text.c)
	}

	motor_busy = 1;
	
	check_current_sensor_and_enable_mosfets();

	if(VIBRATE_N_DATA_ITEMS > sizeof(calibration) / sizeof(int32_t)) { // sanity check here. make sure we don't overflow the available memory.
		fatal_error(29); // "debug1" (all error text is defined in error_text.c)
	}
	memset((void*)vibrate_data, 0, VIBRATE_N_DATA_ITEMS * sizeof(int32_t));
	multipurpose_data_type = MULTIPURPOSE_DATA_TYPE_VIBRATE;
	multipurpose_data_size = VIBRATE_N_DATA_ITEMS * sizeof(int32_t);

	TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
	desired_motor_pwm_voltage = VIBRATE_MAGNITUDE;
	vibration_four_step = 0;
	go_to_closed_loop_avg_counter = 0;
	data_index = 0;
	set_motor_control_mode(OPEN_LOOP_POSITION_CONTROL);
	vibration_active = 1;
    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
#endif
}

#ifdef PRODUCT_NAME_M1
void handle_vibrate_logic(void)
{
	vibrate_data[data_index >> VIBRATE_DATA_INDEX_SHIFT_RIGHT] += hall_position_delta;
	data_index++;
	if(go_to_closed_loop_avg_counter < VIBRATE_LOOP_COUNTER) {
		go_to_closed_loop_avg_counter++;
		return;
	}
	go_to_closed_loop_avg_counter = 0;
	vibration_four_step++;
	if((vibration_four_step & 1) == 0) {
		desired_motor_pwm_voltage = VIBRATE_MAGNITUDE;
		data_index = 0;
	}
	else {
		desired_motor_pwm_voltage = -VIBRATE_MAGNITUDE;
	}
}
#endif

#if 0
void handle_vibrate_logic_old(void)
{
	switch(vibration_four_step) {
	case 0:
		desired_motor_pwm_voltage += VIBRATE_STEP_SIZE;
		if(desired_motor_pwm_voltage >= VIBRATE_MAGNITUDE) {
			vibration_four_step++;
		}
		break;
	case 1:
		desired_motor_pwm_voltage -= VIBRATE_STEP_SIZE;
		if(desired_motor_pwm_voltage <= -VIBRATE_MAGNITUDE) {
			vibration_four_step++;
		}
		break;
	case 2:
		desired_motor_pwm_voltage += VIBRATE_STEP_SIZE;
		if(desired_motor_pwm_voltage >= 0) {
			vibration_four_step = 0;
		}
		break;
	}
	go_to_closed_loop_avg_counter++;
}
#endif

void start_capture(uint8_t capture_type)
{
    print_debug_string("Capture start\n");

	TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation

    memset(&capture, 0, sizeof(capture));
    capture.capture_type = capture_type;

    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
}

void capture_logic(void)
{
    int32_t adjusted_hall_sensor_readings[3];
    static uint16_t counter = 0;

    if(capture.capture_type == CAPTURE_HALL_SENSOR_READINGS) {
    	if(counter == 0) {
    		counter = 256;
		    uint16_t hall_data_buffer[4];
    		hall_data_buffer[0] = (get_hall_sensor1_voltage() << 3) - HALL_SENSOR_SHIFT;
    		hall_data_buffer[1] = (get_hall_sensor2_voltage() << 3) - HALL_SENSOR_SHIFT;
    		hall_data_buffer[2] = (get_hall_sensor3_voltage() << 3) - HALL_SENSOR_SHIFT;
			hall_data_buffer[3] = 65535;
			rs485_transmit((char*)hall_data_buffer, sizeof(hall_data_buffer));
    	}
    	counter--;
    }
    else if(capture.capture_type == CAPTURE_HALL_POSITION) {
    	if(counter == 0) {
    		counter = 256;
			get_sensor_position_return_t get_sensor_position_return = get_sensor_position();
    		memcpy(hall_data_buffer, &get_sensor_position_return.position, sizeof(get_sensor_position_return.position));
			rs485_transmit((char*)&get_sensor_position_return.position, sizeof(get_sensor_position_return.position));
    	}
    	counter--;
    }
    else if(capture.capture_type == CAPTURE_ADJUSTED_HALL_SENSOR_READINGS) {
    	if(counter == 0) {
    		counter = 256;
		    uint16_t hall_data_buffer[4];
    		hall_data_buffer[0] = (get_hall_sensor1_voltage() << 3) - HALL_SENSOR_SHIFT;
    		hall_data_buffer[1] = (get_hall_sensor2_voltage() << 3) - HALL_SENSOR_SHIFT;
    		hall_data_buffer[2] = (get_hall_sensor3_voltage() << 3) - HALL_SENSOR_SHIFT;
			adjust_hall_sensor_readings(hall_data_buffer, adjusted_hall_sensor_readings);
			hall_data_buffer[0] = (adjusted_hall_sensor_readings[0] >> 16) + 32768;
			hall_data_buffer[1] = (adjusted_hall_sensor_readings[1] >> 16) + 32768;
			hall_data_buffer[2] = (adjusted_hall_sensor_readings[2] >> 16) + 32768;
			hall_data_buffer[3] = 65535; // magic number to indicate the end of the data
			rs485_transmit((char*)hall_data_buffer, sizeof(hall_data_buffer));
    	}
    	counter--;
    }
}

void start_homing(int32_t max_homing_displacement, uint32_t max_homing_time)
{
	if(motor_control_mode != CLOSED_LOOP_POSITION_CONTROL) {
		fatal_error(13); // "not in closed loop" (all error text is defined in error_text.c)
	}

	if(n_items_in_queue != 0) {
		fatal_error(8); // "queue not empty" (all error text is defined in error_text.c)
	}

	add_trapezoid_move_to_queue(max_homing_displacement, max_homing_time);

	motor_busy = 1;

	homing_active = 1;
}

#define HOMING_MAX_POSITION_ERROR 50000
void handle_homing_logic(void)
{
	int32_t position_error;
	position_error = abs(current_position.i32[1] - hall_position);

	if(position_error > HOMING_MAX_POSITION_ERROR) {
		homing_active = 0;
		clear_the_queue_and_stop_no_disable_interrupt(); // detected a colision so stop where we are
		if(current_position.i32[1] >= hall_position) {
			current_position.i32[1] -= HOMING_MAX_POSITION_ERROR;
		}
		else {
			current_position.i32[1] += HOMING_MAX_POSITION_ERROR;
		}
	}

	if(n_items_in_queue == 0) {
		homing_active = 0;
//		if(homing_direction == -1) {
//			position_lower_safety_limit = current_position.i32[1];
//		}
//		else {
//			position_upper_safety_limit = urrent_position.i32[1];
//		}
		motor_busy = 0;
	}
}


void start_fast_capture_data(void)
{
	print_debug_string("Fast capture data\n");
	fast_capture_data_index = 0;
	fast_capture_data_active = 1;
}


void fast_capture_until_trigger(void)
{
	memset(fast_capture_data, 0, fast_capture_data_size * sizeof(struct fast_capture_data_struct));
	fast_capture_data_index = 0;
	fast_capture_data_active = 2;
}


void print_position(void)
{
	char buf[100];
	sprintf(buf, "current_position: %ld   hall_position: %ld\n", current_position.i32[1], hall_position);
	print_debug_string(buf);
}

void print_PID_data(void)
{
	char buf[170];
	sprintf(buf, "error: %ld   min_PID_error: %ld   max_PID_error: %ld   PID: %ld %ld %ld   Output: %ld\n", PID_error, min_PID_error, max_PID_error, PID_P, PID_I, PID_D, PID_output_value);
	print_debug_string(buf);
}

void print_queue_stats(void)
{
	char buf[100];
	sprintf(buf, "n_items_in_queue: %u\n", (unsigned int)n_items_in_queue);
	print_debug_string(buf);
}

void print_current_movement(void)
{
	char buf[150];
	uint64_t current_time = get_microsecond_time();
	print_int64("max acceleration:", (int64_t)max_acceleration);
	print_int64("max velocity:", (int64_t)max_velocity);
	sprintf(buf, "current_time: %lu\n", (unsigned long int)current_time);
	print_debug_string(buf);
	sprintf(buf, "motor_control_mode: %u\n", (unsigned int)motor_control_mode);
	print_debug_string(buf);
}


void print_velocity(void)
{
	char buf[150];
//	sprintf(buf, "desired velocity: %ld   actual velocity: %ld\n", desired_velocity, velocity);
	sprintf(buf, "velocity: %ld\n", velocity);
	print_debug_string(buf);
}


void print_time_difference(void)
{
	char buf[150];

	snprintf(buf, sizeof(buf), "ALL_MOTOR_CONTROL_CALULATIONS_PROFILER: latest: %hu max: %hu\n",
	        profiler_get_time_difference(ALL_MOTOR_CONTROL_CALULATIONS_PROFILER), profiler_get_max_time_difference(ALL_MOTOR_CONTROL_CALULATIONS_PROFILER));
	print_debug_string(buf);

#ifdef DO_DETAILED_PROFILING
	snprintf(buf, sizeof(buf), "GET_SENSOR_POSITION_PROFILER: latest: %hu max: %hu\n",
	        profiler_get_time_difference(GET_SENSOR_POSITION_PROFILER), profiler_get_max_time_difference(GET_SENSOR_POSITION_PROFILER));
	print_debug_string(buf);

	snprintf(buf, sizeof(buf), "COMPUTE_VELOCITY_PROFILER: latest: %hu max: %hu\n",
	        profiler_get_time_difference(COMPUTE_VELOCITY_PROFILER), profiler_get_max_time_difference(COMPUTE_VELOCITY_PROFILER));
	print_debug_string(buf);

	snprintf(buf, sizeof(buf), "MOTOR_MOVEMENT_CALCULATIONS_PROFILER: latest: %hu max: %hu\n",
	        profiler_get_time_difference(MOTOR_MOVEMENT_CALCULATIONS_PROFILER), profiler_get_max_time_difference(MOTOR_MOVEMENT_CALCULATIONS_PROFILER));
	print_debug_string(buf);

	snprintf(buf, sizeof(buf), "MOTOR_PHASE_CALCULATIONS_PROFILER: latest: %hu max: %hu\n",
	        profiler_get_time_difference(MOTOR_PHASE_CALCULATIONS_PROFILER), profiler_get_max_time_difference(MOTOR_PHASE_CALCULATIONS_PROFILER));
	print_debug_string(buf);
#endif

	snprintf(buf, sizeof(buf), "MOTOR_CONTROL_LOOP_PERIOD_PROFILER: latest: %hu max: %hu\n",
	profiler_get_time_difference(MOTOR_CONTROL_LOOP_PERIOD_PROFILER), profiler_get_max_time_difference(MOTOR_CONTROL_LOOP_PERIOD_PROFILER));
	print_debug_string(buf);

	print_debug_counter();
}


void print_hall_position_delta_stats(void)
{
	char buf[150];
	int32_t ahpd = average_hall_position_delta;
	int32_t ahpd_count = average_hall_position_delta_count;
	ahpd /= ahpd_count;
	sprintf(buf, "max_hall_position_delta: %ld   min_hall_position_delta: %ld  avg_hall_position_delta: %ld\n", max_hall_position_delta, min_hall_position_delta, ahpd);
	print_debug_string(buf);
	max_hall_position_delta = -2000000000;
	min_hall_position_delta = 2000000000;
	average_hall_position_delta = 0;
	average_hall_position_delta_count = 0;
}

void print_max_motor_current_settings(void)
{
	char buf[150];
	sprintf(buf, "Maximum motor pwm voltage: %hu   Maximum motor regeneration pwm voltage: %hu\n", max_motor_pwm_voltage, max_motor_regen_pwm_voltage);
	print_debug_string(buf);
}


void print_commutation_position_offset(void)
{
	char buf[100];
	if(!global_settings.motor_phases_reversed) {
		sprintf(buf, "Commutation position offset: %lu (phases not reversed)\n", commutation_position_offset);
	}
	else {
		sprintf(buf, "Commutation position offset: %lu (phases reversed)\n", commutation_position_offset);
	}
	print_debug_string(buf);
}


void print_motor_current(void)
{
	char buf[150];
	int16_t current = get_motor_current();
	sprintf(buf, "current: %hd   motor_current_baseline: %hu\n", current, motor_current_baseline);
	print_debug_string(buf);
}


void print_hall_sensor_data(void)
{
	char buf[100];
	uint16_t hall1 = get_hall_sensor1_voltage();
	uint16_t hall2 = get_hall_sensor2_voltage();
	uint16_t hall3 = get_hall_sensor3_voltage();

	sprintf(buf, "hall1: %hu   hall2: %hu   hall3: %hu\n", hall1, hall2, hall3);
	print_debug_string(buf);

	if(!global_settings.motor_phases_reversed) {
		sprintf(buf, "hall_position: %ld   commutation_position_offset: %lu (phases not reversed)\n", hall_position, commutation_position_offset);
	}
	else {
		sprintf(buf, "hall_position: %ld   commutation_position_offset: %lu (phases reversed)\n", hall_position, commutation_position_offset);
	}
	print_debug_string(buf);
}


void print_motor_status(void)
{
	char buf[100];

	uint8_t motor_status_flags = get_motor_status_flags();
	sprintf(buf, "status: %hu\n", motor_status_flags);
	print_debug_string(buf);
}


void print_motor_pwm_voltage(void)
{
	char buf[100];
	sprintf(buf, "motor_pwm_voltage: %d\n", (int)motor_pwm_voltage);
	print_debug_string(buf);
}


uint8_t is_fast_capture_data_result_ready(void)
{
	return fast_capture_data_result_ready;
}


void print_fast_capture_data_result(void)
{
	uint16_t i;
	char buf[100];

	for(i = 0; i < fast_capture_data_size; i++) {
		sprintf(buf, "%hu %hu %hu %hu\n", fast_capture_data[fast_capture_data_index].hall1, fast_capture_data[fast_capture_data_index].hall2,
		                                  fast_capture_data[fast_capture_data_index].hall3, fast_capture_data[fast_capture_data_index].hall_position_16bit);
		print_debug_string(buf);
		fast_capture_data_index++;
		if(fast_capture_data_index >= fast_capture_data_size) {
			fast_capture_data_index = 0;
		}
	}

	fast_capture_data_result_ready = 0;
}

#define TURN_POINT_CALCULATION_SHIFT 4

void add_to_queue(int32_t parameter, uint32_t n_time_steps, movement_type_t movement_type)
{	
	int64_t predicted_final_velocity;
	int64_t predicted_final_position;
//	char buf[150];

	if(motor_busy) {
		fatal_error(19); // "motor busy" (all error text is defined in error_text.c)
	}

	if(n_time_steps == 0) {
		return; // in the case that the number if time steps is zero, it makes sense to not add anything to the queue
	}
    if(n_items_in_queue < MOVEMENT_QUEUE_SIZE) {
		movement_queue[queue_write_position].movement_type = movement_type;
		if(movement_type == MOVE_WITH_ACCELERATION) {
	        movement_queue[queue_write_position].acceleration = parameter;
    	    movement_queue[queue_write_position].acceleration <<= ACCELERATION_SHIFT_LEFT;
            if(abs(movement_queue[queue_write_position].acceleration) > max_acceleration) {
	            fatal_error(15); // "accel too high" (all error text is defined in error_text.c)
            }
			predicted_final_velocity = velocity_after_last_queue_item + movement_queue[queue_write_position].acceleration * n_time_steps;
//			sprintf(buf, "Predicted final velocity: %ld\n", (int32_t)(predicted_final_velocity >> 32));
//			print_debug_string(buf);
			if(abs(predicted_final_velocity) > max_velocity) {
				fatal_error(28); // "predicted velocity too high" (all error text is defined in error_text.c)
			}
			predicted_final_position = position_after_last_queue_item + velocity_after_last_queue_item * n_time_steps + movement_queue[queue_write_position].acceleration * (((uint64_t)n_time_steps * (n_time_steps + 1)) >> 1);
//			sprintf(buf, "Predicted final position: %ld\n", (int32_t)(predicted_final_position >> 32));
//			print_debug_string(buf);
			if((((int32_t*)&predicted_final_position)[1] < position_lower_safety_limit) || (((int32_t*)&predicted_final_position)[1] > position_upper_safety_limit)) {
				fatal_error(27); // "predicted position out of safety zone" (all error text is defined in error_text.c)
			}
			if(movement_queue[queue_write_position].acceleration == 0) {
//				print_debug_string("No turn point (acceleration == 0)\n");
			}
			else {
				int64_t time_step_at_turn_point_shifted = -(int64_t)((velocity_after_last_queue_item << TURN_POINT_CALCULATION_SHIFT) / movement_queue[queue_write_position].acceleration);
//				sprintf(buf, "time_at_turn_point: %lu\n", (uint32_t)(time_step_at_turn_point_shifted >> TURN_POINT_CALCULATION_SHIFT));
//				print_debug_string(buf);
				if((time_step_at_turn_point_shifted > 0) && ((time_step_at_turn_point_shifted >> TURN_POINT_CALCULATION_SHIFT) < n_time_steps)) {
					int64_t relative_position_at_turn_point = (int64_t)(velocity_after_last_queue_item * (int64_t)((int64_t)time_step_at_turn_point_shifted - (int64_t)(1 << TURN_POINT_CALCULATION_SHIFT))) >> (TURN_POINT_CALCULATION_SHIFT + 1);
//					sprintf(buf, "relative_position_at_turn_point: %ld\n", (int32_t)(relative_position_at_turn_point >> 32));
//					print_debug_string(buf);
					int64_t absolute_position_at_turn_point = position_after_last_queue_item + relative_position_at_turn_point;
					if((((int32_t*)&absolute_position_at_turn_point)[1] < position_lower_safety_limit) || (((int32_t*)&absolute_position_at_turn_point)[1] > position_upper_safety_limit)) {
						fatal_error(26); // "turn point out of safety zone" (all error text is defined in error_text.c)
					}
				}
				else {
//					print_debug_string("No turn point\n");
				}
			}
		}
		else {
	        movement_queue[queue_write_position].velocity = parameter;
    	    movement_queue[queue_write_position].velocity <<= VELOCITY_SHIFT_LEFT;
			predicted_final_velocity = movement_queue[queue_write_position].velocity;
//			sprintf(buf, "Predicted final velocity: %ld\n", ((int32_t*)&predicted_final_velocity)[1]);
//			print_debug_string(buf);
            if(abs(movement_queue[queue_write_position].velocity) > max_velocity) {
	            fatal_error(16); // "vel too high" (all error text is defined in error_text.c)
            }
			predicted_final_position = position_after_last_queue_item + movement_queue[queue_write_position].velocity * n_time_steps;
//			sprintf(buf, "Predicted final position: %ld\n", (int32_t)(predicted_final_position >> 32));
//			print_debug_string(buf);
		}
		position_after_last_queue_item = predicted_final_position;
		velocity_after_last_queue_item = predicted_final_velocity;
        movement_queue[queue_write_position].n_time_steps = n_time_steps;
        queue_write_position = (queue_write_position + 1) & (MOVEMENT_QUEUE_SIZE - 1);
        n_items_in_queue++;
    }
	else {
		fatal_error(17); // "queue is full" (all error text is defined in error_text.c)
	}
}


void add_to_queue_test(int32_t parameter, uint32_t n_time_steps, movement_type_t movement_type, add_to_queue_test_results_t *results)
{	
	int64_t movement_queue_queue_write_position_acceleration;
	int64_t movement_queue_queue_write_position_velocity;
	int64_t predicted_final_velocity;
	int64_t predicted_final_position;
	int64_t time_step_at_turn_point_shifted = 0;
	int64_t relative_position_at_turn_point = 0;
	char buf[150];

	memset(results, 0, sizeof(add_to_queue_test_results_t));

	if(n_time_steps == 0) {
		return; // in the case that the number if time steps is zero, it makes sense to not add anything to the queue
	}
	if(movement_type == MOVE_WITH_ACCELERATION) {
		movement_queue_queue_write_position_acceleration = parameter;
		movement_queue_queue_write_position_acceleration <<= ACCELERATION_SHIFT_LEFT;
		if(abs(movement_queue_queue_write_position_acceleration) > max_acceleration) {
			fatal_error(15); // "accel too high" (all error text is defined in error_text.c)
		}
		predicted_final_velocity = velocity_after_last_queue_item + movement_queue_queue_write_position_acceleration * n_time_steps;
		sprintf(buf, "Predicted final velocity: %ld\n", (int32_t)(predicted_final_velocity >> 32));
		print_debug_string(buf);
		if(abs(predicted_final_velocity) > max_velocity) {
			fatal_error(28); // "predicted velocity too high" (all error text is defined in error_text.c)
		}
		predicted_final_position = position_after_last_queue_item + velocity_after_last_queue_item * n_time_steps + movement_queue_queue_write_position_acceleration * (((uint64_t)n_time_steps * (n_time_steps + 1)) >> 1);
		sprintf(buf, "Predicted final position: %ld\n", (int32_t)(predicted_final_position >> 32));
		print_debug_string(buf);
		if((((int32_t*)&predicted_final_position)[1] < position_lower_safety_limit) || (((int32_t*)&predicted_final_position)[1] > position_upper_safety_limit)) {
			fatal_error(27); // "predicted position out of safety zone" (all error text is defined in error_text.c)
		}
		if(movement_queue_queue_write_position_acceleration == 0) {
			print_debug_string("No turn point (acceleration == 0)\n");
		}
		else {
			time_step_at_turn_point_shifted = -(int64_t)((velocity_after_last_queue_item << TURN_POINT_CALCULATION_SHIFT) / movement_queue_queue_write_position_acceleration);
			sprintf(buf, "time_at_turn_point: %lu\n", (uint32_t)(time_step_at_turn_point_shifted >> TURN_POINT_CALCULATION_SHIFT));
			print_debug_string(buf);
			if((time_step_at_turn_point_shifted > 0) && ((time_step_at_turn_point_shifted >> TURN_POINT_CALCULATION_SHIFT) < n_time_steps)) {
				relative_position_at_turn_point = (int64_t)(velocity_after_last_queue_item * (int64_t)((int64_t)time_step_at_turn_point_shifted - (int64_t)(1 << TURN_POINT_CALCULATION_SHIFT))) >> (TURN_POINT_CALCULATION_SHIFT + 1);
				sprintf(buf, "relative_position_at_turn_point: %ld\n", (int32_t)(relative_position_at_turn_point >> 32));
				print_debug_string(buf);
				int64_t absolute_position_at_turn_point = position_after_last_queue_item + relative_position_at_turn_point;
				if((((int32_t*)&absolute_position_at_turn_point)[1] < position_lower_safety_limit) || (((int32_t*)&absolute_position_at_turn_point)[1] > position_upper_safety_limit)) {
					fatal_error(26); // "turn point out of safety zone" (all error text is defined in error_text.c)
				}
			}
			else {
				print_debug_string("No turn point\n");
			}
		}
	}
	else {
		movement_queue_queue_write_position_velocity = parameter;
		movement_queue_queue_write_position_velocity <<= VELOCITY_SHIFT_LEFT;
		predicted_final_velocity = movement_queue_queue_write_position_velocity;
		sprintf(buf, "Predicted final velocity: %ld\n", ((int32_t*)&predicted_final_velocity)[1]);
		print_debug_string(buf);
		if(abs(movement_queue_queue_write_position_velocity) > max_velocity) {
			fatal_error(16); // "vel too high" (all error text is defined in error_text.c)
		}
		predicted_final_position = position_after_last_queue_item + movement_queue_queue_write_position_velocity * n_time_steps;
		sprintf(buf, "Predicted final position: %ld\n", (int32_t)(predicted_final_position >> 32));
		print_debug_string(buf);
	}
	position_after_last_queue_item = predicted_final_position;
	velocity_after_last_queue_item = predicted_final_velocity;

	results->predicted_final_velocity = ((int32_t*)&predicted_final_velocity)[1];
	results->predicted_final_position = ((int32_t*)&predicted_final_position)[1];
	results->time_step_at_turn_point = (time_step_at_turn_point_shifted >> TURN_POINT_CALCULATION_SHIFT);
	results->relative_position_at_turn_point = ((int32_t*)&relative_position_at_turn_point)[1];
}

void move_n_steps_in_m_time(int32_t displacement, uint32_t time_delta)
{
//	uint64_t local_time = get_microsecond_time();
//	add_to_queue(desired_position - displacement, local_time + time_delta);
}


uint8_t get_n_items_in_queue(void)
{
	return n_items_in_queue;
}


void clear_the_queue_and_stop(void)
{
    TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
    clear_the_queue_and_stop_no_disable_interrupt();
    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
}


void add_trapezoid_move_to_queue(int32_t total_displacement, uint32_t total_time)
{
	int32_t acceleration;
	uint32_t delta_t1;
	uint32_t delta_t2;

	compute_trapezoid_move(total_displacement, total_time, &acceleration, &delta_t1, &delta_t2);

	add_to_queue(acceleration, delta_t1, MOVE_WITH_ACCELERATION);
	add_to_queue(0, delta_t2, MOVE_WITH_ACCELERATION);
	add_to_queue(-acceleration, delta_t1, MOVE_WITH_ACCELERATION);
}


void add_go_to_position_to_queue(int32_t absolute_position, uint32_t move_time)
{
	int32_t acceleration;
	uint32_t delta_t1;
	uint32_t delta_t2;

	// This command only supports the case of adding a move when the queue is fully empty. Let's make sure of that.
	if(n_items_in_queue != 0) {
		fatal_error(8); // "queue not empty" (all error text is defined in error_text.c)
	}
	int32_t total_displacement = absolute_position - current_position.i32[1];
	compute_trapezoid_move(total_displacement, move_time, &acceleration, &delta_t1, &delta_t2);

	add_to_queue(acceleration, delta_t1, MOVE_WITH_ACCELERATION);
	add_to_queue(0, delta_t2, MOVE_WITH_ACCELERATION);
	add_to_queue(-acceleration, delta_t1, MOVE_WITH_ACCELERATION);
}


//#define VELOCITY_AVERAGING_SHIFT_RIGHT 8
//#define VELOCITY_AVERAGING_TIME_STEPS ((1 << VELOCITY_AVERAGING_SHIFT_RIGHT) - 1) // keep this maximum 255 so that our n_position_history_items counter does not overflow
#define VELOCITY_AVERAGING_TIME_STEPS 20

static int16_t position_history[VELOCITY_AVERAGING_TIME_STEPS];
static uint8_t position_history_index = 0;
static uint8_t n_position_history_items = 0;
void compute_velocity(void)
{
	uint16_t latest_hall_position = (uint16_t)(((uint32_t)(hall_position + COMMUTATION_POSITION_OFFSET_DEFAULT)) & 0xffff);
	uint16_t old_recorded_hall_position = position_history[position_history_index];
	int32_t computed_velocity = 0;
	position_history[position_history_index] = latest_hall_position;
	position_history_index++;
	if(position_history_index >= VELOCITY_AVERAGING_TIME_STEPS) {
		position_history_index = 0;
	}
	if(n_position_history_items < VELOCITY_AVERAGING_TIME_STEPS) {
		n_position_history_items++;
	}
	if(n_position_history_items == VELOCITY_AVERAGING_TIME_STEPS) {
		uint16_t computed_velocity_uint16 = latest_hall_position - old_recorded_hall_position;
		if(computed_velocity_uint16 <= 32767) {
			computed_velocity = (int32_t)computed_velocity_uint16;
		}
		else {
			computed_velocity = (int32_t)computed_velocity_uint16 - 65536;
		}
		computed_velocity = (computed_velocity * COMPUTED_VELOCITY_CALIBRATION_CONSTANT) >> COMPUTED_VELOCITY_CALIBRATION_SHIFT; 
	}
	velocity = computed_velocity;
//	velocity_moving_average = (VELOCITY_AVERAGING_TIME_STEPS * velocity_moving_average + (hall_position_delta << 8) + (1 << (VELOCITY_AVERAGING_SHIFT_RIGHT - 1))) >> VELOCITY_AVERAGING_SHIFT_RIGHT;
//	velocity = ((velocity_moving_average + (1 << (8 - 1))) >> 8);
//	velocity = 0; // DEBUG
}

uint8_t handle_queued_movements(void)
{
	if(!decelerate_to_stop_active) {
		if(n_items_in_queue > 0) {
			// there is an assumption here that any item in the queue will always have one or more time steps
			// see the add_to_queue function where we make sure to never add an item to the queue with zero time steps
			if(movement_queue[queue_read_position].movement_type == MOVE_WITH_ACCELERATION) {
				current_velocity_i64 += movement_queue[queue_read_position].acceleration; // consume one time step worth of acceleration
				movement_queue[queue_read_position].n_time_steps--;
				if(movement_queue[queue_read_position].n_time_steps == 0) {
					queue_read_position = (queue_read_position + 1) & (MOVEMENT_QUEUE_SIZE - 1);
					n_items_in_queue--;
				}
			}
			else {
				current_velocity_i64 = movement_queue[queue_read_position].velocity; // velocity is constant during this time step
				movement_queue[queue_read_position].n_time_steps--;
				if(movement_queue[queue_read_position].n_time_steps == 0) {
					queue_read_position = (queue_read_position + 1) & (MOVEMENT_QUEUE_SIZE - 1);
					n_items_in_queue--;
				}
			}
		}
		else {
			decelerate_to_stop_active = 1;
		}
	}
	if(decelerate_to_stop_active) {
		if(current_velocity_i64 != 0) {
			fatal_error(18); // "run out of queue items" (all error text is defined in error_text.c)
		}
//		if(current_velocity_i64 >= 0) {
//			if(current_velocity_i64 > max_acceleration) {
//				current_velocity_i64 -= max_acceleration;
//			}
//			else {
//				current_velocity_i64 = 0;
//			}
//		}
//		else {
//			if(-current_velocity_i64 > max_acceleration) {
//				current_velocity_i64 += max_acceleration;
//			}
//			else {
//				current_velocity_i64 = 0;
//			}
//		}
		if(current_velocity_i64 == 0) {
			decelerate_to_stop_active = 0;
		}
	}

	if((current_velocity_i64 > max_velocity) || (current_velocity_i64 < -max_velocity)) {
		fatal_error(16); // "vel too high" (all error text is defined in error_text.c)
	}
	current_position.i64 += current_velocity_i64;

	return (current_velocity_i64 != 0);

}


#define ERROR_HYSTERESIS_P 0
#define ERROR_HYSTERESIS_D 0

#ifdef PRODUCT_NAME_M1
#define PID_SHIFT_RIGHT 18
#define PROPORTIONAL_CONSTANT_PID 5000
#define INTEGRAL_CONSTANT_PID     1
#define DERIVATIVE_CONSTANT_PID   5000
#endif
#ifdef PRODUCT_NAME_M2
#define PID_SHIFT_RIGHT 18
#define PROPORTIONAL_CONSTANT_PID 20000
#define INTEGRAL_CONSTANT_PID     1
#define DERIVATIVE_CONSTANT_PID   100000
#endif
#ifdef PRODUCT_NAME_M3
#define PID_SHIFT_RIGHT 14
#define PROPORTIONAL_CONSTANT_PID 9000
#define INTEGRAL_CONSTANT_PID     100
#define DERIVATIVE_CONSTANT_PID   1000000
#endif
#ifdef PRODUCT_NAME_M4
#define PID_SHIFT_RIGHT 14
#define PROPORTIONAL_CONSTANT_PID 10000
#define INTEGRAL_CONSTANT_PID     10
#define DERIVATIVE_CONSTANT_PID   100000
#endif

#define MAX_INT32         2147483647
#define MAX_I_TERM (MAX_PWM_VOLTAGE_AT_ZERO_VELOCITY << PID_SHIFT_RIGHT) // ~17000000
#define MAX_PD_TERMS   ((MAX_INT32 - MAX_I_TERM) / 2) // ~1065000000
//#define MAX_ERROR         ((MAX_PD_TERMS / PROPORTIONAL_CONSTANT_PID) - ERROR_HYSTERESIS_P) // ~106500
#define MAX_ERROR 106500
//#define MAX_ERROR_CHANGE  ((MAX_PD_TERMS / DERIVATIVE_CONSTANT_PID) - ERROR_HYSTERESIS_D)
#if DERIVATIVE_CONSTANT_PID == 0
#define MAX_ERROR_CHANGE 0
#else
#define MAX_ERROR_CHANGE  (MAX_PD_TERMS / DERIVATIVE_CONSTANT_PID)  // ~10650
#endif
#define DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT 5
#define DERIVATIVE_CONSTANT_AVERAGING_SCALAR (1 << DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT)
static int32_t proportional_constant_pid = PROPORTIONAL_CONSTANT_PID;
static int32_t integral_constant_pid = INTEGRAL_CONSTANT_PID;
static int32_t derivative_constant_pid_scaled_for_averaging = DERIVATIVE_CONSTANT_PID >> DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT;
static int32_t max_error = MAX_ERROR;
static int32_t max_error_change = MAX_ERROR_CHANGE;


void set_pid_constants(uint32_t p, uint32_t i, uint32_t d)
{
	int32_t max_pd_terms = ((MAX_INT32 - MAX_I_TERM) / 2); // there are maximum values for the three terms. we do this so that we can calculate everything within a int32_t and don't overflow the math
	int32_t new_max_error = ((max_pd_terms / p) - ERROR_HYSTERESIS_P);
	int32_t new_max_error_change = 0;
	if (d != 0) {
		new_max_error_change = (max_pd_terms / d);
	}
	__disable_irq();
	proportional_constant_pid = (int32_t)p;
	integral_constant_pid = (int32_t)i;
	derivative_constant_pid_scaled_for_averaging = (int32_t)(d >> DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT);
	max_error = new_max_error;
	max_error_change = new_max_error_change;
	__enable_irq();
}


int32_t PID_controller(int32_t error)
{
    int32_t output_value;
    int32_t proportional_term;
    int32_t derivative_term;

	if(error < min_PID_error) {
		min_PID_error = error;
	}
	if(error > max_PID_error) {
		max_PID_error = error;
	}

#ifdef PRODUCT_NAME_M2
    error >>= 3;
#endif

    if (derivative_constant_pid_scaled_for_averaging != 0) {
		int32_t error_change = error - previous_error;
		if(error_change > max_error_change) {
			error_change = max_error_change;
		}
		else if(error_change < -max_error_change) {
			error_change = -max_error_change;
		}
		low_pass_filtered_error_change = (low_pass_filtered_error_change * (DERIVATIVE_CONSTANT_AVERAGING_SCALAR - 1)) >> DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT;
		low_pass_filtered_error_change += error_change;
		derivative_term = low_pass_filtered_error_change * derivative_constant_pid_scaled_for_averaging;
	}
	else {
		derivative_term = 0;
	}
    previous_error = error;

    if(error < -max_error) {
        error = -max_error;
    }
    else if(error > max_error) {
        error = max_error;
    }
    integral_term += (error * integral_constant_pid);
    if(integral_term > (max_motor_pwm_voltage << PID_SHIFT_RIGHT)) {
    	integral_term = max_motor_pwm_voltage << PID_SHIFT_RIGHT;
    }
    else if(integral_term < -(max_motor_pwm_voltage << PID_SHIFT_RIGHT)) {
    	integral_term = -(max_motor_pwm_voltage << PID_SHIFT_RIGHT);
    }
    proportional_term = error * proportional_constant_pid;

    output_value = (integral_term + proportional_term + derivative_term) >> PID_SHIFT_RIGHT;

	// copy this information to these variables so that we can print them in the debugging interface
	PID_error = error;
	PID_P = proportional_term;
	PID_I = integral_term;
	PID_D = derivative_term;
	PID_output_value = output_value;

    return output_value;
}


int32_t PID_controller_with_hysteresis(int32_t error)
{
	static int32_t error_with_hysteresis_p;
	static int32_t error_with_hysteresis_d;
    int32_t output_value;
    int32_t proportional_term;
    int32_t derivative_term;

	// make sure the error is winin some range to prevent overlow of the math
    if(error < -max_error) {
        error = -max_error;
    }
    else if(error > max_error) {
        error = max_error;
    }

	// calculate the error with hysteresis to be used in the proportional term of the PID controller
	if(error - (ERROR_HYSTERESIS_P >> 1) > error_with_hysteresis_p) {
		error_with_hysteresis_p = error - (ERROR_HYSTERESIS_P >> 1);
	}
	else if(error + (ERROR_HYSTERESIS_P >> 1) < error_with_hysteresis_p) {
		error_with_hysteresis_p = error + (ERROR_HYSTERESIS_P >> 1);
	}

	// calculate the error with hysteresis to be used in the derivative term of the PID controller
	if(error - (ERROR_HYSTERESIS_D >> 1) > error_with_hysteresis_d) {
		error_with_hysteresis_d = error - (ERROR_HYSTERESIS_D >> 1);
	}
	else if(error + (ERROR_HYSTERESIS_D >> 1) < error_with_hysteresis_d) {
		error_with_hysteresis_d = error + (ERROR_HYSTERESIS_D >> 1);
	}

	// calculate the integral term of the PID controller
    integral_term += (error * integral_constant_pid);
    if(integral_term > (max_motor_pwm_voltage << PID_SHIFT_RIGHT)) {
    	integral_term = max_motor_pwm_voltage << PID_SHIFT_RIGHT;
    }
    else if(integral_term < -(max_motor_pwm_voltage << PID_SHIFT_RIGHT)) {
    	integral_term = -(max_motor_pwm_voltage << PID_SHIFT_RIGHT);
    }

	// calculate the proportional term of the PID controller
    proportional_term = error_with_hysteresis_p * proportional_constant_pid;

    if (derivative_constant_pid_scaled_for_averaging != 0) {
		// calculate the derivative term of the PID controller
		int32_t error_change = error_with_hysteresis_d - previous_error;
		if(error_change > max_error_change) {
			error_change = max_error_change;
		}
		if(error_change < -max_error_change) {
			error_change = -max_error_change;
		}
		low_pass_filtered_error_change = (low_pass_filtered_error_change * (DERIVATIVE_CONSTANT_AVERAGING_SCALAR - 1)) >> DERIVATIVE_CONSTANT_AVERAGING_SCALAR_SHIFT;
		low_pass_filtered_error_change += error_change;
		derivative_term = low_pass_filtered_error_change * derivative_constant_pid_scaled_for_averaging;
	}
	else {
		derivative_term = 0;
	}
	previous_error = error_with_hysteresis_d;
    // maximum value are (approximately):
    // integral term:       66000000
    // proportional term:  250000000
    // derivative term:   1250000000
    // sum:               1510000000
    // Make sure it does not exceed the max int32_t = 2147483647
    // After shifting, the maximum values are:
    // integral term:      250
    // proportional term:  953
    // derivative term:   4768
    // sum:               6971
	// sum together the P, I, and D terms to get the final output value
    output_value = (integral_term + proportional_term + derivative_term) >> PID_SHIFT_RIGHT;

//    if(output_value < -max_motor_pwm_voltage) {
//        output_value = -max_motor_pwm_voltage;
//    }
//    else if(output_value > max_motor_pwm_voltage) {
//        output_value = max_motor_pwm_voltage;
//    }

    return output_value;
}


void motor_movement_calculations(void)
{
	uint8_t moving = 0; // 1 indicates that the motor is moving, 0 indicates that it is stopped

#ifdef PRODUCT_NAME_M1
    if(go_to_closed_loop_step != 0) {
        go_to_closed_loop_mode_logic();
    }
    else
#endif
	if(homing_active) {
        handle_homing_logic();
    }
    else if(calibration_step != 0) {
        handle_calibration_logic();
    }
#ifdef PRODUCT_NAME_M1
	else if(vibration_active) {
		handle_vibrate_logic();
	}
#endif

    moving = handle_queued_movements();

	if( (current_position.i32[1] > position_upper_safety_limit) || (current_position.i32[1] < position_lower_safety_limit) ) {
		fatal_error(25); // "safety limit exceeded" (all error text is defined in error_text.c)
	}

#if defined(PRODUCT_NAME_M3) || defined(PRODUCT_NAME_M4)
	if(motor_control_mode == OPEN_LOOP_POSITION_CONTROL) {
		commutation_position = current_position.i32[1] + commutation_position_offset;
		if(calibration_step != 0 ) {
			motor_pwm_voltage = desired_motor_pwm_voltage;
		}
		else if(moving) {
			motor_pwm_voltage = max_motor_pwm_voltage;
		}
		else {
			motor_pwm_voltage = max_motor_pwm_voltage >> 1;
		}
	}
	else {
		commutation_position = sensor_position + commutation_position_offset;
		if(motor_control_mode == CLOSED_LOOP_POSITION_CONTROL) {
			desired_motor_pwm_voltage = PID_controller(current_position.i32[1] - hall_position);
			if(desired_motor_pwm_voltage > HALL_TO_POSITION_90_DEGREE_OFFSET) {
				commutation_position += HALL_TO_POSITION_90_DEGREE_OFFSET;
			}
			else if(desired_motor_pwm_voltage < -HALL_TO_POSITION_90_DEGREE_OFFSET) {
				commutation_position -= HALL_TO_POSITION_90_DEGREE_OFFSET;
			}
			else {
				commutation_position += desired_motor_pwm_voltage;
			}
			if(desired_motor_pwm_voltage >= 0) {
				motor_pwm_voltage = desired_motor_pwm_voltage;
			}
			else {
				motor_pwm_voltage = -desired_motor_pwm_voltage;
			}
			if(motor_pwm_voltage > max_motor_pwm_voltage) {
				motor_pwm_voltage = max_motor_pwm_voltage;
			}
		}
		else {
			if(desired_motor_pwm_voltage > 0) {
				commutation_position += HALL_TO_POSITION_90_DEGREE_OFFSET;
				motor_pwm_voltage = desired_motor_pwm_voltage;
			}
			else if(desired_motor_pwm_voltage < 0) {
				commutation_position -= HALL_TO_POSITION_90_DEGREE_OFFSET;
				motor_pwm_voltage = -desired_motor_pwm_voltage;
			}
			else {
				motor_pwm_voltage = 0;
			}
		}
	}
#else
	if(motor_control_mode == OPEN_LOOP_POSITION_CONTROL) {
		commutation_position = current_position.i32[1] + commutation_position_offset;
		if(vibration_active) {
			if(desired_motor_pwm_voltage >= 0) {
				commutation_position += HALL_TO_POSITION_90_DEGREE_OFFSET;
				motor_pwm_voltage = desired_motor_pwm_voltage;
			}
			else {
				commutation_position -= HALL_TO_POSITION_90_DEGREE_OFFSET;
				motor_pwm_voltage = -desired_motor_pwm_voltage;
			}
		}
		else if(calibration_step != 0 ) {
			motor_pwm_voltage = desired_motor_pwm_voltage;
		}
		else if(moving) {
			motor_pwm_voltage = max_motor_pwm_voltage;
		}
		else {
			motor_pwm_voltage = max_motor_pwm_voltage >> 1;
		}
	}
	else {
		commutation_position = sensor_position + commutation_position_offset;
		if(motor_control_mode == CLOSED_LOOP_POSITION_CONTROL) {
			motor_pwm_voltage = PID_controller(current_position.i32[1] - hall_position);
			int32_t back_emf_voltage = (velocity * VOLTS_PER_ROTATIONAL_VELOCITY) >> 8;
			back_emf_voltage = 0;
			int32_t motor_max_allowed_pwm_voltage = back_emf_voltage + max_motor_pwm_voltage;
			int32_t motor_min_allowed_pwm_voltage = back_emf_voltage - max_motor_pwm_voltage;
			if(motor_pwm_voltage > motor_max_allowed_pwm_voltage) {
				motor_pwm_voltage = motor_max_allowed_pwm_voltage;
			}
			if(motor_pwm_voltage < motor_min_allowed_pwm_voltage) {
				motor_pwm_voltage = motor_min_allowed_pwm_voltage;
			}
			if(motor_pwm_voltage >= 0) {
				commutation_position += HALL_TO_POSITION_90_DEGREE_OFFSET;
			}
			else {
				commutation_position -= HALL_TO_POSITION_90_DEGREE_OFFSET;
				motor_pwm_voltage = -motor_pwm_voltage;
			}
		}
		else {
			if(desired_motor_pwm_voltage >= 0) {
				commutation_position += HALL_TO_POSITION_90_DEGREE_OFFSET;
				motor_pwm_voltage = desired_motor_pwm_voltage;
			}
			else {
				commutation_position -= HALL_TO_POSITION_90_DEGREE_OFFSET;
				motor_pwm_voltage = -desired_motor_pwm_voltage;
			}
		}
	}
#endif
}


#if !defined(PRODUCT_NAME_M3) && !defined(PRODUCT_NAME_M4)
void motor_phase_calculations(void)
{
    static uint16_t commutation_step = 0;
    static uint16_t commutation_sub_step = 0;
    static uint32_t phase1;
    static uint32_t phase2;
    static uint32_t phase3;
    static int32_t phase1_slope;
    static int32_t phase2_slope;
    static int32_t phase3_slope;
    static int32_t tmp32bit;

	if(is_mosfets_enabled() == 0) {
		TIM1->CCR1 = 65535; // set all PWMs high at first (as a workaround to a problem with the MOSFET gate driver that causes high side and low side MOSFETS to turn on at the same time at the instant that the switch disable line goes high)
		TIM1->CCR2 = 65535;
		TIM1->CCR3 = 65535;
		return;
	}

    commutation_step = (commutation_position >> N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT) % N_COMMUTATION_STEPS;
    commutation_sub_step = (commutation_position & 0xff) >> (8 - N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT);

    phase1 = commutation_lookup_table[commutation_step].phase1;
    phase2 = commutation_lookup_table[commutation_step].phase2;
    phase3 = commutation_lookup_table[commutation_step].phase3;
    phase1_slope = commutation_lookup_table[commutation_step].phase1_slope;
    phase2_slope = commutation_lookup_table[commutation_step].phase2_slope;
    phase3_slope = commutation_lookup_table[commutation_step].phase3_slope;

    tmp32bit = phase1_slope;
    tmp32bit *= commutation_sub_step;
    tmp32bit >>= N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT;
    phase1 = phase1 + tmp32bit;

    tmp32bit = phase2_slope;
    tmp32bit *= commutation_sub_step;
    tmp32bit >>= N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT;
    phase2 = phase2 + tmp32bit;

    tmp32bit = phase3_slope;
    tmp32bit *= commutation_sub_step;
    tmp32bit >>= N_COMMUTATION_SUB_STEPS_SHIFT_RIGHT;
    phase3 = phase3 + tmp32bit;

	// Here we compensate for the supply voltage. The higher is the supply voltage, the lower we will set the PWM to to get the same resulting motor voltage.
	int16_t supply_voltage = get_supply_voltage_ADC_value();
	motor_pwm_voltage = (motor_pwm_voltage << 12) / supply_voltage;

    phase1 >>= 8;
    phase1 *= (uint32_t)motor_pwm_voltage;
    phase1 >>= 16;
    phase1 += 13;
	if(!global_settings.motor_phases_reversed) {
	    TIM1->CCR1 = phase1;
	}
	else {
	    TIM1->CCR3 = phase1;
	}

    phase2 >>= 8;
    phase2 *= (uint32_t)motor_pwm_voltage;
    phase2 >>= 16;
    phase2 += 13;
	TIM1->CCR2 = phase2;

    phase3 >>= 8;
    phase3 *= (uint32_t)motor_pwm_voltage;
    phase3 >>= 16;
    phase3 += 13;
	if(!global_settings.motor_phases_reversed) {
	    TIM1->CCR3 = phase3;
	}
	else {
	    TIM1->CCR1 = phase3;
	}
}
#else
void motor_phase_calculations(void)
{
	volatile uint32_t delay;
	uint32_t desired_step_position;

	if(is_mosfets_enabled() == 0) {
		TIM1->CCR1 = 0;
		return;
	}

	TIM1->CCR1 = motor_pwm_voltage;

	desired_step_position = (commutation_position / N_COMMUTATION_SUB_STEPS) & (N_COMMUTATION_STEPS - 1);
	int32_t steps_to_target = desired_step_position - actual_step_position;

	if(steps_to_target > (N_COMMUTATION_STEPS >> 1)) {
		steps_to_target -= N_COMMUTATION_STEPS;
	}
	else if(steps_to_target < -(N_COMMUTATION_STEPS >> 1)) {
		steps_to_target += N_COMMUTATION_STEPS;
	}

	#define ONE_MICROSECOND_DELAY 10
	if(steps_to_target < 0) {
		if(!global_settings.motor_phases_reversed) {  // control the direction line to the stepper motor driver
		    GPIOA->BSRR = (1 << 0) << 16;
		}
		else {
		    GPIOA->BSRR = (1 << 0);
		}
		for(delay = 0; delay < ONE_MICROSECOND_DELAY; delay++);
		actual_step_position = (actual_step_position - 1) & (N_COMMUTATION_STEPS - 1);
		GPIOA->BSRR = (1 << 1); // STEP line high
		for(delay = 0; delay < ONE_MICROSECOND_DELAY; delay++);
		GPIOA->BSRR = (1 << 1) << 16; // STEP line low
	}
	else if(steps_to_target > 0) {
		if(!global_settings.motor_phases_reversed) {  // control the direction line to the stepper motor driver
		    GPIOA->BSRR = (1 << 0);
		}
		else {
		    GPIOA->BSRR = (1 << 0) << 16;
		}
		for(delay = 0; delay < ONE_MICROSECOND_DELAY; delay++);
		actual_step_position = (actual_step_position + 1) & (N_COMMUTATION_STEPS - 1);
		GPIOA->BSRR = (1 << 1); // STEP line high
		for(delay = 0; delay < ONE_MICROSECOND_DELAY; delay++);
		GPIOA->BSRR = (1 << 1) << 16; // STEP line low
	}
}
#endif


#define MAX_HALL_POSITION_DELTA_FATAL_ERROR_THRESHOLD 20000
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
	profiler_start_time(ALL_MOTOR_CONTROL_CALULATIONS_PROFILER);

	static uint8_t hall_position_delta_violation = 0;
	get_sensor_position_return_t get_sensor_position_return;

#ifdef DO_DETAILED_PROFILING
	profiler_start_time(GET_SENSOR_POSITION_PROFILER);
#endif
	get_sensor_position_return = get_sensor_position();
#ifdef DO_DETAILED_PROFILING
	profiler_end_time(GET_SENSOR_POSITION_PROFILER);
#endif
	sensor_position = get_sensor_position_return.position;
	hall_position_delta = get_sensor_position_return.change;
	hall_position += hall_position_delta;

	// DEBUG following 8 lines
	#define MAX_ASSUMED_MOTOR_RPM 1000
//	#define MAX_POSITION_COUNTS_PER_CONTROL_CYCLE ((uint32_t)((int64_t)357*(int64_t)256*(int64_t)50*(int64_t)MAX_ASSUMED_MOTOR_RPM/(60*31250)))
	#define MAX_POSITION_COUNTS_PER_CONTROL_CYCLE (2500)
//	if(hall_position > previous_hall_position + MAX_POSITION_COUNTS_PER_CONTROL_CYCLE) {  // we assume here that the motor will never rotate faster than 1000 rpm
//		hall_position = previous_hall_position + MAX_POSITION_COUNTS_PER_CONTROL_CYCLE;    
//	}
//	else if(hall_position < previous_hall_position - MAX_POSITION_COUNTS_PER_CONTROL_CYCLE) {
//		hall_position = previous_hall_position - MAX_POSITION_COUNTS_PER_CONTROL_CYCLE;
//	}

	if(hall_position_delta > max_hall_position_delta) {
		max_hall_position_delta = hall_position_delta;
	}
	if(hall_position_delta < min_hall_position_delta) {
		min_hall_position_delta = hall_position_delta;
	}
	average_hall_position_delta += hall_position_delta;
	average_hall_position_delta_count++;

	if(fast_capture_data_active != 0) {
		fast_capture_data[fast_capture_data_index].hall1 = get_hall_sensor1_voltage();
		fast_capture_data[fast_capture_data_index].hall2 = get_hall_sensor2_voltage();
		fast_capture_data[fast_capture_data_index].hall3 = get_hall_sensor3_voltage();
		fast_capture_data[fast_capture_data_index].hall_position_16bit = (uint16_t)(hall_position >> 8);
		fast_capture_data_index++;
		if(fast_capture_data_index >= fast_capture_data_size) {
			if(fast_capture_data_active == 1) {
				fast_capture_data_active = 0;
				fast_capture_data_result_ready = 1;
			}
			else {
				fast_capture_data_index = 0;
			}
		}
	}

	// check that the hall position didn't change too much in one cycle. if it did then there is something very wrong.
	if((hall_position_delta < -MAX_HALL_POSITION_DELTA_FATAL_ERROR_THRESHOLD) || (hall_position_delta > MAX_HALL_POSITION_DELTA_FATAL_ERROR_THRESHOLD)) {
		if(hall_position_delta_violation == 0) {
			hall_position_delta_violation = 1;
		}
		else {
			disable_mosfets();
			fatal_error(29); // DEBUG
			if(fast_capture_data_active) {
				fast_capture_data_active = 0;
				fast_capture_data_result_ready = 1;
			}
		}
	}

#ifdef DO_DETAILED_PROFILING
	profiler_start_time(COMPUTE_VELOCITY_PROFILER);
#endif
	compute_velocity();
#ifdef DO_DETAILED_PROFILING
	profiler_end_time(COMPUTE_VELOCITY_PROFILER);
#endif

#ifdef DO_DETAILED_PROFILING
	profiler_start_time(MOTOR_MOVEMENT_CALCULATIONS_PROFILER);
#endif
	motor_movement_calculations();
#ifdef DO_DETAILED_PROFILING
	profiler_end_time(MOTOR_MOVEMENT_CALCULATIONS_PROFILER);
#endif

	// check that the position values don't go out of range (overflow)
	if((current_position.i32[1] > POSITION_OUT_OF_RANGE_FATAL_ERROR_THRESHOLD) || (current_position.i32[1] < -POSITION_OUT_OF_RANGE_FATAL_ERROR_THRESHOLD)) {
		fatal_error(20); // "position out of range" (all error text is defined in error_text.c)
	}

	// check that the hall sensor position doesn't go out of range (overflow)
	if((hall_position > POSITION_OUT_OF_RANGE_FATAL_ERROR_THRESHOLD) || (hall_position < -POSITION_OUT_OF_RANGE_FATAL_ERROR_THRESHOLD)) {
		fatal_error(21); // "hall position out of range" (all error text is defined in error_text.c)
	}

#ifdef DO_DETAILED_PROFILING
	profiler_start_time(MOTOR_PHASE_CALCULATIONS_PROFILER);
#endif
	motor_phase_calculations();
#ifdef DO_DETAILED_PROFILING
	profiler_end_time(MOTOR_PHASE_CALCULATIONS_PROFILER);
#endif

	uint16_t motor_control_period = profiler_period(MOTOR_CONTROL_LOOP_PERIOD_PROFILER);
	if(motor_control_period > MAX_MOTOR_CONTROL_PERIOD_FATAL_ERROR_THRESHOLD) {
		fatal_error(30); // "control loop took too long"  (all error text is defined in error_text.c)
	}

	profiler_end_time(ALL_MOTOR_CONTROL_CALULATIONS_PROFILER);

	TIM1->SR = 0; // clear the interrupt flag
}


void disable_motor_control_loop(void)
{
    TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt
}


void increase_commutation_offset(void)
{
	char buf[100];
    TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
	commutation_position_offset += ((N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS + 50) / 100); // the + 50 is for rounding
    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
	sprintf(buf, "commutation_position_offset: %lu\n", commutation_position_offset);
	print_debug_string(buf);
}


void decrease_commutation_offset(void)
{
	char buf[100];
    TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
	commutation_position_offset -= ((N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS + 50) / 100); // the + 50 is for rounding
    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
	sprintf(buf, "commutation_position_offset: %lu\n", commutation_position_offset);
	print_debug_string(buf);
}


#define MOTOR_PWM_VOLTAGE_MANUAL_STEP 10
void increase_motor_pwm_voltage(void)
{
	char buf[100];
    TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
	if((desired_motor_pwm_voltage > -50) && (desired_motor_pwm_voltage < 50)) {
		desired_motor_pwm_voltage++;
	}
	else {
		if(desired_motor_pwm_voltage < (PWM_PERIOD - MOTOR_PWM_VOLTAGE_MANUAL_STEP)) {
			desired_motor_pwm_voltage += MOTOR_PWM_VOLTAGE_MANUAL_STEP;
		}
		else {
			desired_motor_pwm_voltage = PWM_PERIOD;
		}
	}
    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
	sprintf(buf, "desired_motor_pwm_voltage: %d\n", (int)desired_motor_pwm_voltage);
	print_debug_string(buf);
}

void decrease_motor_pwm_voltage(void)
{
	char buf[100];
    TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
	if((desired_motor_pwm_voltage > -50) && (desired_motor_pwm_voltage < 50)) {
		desired_motor_pwm_voltage--;
	}
	else {
		if(desired_motor_pwm_voltage > -(PWM_PERIOD - MOTOR_PWM_VOLTAGE_MANUAL_STEP)) {
			desired_motor_pwm_voltage -= MOTOR_PWM_VOLTAGE_MANUAL_STEP;
		}
		else {
			desired_motor_pwm_voltage = -PWM_PERIOD;
		}
	}
    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
	sprintf(buf, "desired_motor_pwm_voltage: %d\n", (int)desired_motor_pwm_voltage);
	print_debug_string(buf);
}

void set_motor_control_mode(uint8_t new_motor_control_mode)
{
	motor_control_mode = new_motor_control_mode;
}

uint8_t get_motor_control_mode(void)
{
	return motor_control_mode;
}

uint32_t get_update_frequency(void)
{
	return PWM_FREQUENCY >> 1;
}

void zero_position(void)
{
	if(n_items_in_queue != 0) {
		fatal_error(8); // "queue not empty" (all error text is defined in error_text.c)
	}

    TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
	current_position.i64 = 0;
	current_velocity_i64 = 0;
	hall_position = 0;
	hall_position_delta = 0;
	bound_the_sensor_position(N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS);
//	zero_hall_position(1);
	velocity = 0;
	integral_term = 0;
	previous_error = 0;
	low_pass_filtered_error_change = 0;
    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
}


void set_max_velocity(uint32_t new_max_velocity)
{
    TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
	max_velocity = new_max_velocity;
	max_velocity <<= VELOCITY_SHIFT_LEFT;
	if(max_velocity > MAX_VELOCITY) {
		max_velocity = MAX_VELOCITY;
	}
    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
}

int32_t get_max_velocity(void)
{
	return max_velocity;
}

void set_max_acceleration(uint32_t new_max_acceleration)
{
    TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
	max_acceleration = new_max_acceleration;
	max_acceleration <<= ACCELERATION_SHIFT_LEFT;
	if(max_acceleration > MAX_ACCELERATION) {
		max_acceleration = MAX_ACCELERATION;
	}
    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
}

int32_t get_max_acceleration(void)
{
	return max_acceleration;
}

int32_t get_current_position(void)
{
	return current_position.i32[1];
}

void reset_time(void)
{
    TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
    clear_the_queue_and_stop_no_disable_interrupt();
	reset_profilers();
    reset_microsecond_time();
    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
}

void emergency_stop(void)
{
    TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
    disable_mosfets();
    clear_the_queue_and_stop_no_disable_interrupt();
    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
}


int32_t get_motor_position(void)
{
	int32_t ret;

    TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
	if(motor_control_mode == CLOSED_LOOP_POSITION_CONTROL) {
		ret = hall_position;
	}
	else {
		ret = current_position.i32[1];
	}
    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
    return ret;
}


int32_t get_hall_position(void)
{
    return hall_position;
}


uint8_t get_motor_status_flags(void)
{
	uint8_t motor_status_flags = 0;
	
    TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
	if(is_mosfets_enabled()) {
		motor_status_flags |= (1 << STATUS_MOSFETS_ENABLED_FLAG_BIT);
	}
	if(motor_control_mode == CLOSED_LOOP_POSITION_CONTROL) {
		motor_status_flags |= (1 << STATUS_CLOSED_LOOP_FLAG_BIT);
	}
	if(calibration_step != 0) {
		motor_status_flags |= (1 << STATUS_CALIBRATING_FLAG_BIT);
	}
	if(homing_active) {
		motor_status_flags |= (1 << STATUS_HOMING_FLAG_BIT);
	}
	if(go_to_closed_loop_step != 0) {
		motor_status_flags |= (1 << STATUS_GO_TO_CLOSED_LOOP_FLAG_BIT);
	}
	if(motor_busy) {
		motor_status_flags |= (1 << STATUS_MOTOR_BUSY_FLAG_BIT);
	}
    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt

	return motor_status_flags;
}


void get_max_PID_error(int32_t *_min_PID_error, int32_t *_max_PID_error)
{
    TIM1->DIER &= ~TIM_DIER_UIE; // disable the update interrupt during this operation
	int32_t min = min_PID_error;
	int32_t max = max_PID_error;
	min_PID_error = 2147483647;
	max_PID_error = -2147483648;
    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt
	*_min_PID_error = min;
	*_max_PID_error = max;
}


void set_movement_limits(int32_t lower_limit, int32_t upper_limit)
{
	position_lower_safety_limit = lower_limit;
	position_upper_safety_limit = upper_limit;
}


void calculate_and_set_analog_watchdog_limits(void)
{
	int32_t analog_watchdog_lower_limit = (int32_t)max_motor_regen_pwm_voltage * (int32_t)ANALOG_WATCHDOG_LIMIT_MULTIPLIER / (int32_t)128;
	int32_t analog_watchdog_upper_limit = (int32_t)max_motor_pwm_voltage * (int32_t)ANALOG_WATCHDOG_LIMIT_MULTIPLIER / (int32_t)128;
	set_analog_watchdog_limits(motor_current_baseline - analog_watchdog_lower_limit, motor_current_baseline + analog_watchdog_upper_limit);
}


void set_max_motor_current(uint16_t new_max_motor_pwm_voltage, uint16_t new_max_motor_regen_pwm_voltage)
{
	if(new_max_motor_pwm_voltage > MAX_PWM_VOLTAGE_AT_ZERO_VELOCITY) {
		fatal_error(23);
	}
	if(new_max_motor_regen_pwm_voltage > MAX_PWM_VOLTAGE_AT_ZERO_VELOCITY) {
		fatal_error(23);
	}
	max_motor_pwm_voltage = new_max_motor_pwm_voltage;
	max_motor_regen_pwm_voltage = new_max_motor_regen_pwm_voltage;
//#ifndef PRODUCT_NAME_M3
	calculate_and_set_analog_watchdog_limits();
//#endif
}


void set_commutation_position_offset(uint32_t new_commutation_position_offset)
{
	commutation_position_offset = new_commutation_position_offset;
}


#if !defined(PRODUCT_NAME_M3) && !defined(PRODUCT_NAME_M4)
void check_current_sensor_and_enable_mosfets(void)
{
//	volatile uint32_t delay;
	char buf[100];

	if(is_mosfets_enabled()) {
		print_debug_string("MOSFETs already enabled\n");
		return;
	}
	TIM1->CCR1 = 65535; // set all PWMs to a known state where the motor voltage between windings will be 0
	TIM1->CCR2 = 65535;
	TIM1->CCR3 = 65535;

//	print_debug_string("Delay 1\n");
//	for(delay = 0; delay < 1000000; delay++);

	motor_current_baseline = get_motor_current();
	sprintf(buf, "motor_current_baseline: %hu\n", motor_current_baseline);

//	print_debug_string("Delay 2\n");
//	for(delay = 0; delay < 1000000; delay++);

	if((motor_current_baseline < MIN_MOTOR_CURRENT_BASELINE) || (motor_current_baseline > MAX_MOTOR_CURRENT_BASELINE)) {
		fatal_error(22); // "current sensor failed". All error messages are defined in error_text.c
	}
	calculate_and_set_analog_watchdog_limits();

//	print_debug_string("Delay 3\n");
//	for(delay = 0; delay < 1000000; delay++);

	print_debug_string("Enabling MOSFETs\n");

//	print_debug_string("Delay 4\n");
//	for(delay = 0; delay < 100000; delay++);

	enable_mosfets();

//	print_debug_string("Delay 5\n");
//	for(delay = 0; delay < 1000000; delay++);

}
#else
void check_current_sensor_and_enable_mosfets(void)
{
	if(is_mosfets_enabled()) {
		print_debug_string("MOSFETs already enabled\n");
		return;
	}
	TIM1->CCR1 = 0;
	print_debug_string("Enabling MOSFETs\n");
	enable_mosfets();
}
#endif


void get_multipurpose_data(uint8_t *data_type, uint16_t *data_size, uint8_t **multipurpose_data_ptr)
{
	*data_type = multipurpose_data_type;
	*data_size = multipurpose_data_size;
	*multipurpose_data_ptr = (uint8_t*)calibration;
}


void set_motor_test_mode(uint8_t new_test_mode)
{
	test_mode = new_test_mode;
}

// We will set up the motor driver as follows:
// set the enable line low to enable the motor (active low)
// set the direction line high to let the motor spin in the forward direction (not sure of that is clockwise or counter-clockwise)
// put a pulse onto the step line to make the motor take steps. the pulse will be at 1 kHz
void test_M3_motor_spin(void)
{
    volatile uint32_t delay = 0; 
//    volatile uint32_t delay2 = 0;
    // set the enable line low to enable the motor (active low)
    GPIOB->BSRR = (1 << 0) << 16;
    // set the direction line high to let the motor spin in the forward direction (not sure of that is clockwise or counter-clockwise)
    GPIOA->BSRR = (1 << 0);
    // put a pulse onto the step line to make the motor take steps. the pulse will be at 1 kHz
    while(1) {
        GPIOA->BSRR = (1 << 1); // STEP line high
        red_LED_on();
        for(delay = 0; delay < 500; delay++){
//            for(delay2 = 0; delay2 < 5; delay2++) {
//                GPIOA->BSRR = (1 << 8) << 16;  // IMCONTROL line low
//            }
            GPIOA->BSRR = (1 << 8); // IMCONTROL line high
        }
        GPIOA->BSRR = (1 << 1) << 16; // STEP line low
        red_LED_off();
        for(delay = 0; delay < 500; delay++){
//            for(delay2 = 0; delay2 < 5; delay2++) {
//                GPIOA->BSRR = (1 << 8) << 16; // IMCONTROL line low
//            }
            GPIOA->BSRR = (1 << 8); // IMCONTROL line high
        }
    }
}
