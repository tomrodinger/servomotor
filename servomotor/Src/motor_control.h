#ifndef SRC_MOTOR_CONTROL_H_
#define SRC_MOTOR_CONTROL_H_

#include <stdint.h>

#ifdef PRODUCT_NAME_M1
#include "hall_sensor_constants_M1.h"
#endif
#ifdef PRODUCT_NAME_M2
#include "hall_sensor_constants_M2.h"
#endif
#ifdef PRODUCT_NAME_M3
#include "hall_sensor_constants_M3.h"
#endif

#define OPEN_LOOP_POSITION_CONTROL 0
#define CLOSED_LOOP_POSITION_CONTROL 1
#define OPEN_LOOP_PWM_VOLTAGE_CONTROL 2

#define CAPTURE_HALL_SENSOR_READINGS 1
#define CAPTURE_HALL_POSITION 2
#define CAPTURE_ADJUSTED_HALL_SENSOR_READINGS 3
#define CAPTURE_HALL_SENSOR_READINGS_WHILE_TURNING 4

#define TIME_STEPS_PER_SECOND         (PWM_FREQUENCY >> 1)
#define MICROSTEPS_PER_ROTATION       ((uint64_t)ONE_REVOLUTION_MICROSTEPS * ((uint64_t)1 << 32))
//#define MICROSTEPS_PER_ROTATION       ((uint64_t)ONE_REVOLUTION_MICROSTEPS)
#ifdef PRODUCT_NAME_M3
#define MAX_RPM                       1020 // should be divisable by 60 ideally so that the MAX_RPS is an integer
#else
#define MAX_RPM                       2040 // should be divisable by 60 ideally so that the MAX_RPS is an integer
#endif
#define MAX_RPS                       (MAX_RPM / 60)
#define MAX_MICROSTEPS_PER_SECOND     ((uint64_t)MAX_RPS * (uint64_t)MICROSTEPS_PER_ROTATION)
#define MAX_MICROSTEPS_PER_TIME_STEP  ((uint64_t)MAX_MICROSTEPS_PER_SECOND / (uint64_t)TIME_STEPS_PER_SECOND)
#define MAX_VELOCITY                  ((int64_t)MAX_MICROSTEPS_PER_TIME_STEP)

#define MM_PER_ROTATION                                    20
#define MAX_ACCELERATION_MM_PER_SECOND_SQUARED             10000
#define MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED      (MAX_ACCELERATION_MM_PER_SECOND_SQUARED / MM_PER_ROTATION)
#define MAX_ACCELERATION_MICROSTEPS_PER_SECOND_SQUARED     ((uint64_t)MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED * (uint64_t)MICROSTEPS_PER_ROTATION)
#define MAX_ACCELERATION_MICROSTEPS_PER_TIME_STEP_SQUARED  ((uint64_t)MAX_ACCELERATION_MICROSTEPS_PER_SECOND_SQUARED / (uint64_t)(TIME_STEPS_PER_SECOND * TIME_STEPS_PER_SECOND))
#define MAX_ACCELERATION                                   ((int64_t)MAX_ACCELERATION_MICROSTEPS_PER_TIME_STEP_SQUARED)

#define DEFAULT_COMMUTATION_POSITION_OFFSET 2147483648
#ifdef PRODUCT_NAME_M3
#define DEFAULT_MAX_MOTOR_PWM_VOLTAGE 200
#define DEFAULT_MAX_MOTOR_REGEN_PWM_VOLTAGE 200
#else
#define DEFAULT_MAX_MOTOR_PWM_VOLTAGE 100
#define DEFAULT_MAX_MOTOR_REGEN_PWM_VOLTAGE 100
#endif

#define MULTIPURPOSE_DATA_TYPE_CALIBRATION 1
#define MULTIPURPOSE_DATA_TYPE_GO_TO_CLOSED_LOOP 2
#define MULTIPURPOSE_DATA_TYPE_VIBRATE 3

typedef enum {MOVE_WITH_ACCELERATION = 0, MOVE_WITH_VELOCITY} movement_type_t;

void reset_time_profiler(void);
void processCommand(uint8_t axis, uint8_t command, uint8_t *parameters);
void start_calibration(uint8_t print_output);
void start_go_to_closed_loop_mode(void);
void start_homing(int32_t max_homing_displacement, uint32_t max_homing_time);
void move_n_steps_in_m_time(int32_t displacement, uint32_t time_delta);
void print_queue_stats(void);
void start_capture(uint8_t capture_type);
void vibrate(uint8_t vibration_level);
void reset_time(void);

void print_position(void);
void print_PID_data(void);
void print_current_movement(void);
void print_velocity(void);
void print_time_difference(void);
void print_max_motor_current_settings(void);
void print_commutation_position_offset(void);
void print_motor_current(void);
void print_hall_sensor_data(void);
void print_hall_position_delta_stats(void);
void print_motor_pwm_voltage(void);
void print_motor_status(void);
void print_motor_temperature(void);

void start_fast_capture_data(void);
uint8_t is_fast_capture_data_result_ready(void);
void print_fast_capture_data_result(void);
void fast_capture_until_trigger(void);

void move_n_steps_in_m_time(int32_t displacement, uint32_t time_delta);
void add_to_queue(int32_t parameter, uint32_t n_time_steps, movement_type_t movement_type);
void add_trapezoid_move_to_queue(int32_t total_displacement, uint32_t total_time);
uint8_t take_from_queue(int32_t *end_position, uint64_t *end_time);
uint8_t get_n_items_in_queue(void);
void clear_the_queue_and_stop(void);
void increase_commutation_offset(void);
void decrease_commutation_offset(void);
void increase_motor_pwm_voltage(void);
void decrease_motor_pwm_voltage(void);
void set_motor_control_mode(uint8_t new_motor_closed_loop_control);
uint32_t get_update_frequency(void);
uint8_t get_motor_control_mode(void);
void zero_position_and_hall_sensor(void);
void set_max_velocity(uint32_t new_max_velocity);
int32_t get_max_velocity(void);
int32_t get_current_position(void);
void set_max_acceleration(uint32_t new_max_acceleration);
int32_t get_max_acceleration(void);
void emergency_stop(void);
int32_t get_motor_position(void);
int32_t get_hall_position(void);
uint8_t get_motor_status_flags(void);
void get_max_PID_error(int32_t *_min_PID_error, int32_t *_max_PID_error);
uint8_t is_calibration_data_available(void);
uint8_t process_calibration_data(void);
void set_motor_commutation_position_offset(uint32_t _commutation_position_offset);
void set_max_motor_current(uint16_t new_max_closed_loop_pwm_voltage, uint16_t new_max_closed_loop_regen_pwm_voltage);
void set_movement_limits(int32_t lower_limit, int32_t upper_limit);
void disable_motor_control_loop(void);
uint8_t is_go_to_closed_loop_data_available(void);
void process_go_to_closed_loop_data(void);

typedef struct __attribute__((__packed__)) {
	int32_t predicted_final_velocity;
	int32_t predicted_final_position;
	int32_t time_step_at_turn_point;
	int32_t relative_position_at_turn_point;
} add_to_queue_test_results_t;

void add_to_queue_test(int32_t parameter, uint32_t n_time_steps, movement_type_t movement_type, add_to_queue_test_results_t *results);
void get_multipurpose_data(uint8_t *data_type, uint16_t *data_size, uint8_t **multipurpose_data_ptr);
void set_commutation_position_offset(uint32_t new_commutation_position_offset);
void check_current_sensor_and_enable_mosfets(void);
void set_test_mode(uint8_t new_test_mode);
void test_M3_motor_spin(void);

#endif /* SRC_MOTOR_CONTROL_H_ */
