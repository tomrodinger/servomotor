#ifndef SRC_MOTOR_CONTROL_H_
#define SRC_MOTOR_CONTROL_H_

#define OPEN_LOOP_POSITION_CONTROL 0
#define CLOSED_LOOP_POSITION_CONTROL 1
#define OPEN_LOOP_PWM_VOLTAGE_CONTROL 2

#define CAPTURE_HALL_SENSOR_READINGS 1
#define CAPTURE_HALL_POSITION 2
#define CAPTURE_ADJUSTED_HALL_SENSOR_READINGS 3
#define CAPTURE_HALL_SENSOR_READINGS_WHILE_TURNING 4

#define TIME_STEPS_PER_SECOND         (PWM_FREQUENCY >> 1)
#define MICROSTEPS_PER_ROTATION       ((uint64_t)(N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS * ONE_REVOLUTION_STEPS) * ((uint64_t)1 << 32))
//#define MICROSTEPS_PER_ROTATION       ((uint64_t)(N_COMMUTATION_STEPS * N_COMMUTATION_SUB_STEPS * ONE_REVOLUTION_STEPS))
#define MAX_RPM                       1980 // should be divisable by 60 ideally
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

#define DEFAULT_MAX_MOTOR_CURRENT 100
#define DEFAULT_MAX_MOTOR_REGEN_CURRENT 100

typedef enum {MOVE_WITH_ACCELERATION = 0, MOVE_WITH_VELOCITY} movement_type_t;

void processCommand(uint8_t axis, uint8_t command, uint8_t *parameters);
void start_calibration(uint8_t print_output);
void start_go_to_closed_loop_mode(void);
void start_homing(int32_t max_homing_displacement, uint32_t max_homing_time);
void move_n_steps_in_m_time(int32_t displacement, uint32_t time_delta);
void print_queue_stats(void);
void start_capture(uint8_t capture_type);
void reset_time(void);

void print_position(void);
void print_current_movement(void);
void print_velocity(void);
void print_time_difference(void);
void print_max_motor_current_settings(void);
void print_motor_current(void);
void print_hall_sensor_data(void);
void print_hall_position_delta_stats(void);
void print_motor_status(void);

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
int32_t get_actual_motor_position(void);
uint8_t get_motor_status_flags(void);
uint8_t is_calibration_data_available(void);
void process_calibration_data(void);
void set_motor_current_baseline(void);
void set_max_motor_current(uint16_t new_max_motor_current, uint16_t new_max_motor_regen_current);
void set_movement_limits(int32_t lower_limit, int32_t upper_limit);


#endif /* SRC_MOTOR_CONTROL_H_ */
