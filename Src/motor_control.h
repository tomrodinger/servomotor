#ifndef SRC_MOTOR_CONTROL_H_
#define SRC_MOTOR_CONTROL_H_

#define OPEN_LOOP_POSITION_CONTROL 0
#define CLOSED_LOOP_POSITION_CONTROL 1
#define OPEN_LOOP_PWM_VOLTAGE_CONTROL 2

#define MAX_VELOCITY 1
#define MAX_ACCELERATION 1

typedef enum {MOVE_WITH_ACCELERATION = 0, MOVE_WITH_VELOCITY} movement_type_t;

void processCommand(uint8_t axis, uint8_t command, uint8_t *parameters);
void start_calibration(uint8_t verbose_data);
void start_go_to_closed_loop_mode(void);
void start_homing(int32_t max_homing_displacement);
void move_n_steps_in_m_time(int32_t displacement, uint32_t time_delta);
void print_queue_stats(void);
void start_capture(uint8_t capture_type);
void reset_time(void);

void print_position(void);
void print_current_movement(void);
void print_velocity(void);
void print_time_difference(void);
void print_motor_current(void);

void move_n_steps_in_m_time(int32_t displacement, uint32_t time_delta);
void add_to_queue(int32_t parameter, uint32_t n_time_steps, movement_type_t movement_type);
void add_trapezoid_move_to_queue(int32_t new_position, uint32_t max_velocity, int32_t acceleration);
uint8_t take_from_queue(int32_t *end_position, uint64_t *end_time);
uint8_t get_n_items_in_queue(void);
void clear_the_queue_and_stop(void);
void increase_motor_pwm_voltage(void);
void decrease_motor_pwm_voltage(void);
void set_motor_control_mode(uint8_t new_motor_closed_loop_control);
uint32_t get_update_frequency(void);
uint8_t get_motor_control_mode(void);
void zero_position_and_hall_sensor(void);
void set_max_velocity(int32_t new_max_velocity);
int32_t get_max_velocity(void);
int32_t get_desired_position(void);
void set_max_acceleration(uint16_t new_max_acceleration);
void emergency_stop(void);
int32_t get_actual_motor_position(void);
void get_motor_status(uint8_t *buf);


#endif /* SRC_MOTOR_CONTROL_H_ */
