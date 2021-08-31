#ifndef SRC_MOTOR_CONTROL_H_
#define SRC_MOTOR_CONTROL_H_

// set which axis this motor is controlling
#ifdef X_AXIS
	#define MY_AXIS 'X'
#else
	#ifdef Y_AXIS
		#define MY_AXIS 'Y'
	#else
		#ifdef Z_AXIS
			#define MY_AXIS 'Z'
		#else
			#ifdef SMALL_Z_AXIS
				#define MY_AXIS 'z'
			#else
				#ifdef E_AXIS
					#define MY_AXIS 'E'
				#else
					#error "You need to set an axis"
				#endif
			#endif
		#endif
	#endif
#endif

#define ALL_AXIS 'A' // will also respond to this 'all' command

#define OPEN_LOOP_POSITION_CONTROL 0
#define CLOSED_LOOP_POSITION_CONTROL 1
#define OPEN_LOOP_PWM_VOLTAGE_CONTROL 2

void print_position(void);
void increase_motor_pwm_voltage(void);
void decrease_motor_pwm_voltage(void);
uint8_t get_motor_control_mode(void);
void set_motor_control_mode(uint8_t new_motor_closed_loop_control);
void processCommand(uint8_t axis, uint8_t command, uint8_t *parameters);
void add_to_queue(int32_t position, uint64_t time);
void start_calibration(uint8_t verbose_data);
void start_go_to_closed_loop_mode(void);
void start_homing(int32_t max_homing_displacement);
void move_n_steps_in_m_time(int32_t displacement, uint32_t time_delta);
void motor_control(void);
void print_queue_stats(void);
void zero_position_and_hall_sensor(void);
void print_current_movement(void);
void print_velocity(void);
void print_time_difference(void);
void print_motor_current(void);



#endif /* SRC_MOTOR_CONTROL_H_ */
