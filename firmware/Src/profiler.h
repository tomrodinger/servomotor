#ifndef INC_PROFILER_H_
#define INC_PROFILER_H_

#define ALL_MOTOR_CONTROL_CALULATIONS_PROFILER 0
#define GET_SENSOR_POSITION_PROFILER 1
#define COMPUTE_VELOCITY_PROFILER 2
#define MOTOR_MOVEMENT_CALCULATIONS_PROFILER 3
#define MOTOR_PHASE_CALCULATIONS_PROFILER 4
#define MOTOR_CONTROL_LOOP_PERIOD_PROFILER 5
#define N_PROFILERS 6

void profiler_start_time(uint8_t profiler_clock_number);
void profiler_end_time(uint8_t profiler_clock_number);
uint16_t profiler_get_time_difference(uint8_t profiler_clock_number);
uint16_t profiler_get_max_time_difference(uint8_t profiler_clock_number);
uint16_t profiler_period(uint8_t profiler_clock_number);
void print_debug_counter(void);
void reset_profilers(void);

#endif
