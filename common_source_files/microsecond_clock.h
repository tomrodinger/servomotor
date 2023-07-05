#ifndef INC_MICROSECOND_CLOCK_H_
#define INC_MICROSECOND_CLOCK_H_

#include <stdint.h>

#define MOTOR_CONTROL_LOOP_PERIOD_PROFILER 0
#define N_PROFILERS 1

void microsecond_clock_init(void);
uint64_t get_microsecond_time(void);
void reset_microsecond_time(void);
void microsecond_delay(uint32_t microseconds);
uint16_t profiler(uint8_t profiler_clock_number);

#endif /* INC_MICROSECOND_CLOCK_H_ */
