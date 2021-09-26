#ifndef INC_MICROSECOND_CLOCK_H_
#define INC_MICROSECOND_CLOCK_H_

#include <stdint.h>

void microsecond_clock_init(void);
uint64_t get_microsecond_time(void);
void reset_microsecond_time(void);


#endif /* INC_MICROSECOND_CLOCK_H_ */
