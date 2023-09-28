#ifndef SRC_PWM_H_
#define SRC_PWM_H_

#define PWM_CLOCK_FREQUENCY 64000000
#define PWM_PERIOD 1024 // this sets the frequency to be about 62.5 kHz @ input clock frequency of 64MHz
#define PWM_PERIOD_MICROSECONDS (PWM_PERIOD * 1000000 / PWM_CLOCK_FREQUENCY)
#define PWM_FREQUENCY (PWM_CLOCK_FREQUENCY / PWM_PERIOD)
void pwm_init(void);


#endif /* SRC_PWM_H_ */
