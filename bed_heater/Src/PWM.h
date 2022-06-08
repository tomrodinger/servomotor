#ifndef SRC_PWM_H_
#define SRC_PWM_H_

#define PWM_PERIOD 1024 // this sets the frequency to be about 62500 Hz @ input clock frequency of 64MHz
#define PWM_FREQUENCY (64000000 / PWM_PERIOD)
void pwm_init(void);


#endif /* SRC_PWM_H_ */
