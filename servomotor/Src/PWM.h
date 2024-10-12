#ifndef SRC_PWM_H_
#define SRC_PWM_H_

#define PWM_CLOCK_FREQUENCY 64000000

#if defined (PRODUCT_NAME_M1) || defined (PRODUCT_NAME_M2)
#define PWM_PERIOD_TIM1 1024 // this sets the frequency to be about 62.5 kHz @ input clock frequency of 64MHz (which is for the PWM to set the motor current and the overvoltage threshold)
#define PWM_PERIOD_TIM16 2048 // this sets the frequency to be about 31.25 kHz @ input clock frequency of 64MHz (which is for the motor control interrupt)
#define OVERVOLTAGE_PROTECTION_SETTING1 290
#define OVERVOLTAGE_PROTECTION_SETTING2 12
#define OVERVOLTAGE_PROTECTION_SETTING 26

#endif
#if defined (PRODUCT_NAME_M3)
#define PWM_PERIOD_TIM1 512 // this sets the frequency to be about 250 kHz @ input clock frequency of 64MHz (which is for the PWM to set the motor current and the overvoltage threshold)
#define PWM_PERIOD_TIM16 2048 // this sets the frequency to be about 31.25 kHz @ input clock frequency of 64MHz (which is for the motor control interrupt)
#define OVERVOLTAGE_PROTECTION_SETTING1 287
#define OVERVOLTAGE_PROTECTION_SETTING2 24
#define OVERVOLTAGE_PROTECTION_SETTING 32
#endif

#if defined (PRODUCT_NAME_M4)
#define PWM_PERIOD_TIM1 1024 // this sets the frequency to be about 62.5 kHz @ input clock frequency of 64MHz (which is for the PWM to set the motor current and the overvoltage threshold)
#define PWM_PERIOD_TIM16 2048 // this sets the frequency to be about 31.25 kHz @ input clock frequency of 64MHz (which is for the motor control interrupt)
#define OVERVOLTAGE_PROTECTION_SETTING1 290
#define OVERVOLTAGE_PROTECTION_SETTING2 12
#define OVERVOLTAGE_PROTECTION_SETTING 38
#endif

#define PWM_PERIOD_MICROSECONDS (PWM_PERIOD_TIM16 * 1000000 / PWM_CLOCK_FREQUENCY)
#define PWM_FREQUENCY (PWM_CLOCK_FREQUENCY / PWM_PERIOD_TIM16)
void pwm_init(void);


#endif /* SRC_PWM_H_ */
