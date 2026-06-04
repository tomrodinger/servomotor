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
#if defined (PRODUCT_NAME_M17)
#define PWM_PERIOD_TIM1 512 // this sets the frequency to be about 250 kHz @ input clock frequency of 64MHz (which is for the PWM to set the motor current and the overvoltage threshold)
#define PWM_PERIOD_TIM16 2048 // this sets the frequency to be about 31.25 kHz @ input clock frequency of 64MHz (which is for the motor control interrupt)
#define OVERVOLTAGE_PROTECTION_SETTING1 287
#define OVERVOLTAGE_PROTECTION_SETTING2 24
#define OVERVOLTAGE_PROTECTION_SETTING 32
#endif

#if defined (PRODUCT_NAME_M23)
#define TIM1_CLOCK_FREQUENCY 128000000 // TIM1 clocked from PLLQCLK at 128 MHz
#define PWM_PERIOD_TIM1 1024 // center-aligned mode: 128MHz / (2 * 1024) = 62.5 kHz PWM frequency
#define PWM_PERIOD_TIM16 2048 // this sets the frequency to be about 31.25 kHz @ input clock frequency of 64MHz (which is for the motor control interrupt)
#define OVERVOLTAGE_PROTECTION_SETTING1 290
#define OVERVOLTAGE_PROTECTION_SETTING2 12
#define OVERVOLTAGE_PROTECTION_SETTING 38
#endif

#define PWM_PERIOD_MICROSECONDS (PWM_PERIOD_TIM16 * 1000000 / PWM_CLOCK_FREQUENCY)
#define PWM_FREQUENCY (PWM_CLOCK_FREQUENCY / PWM_PERIOD_TIM16)

// Fixed overvoltage-protection thresholds used by the production test modes (see TEST_MODE_COMMAND
// in main.c). With the production rack on a 24 V supply, the 22 V threshold must trip overvoltage
// protection and the 26 V threshold must not.
#define OVERVOLTAGE_TEST_LOW_VOLTAGE  22 // should trip on a 24 V rack
#define OVERVOLTAGE_TEST_HIGH_VOLTAGE 26 // should NOT trip on a 24 V rack

void pwm_init(void);
void set_overvoltage_threshold(uint16_t voltage); // drive the OV-threshold PWM (TIM1_CH4 / PA11) to the given voltage


#endif /* SRC_PWM_H_ */
