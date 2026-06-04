#include <stdio.h>
#include "stm32g0xx_hal.h"
#include "PWM.h"
#include "GPIO_interrupts.h"
#include "motor_control.h"


// Drive the overvoltage-threshold PWM (TIM1_CH4, on PA11 for M17) so that the comparator reference
// corresponds to the given supply voltage in volts. OVERVOLTAGE_PROTECTION_SETTING1/SETTING2 are the
// per-product calibration constants relating threshold voltage to the TIM1->CCR4 compare value.
void set_overvoltage_threshold(uint16_t voltage)
{
    TIM1->CCR4 = (OVERVOLTAGE_PROTECTION_SETTING1 * voltage) / OVERVOLTAGE_PROTECTION_SETTING2;
}


void pwm_init(void)
{
    RCC->APBENR2 |= RCC_APBENR2_TIM1EN; // enable the clock to TIM1
#ifdef PRODUCT_NAME_M23
//    RCC->CCIPR |= RCC_CCIPR_TIM1SEL; // select PLLQCLK (128 MHz) as TIM1 clock source
#endif
    set_overvoltage_threshold(OVERVOLTAGE_PROTECTION_SETTING); // set the default overvoltage threshold
#if defined(PRODUCT_NAME_M1) || defined(PRODUCT_NAME_M2)
    TIM1->CCR1 = 65535; // set all PWMs high at first (as a workaround to a problem with the MOSFET gate driver that causes high side and low side MOSFETS to turn on at the same time at the instant that the switch disable line goes high)
    TIM1->CCR2 = 65535;
    TIM1->CCR3 = 65535;
    GPIOA->AFR[1] |= (2 << GPIO_AFRH_AFSEL8_Pos) | (2 << GPIO_AFRH_AFSEL10_Pos) | (2 << GPIO_AFRH_AFSEL11_Pos); // choose the right alternate function to put on these pins to make the PWM
    GPIOB->AFR[0] |= (1 << GPIO_AFRL_AFSEL3_Pos);                                                               // see tables 13 to 17 in the chip's datasheet
#endif
#ifdef PRODUCT_NAME_M17
    TIM1->CCR1 = 0;
    GPIOA->AFR[1] |= (2 << GPIO_AFRH_AFSEL8_Pos) | (2 << GPIO_AFRH_AFSEL11_Pos); // choose the right alternate function to put on these pins to make the PWM
#endif
#ifdef PRODUCT_NAME_M23
    TIM1->CCR1 = PWM_PERIOD_TIM1 >> 1; // duty ratio should be 50% at first, which will put 0V on the phases on average
    TIM1->CCR2 = PWM_PERIOD_TIM1 >> 1;
    GPIOA->AFR[0] |= (2 << GPIO_AFRL_AFSEL7_Pos);  // PA7 will get function 2, which is TIM1_CH1N,   see tables 13 to 17 in the chip's datasheet
    GPIOA->AFR[1] |= (2 << GPIO_AFRH_AFSEL8_Pos);  // PA8 will get function 2, which is TIM1_CH1
    GPIOA->AFR[1] |= (2 << GPIO_AFRH_AFSEL11_Pos); // PA11 will get function 2, which is TIM1_CH4
    GPIOB->AFR[0] |= (2 << GPIO_AFRL_AFSEL0_Pos);  // PB0 will get function 2, which is TIM1_CH2N 
    GPIOB->AFR[0] |= (1 << GPIO_AFRL_AFSEL3_Pos);  // PB3 will get function 1, which is TIM1_CH2 
#endif

    TIM1->PSC = 0; // no prescaler
    TIM1->ARR = PWM_PERIOD_TIM1;
    TIM1->CNT = 0;


#if defined(PRODUCT_NAME_M1) || defined(PRODUCT_NAME_M2)
    TIM1->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos); // set channels 1 and 2 as PWM mode 1
    TIM1->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | (6 << TIM_CCMR2_OC4M_Pos); // set channels 3 and 4 as PWM mode 1
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E; // enable the outputs on all 4 channels
    TIM1->CCMR1 |= TIM_CCMR1_OC1CE | TIM_CCMR1_OC2CE; // enable the ability to force 0 onto these channels when a fault signal is detected (from the analog watchdog in this case)
    TIM1->CCMR2 |= TIM_CCMR2_OC3CE;                   // same as above but for the third channel
    TIM1->SMCR = TIM_SMCR_OCCS; // select the ETRF signal as the source that will cause the output of the PWM to be cleared
//    TIM1->BDTR = TIM_BDTR_MOE | TIM_BDTR_AOE | TIM_BDTR_BKP | TIM_BDTR_BKE | TIM_BDTR_OSSR | TIM_BDTR_OSSI; // main output enable, on break put lines in idle state only for the current PWM cycle, enable the break function
//    TIM1->BDTR = TIM_BDTR_MOE | TIM_BDTR_AOE | TIM_BDTR_BK2P | TIM_BDTR_BK2E | TIM_BDTR_OSSR | TIM_BDTR_OSSI; // main output enable, on break put lines in idle state only for the current PWM cycle, enable the break function
    TIM1->BDTR = TIM_BDTR_MOE | TIM_BDTR_AOE | TIM_BDTR_OSSR | TIM_BDTR_OSSI; // main output enable, on break put lines in idle state only for the current PWM cycle, enable the break function
    TIM1->AF1 = (4 << TIM1_AF1_ETRSEL_Pos) | TIM1_AF1_BKINE; // choose analog watchdog 2 as the break source, break on high signal, enable the break function
//    TIM1->AF2 = TIM1_AF2_BK2INE; // enable the break 2 function
#endif
#ifdef PRODUCT_NAME_M17
    TIM1->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos); // set channel 1
    TIM1->CCMR2 = (6 << TIM_CCMR2_OC4M_Pos); // and 4 as PWM mode 1
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC4E; // enable the outputs on channels 1 and 4
    TIM1->BDTR = TIM_BDTR_MOE; // main output enable
#endif
#ifdef PRODUCT_NAME_M23
    TIM1->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos); // set channels 1 and 2 as PWM mode 1
    TIM1->CCR3 = 512; // ADC trigger offset: OC3REF edges at CNT=512 → 4000 ns before both valley and peak
    TIM1->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | (6 << TIM_CCMR2_OC4M_Pos); // CH3 PWM mode 1 (ADC trigger), CH4 PWM mode 1 (overvoltage)
    TIM1->CCER = TIM_CCER_CC4E; // enable output on CH4 only (CH3 is internal trigger, no GPIO)
    TIM1->BDTR = TIM_BDTR_MOE; // main output enable
    TIM1->CR2 = (6 << TIM_CR2_MMS2_Pos); // MMS2=0110: OC3REF → TRGO2 (triggers ADC 4 us before valley and peak)
#endif

    TIM1->EGR |= TIM_EGR_UG; // Reinitialize the counter and generates an update of the registers. The prescaler internal counter is also cleared.
#ifdef PRODUCT_NAME_M23
    TIM1->CR1 = TIM_CR1_CEN | (1 << TIM_CR1_CMS_Pos); // center-aligned mode 1, 128MHz / (2 * 1024) = 62.5 kHz PWM
#else
    TIM1->CR1 = TIM_CR1_CEN; // enable the counter of the timer 1
#endif
}
