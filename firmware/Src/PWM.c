#include <stdio.h>
#include "stm32g0xx_hal.h"
#include "PWM.h"
#include "GPIO_interrupts.h"
#include "motor_control.h"


void pwm_init(void)
{
    RCC->APBENR2 |= RCC_APBENR2_TIM1EN; // enable the clock to TIM1
//    RCC->CCIPR |= RCC_CCIPR_TIM1SEL; // select the PLLQCLK as the clock source to TIM1
    TIM1->CCR4 = (OVERVOLTAGE_PROTECTION_SETTING1 * OVERVOLTAGE_PROTECTION_SETTING) / OVERVOLTAGE_PROTECTION_SETTING2;
#if defined(PRODUCT_NAME_M1) || defined(PRODUCT_NAME_M2)
    TIM1->CCR1 = 65535; // set all PWMs high at first (as a workaround to a problem with the MOSFET gate driver that causes high side and low side MOSFETS to turn on at the same time at the instant that the switch disable line goes high)
    TIM1->CCR2 = 65535;
    TIM1->CCR3 = 65535;
    GPIOA->AFR[1] |= (2 << GPIO_AFRH_AFSEL8_Pos) | (2 << GPIO_AFRH_AFSEL10_Pos) | (2 << GPIO_AFRH_AFSEL11_Pos); // choose the right alternate function to put on these pins to make the PWM
    GPIOB->AFR[0] |= (1 << GPIO_AFRL_AFSEL3_Pos);                                                               // see tables 13 to 17 in the chip's datasheet
#endif
#ifdef PRODUCT_NAME_M3
    TIM1->CCR1 = 0;
    GPIOA->AFR[1] |= (2 << GPIO_AFRH_AFSEL8_Pos) | (2 << GPIO_AFRH_AFSEL11_Pos); // choose the right alternate function to put on these pins to make the PWM
#endif
#ifdef PRODUCT_NAME_M4
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    GPIOA->AFR[1] |= (2 << GPIO_AFRH_AFSEL8_Pos) | (2 << GPIO_AFRH_AFSEL11_Pos); // choose the right alternate function to put on these pins to make the PWM
    GPIOB->AFR[0] |= (1 << GPIO_AFRL_AFSEL3_Pos);                                                               // see tables 13 to 17 in the chip's datasheet
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
#ifdef PRODUCT_NAME_M3
    TIM1->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos); // set channel 1
    TIM1->CCMR2 = (6 << TIM_CCMR2_OC4M_Pos); // and 4 as PWM mode 1
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC4E; // enable the outputs on channels 1 and 4
    TIM1->BDTR = TIM_BDTR_MOE; // main output enable
#endif
#ifdef PRODUCT_NAME_M4
    TIM1->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos); // set channels 1 and 2 as PWM mode 1
    TIM1->CCMR2 = (6 << TIM_CCMR2_OC4M_Pos); // and 4 as PWM mode 1
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC4E; // enable the outputs on channels 1, 2 and 4
    TIM1->BDTR = TIM_BDTR_MOE; // main output enable
#endif

    TIM1->EGR |= TIM_EGR_UG; // Reinitialize the counter and generates an update of the registers. The prescaler internal counter is also cleared. 
    TIM1->CR1 = TIM_CR1_CEN; // enable the counter of the timer 1

//    TIM1->SR = 0; // clear all the interrupt flags
//    TIM1->DIER |= TIM_DIER_UIE; // enable the update interrupt




#ifdef PRODUCT_NAME_M4
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN; // enable the clock to TIM3
    GPIOB->AFR[0] |= (1 << GPIO_AFRL_AFSEL4_Pos) | (1 << GPIO_AFRL_AFSEL5_Pos); // select the alternative function 1 for pins PB4 and PB5

    TIM3->PSC = 0; // no prescaler
    TIM3->ARR = PWM_PERIOD_TIM1;
    TIM3->CNT = (PWM_PERIOD_TIM1 >> 1);
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;

    TIM3->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos); // set channels 1 and 2 as PWM mode 1
//    TIM3->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos);                            // COMMENTED OUT set channels 3 as PWM mode 1
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;                          // enable the outputs of channels 1 and 2

    TIM3->CCMR1 |= TIM_CCMR1_OC1CE | TIM_CCMR1_OC2CE; // enable the ability to force 0 onto these channels when a fault signal is detected (from the analog watchdog in this case)
//    TIM3->CCMR2 |= TIM_CCMR2_OC3CE;                   // same as above but for the third channel
//    TIM3->SMCR = TIM_SMCR_OCCS;                       // set the ETRF signal as the OCREF clear source to be the internal trigger output
//    TIM3->BDTR = TIM_BDTR_MOE | TIM_BDTR_AOE | TIM_BDTR_OSSR | TIM_BDTR_OSSI; // main output enable, on break put lines in idle state only for the current PWM cycle, enable the break function
//    TIM3->BDTR = TIM_BDTR_MOE | TIM_BDTR_AOE; // main output enable, on break put lines in idle state only for the current PWM cycle
    TIM3->CR1 |= TIM_CR1_CEN;                         // enable the counter
//    TIM3->SR = 0; // clear all the interrupt flags
//    TIM3->DIER |= TIM_DIER_UIE; // enable the update interrupt
#endif


//    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 1); // second highest priority for the interrupt that controls the motor
//  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn); // enable the interrupt to this timer
}
