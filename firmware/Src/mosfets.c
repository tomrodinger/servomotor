#include "stm32g0xx_hal.h"
#include "PWM.h"

void enable_mosfets(void)
{
    #if defined(PRODUCT_NAME_M1) || defined(PRODUCT_NAME_M2)
    GPIOA->BSRR = (1 << 1);
    #endif
    #ifdef PRODUCT_NAME_M17
    GPIOB->BSRR = (1 << 0) << 16;  // Clear PB0 to enable MOSFETs
    #endif
    #ifdef PRODUCT_NAME_M23
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC1NE | TIM_CCER_CC2NE; // enable the outputs on channels 1, 2 as well as their complementary outputs
    #endif
}

void disable_mosfets(void)
{
    #if defined(PRODUCT_NAME_M1) || defined(PRODUCT_NAME_M2)
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    GPIOA->BSRR = (1 << 1) << 16;
    #endif
    #ifdef PRODUCT_NAME_M17
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    GPIOB->BSRR = (1 << 0);  // Set PB0 to disable MOSFETs
    #endif
    #ifdef PRODUCT_NAME_M23
    TIM1->CCR1 = PWM_PERIOD_TIM1 >> 1;
    TIM1->CCR2 = PWM_PERIOD_TIM1 >> 1;
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC1NE | TIM_CCER_CC2NE); // disable the outputs on channels 1, 2 as well as their complementary outputs
    #endif
}

uint8_t is_mosfets_enabled(void)
{
    #if defined(PRODUCT_NAME_M1) || defined(PRODUCT_NAME_M2)
    return (GPIOA->IDR & (1 << 1)) != 0;
    #endif
    #ifdef PRODUCT_NAME_M17
    return (GPIOB->IDR & (1 << 0)) == 0;  // MOSFETs enabled when PB0 is low
    #endif
    #ifdef PRODUCT_NAME_M23
    return (TIM1->CCER & TIM_CCER_CC1E) != 0;
    #endif
}
