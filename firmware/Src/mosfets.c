#include "stm32g0xx_hal.h"

void enable_mosfets(void)
{
    #if defined(PRODUCT_NAME_M1) || defined(PRODUCT_NAME_M2)
    GPIOA->BSRR = (1 << 1);
    #endif
    #ifdef PRODUCT_NAME_M17
    GPIOB->BSRR = (1 << 0) << 16;  // Clear PB0 to enable MOSFETs
    #endif
    #ifdef PRODUCT_NAME_M23
    GPIOA->BSRR = (1 << 1);
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
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    GPIOA->BSRR = ((1 << 1) << 16);
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
    return (GPIOA->IDR & (1 << 1)) != 0;
    #endif
    return 0;
}
