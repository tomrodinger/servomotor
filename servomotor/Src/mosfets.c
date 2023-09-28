#include "stm32g0xx_hal.h"

static uint8_t mosfets_enabled = 0;

void enable_mosfets(void)
{
    #ifndef PRODUCT_NAME_M3
    GPIOA->BSRR = (1 << 1);
    #else
    GPIOB->BSRR = (1 << 0) << 16;
    #endif
    mosfets_enabled = 1;
}

void disable_mosfets(void)
{
    #ifndef PRODUCT_NAME_M3
    GPIOA->BSRR = (1 << 1) << 16;
    #else
    GPIOB->BSRR = (1 << 0);
    #endif
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    mosfets_enabled = 0;
}

uint8_t is_mosfets_enabled(void)
{
	return mosfets_enabled;
}
