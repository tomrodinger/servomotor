#include "stm32g0xx_hal.h"

static uint8_t mosfets_enabled = 0;

void enable_mosfets(void)
{
    GPIOA->BSRR = 1 << 1;
    // mosfets_enabled = 1;
    GPIOB->BSRR = 1 << 8;
    mosfets_enabled = 1;
}

void disable_mosfets(void)
{
    GPIOA->BSRR = (1 << 1) << 16;
    // mosfets_enabled = 0;
    GPIOB->BSRR = (1 << 8) << 16;
    mosfets_enabled = 0;
}

uint8_t get_mosfets_enabled(void)
{
	return mosfets_enabled;
}
