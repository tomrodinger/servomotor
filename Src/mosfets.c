#include "stm32g0xx_hal.h"

static uint8_t mosfets_enabled = 0;

void enable_mosfets(void)
{
    GPIOA->BSRR = 1 << 12;
}

void disable_mosfets(void)
{
    GPIOA->BSRR = (1 << 12) << 16;
}

uint8_t get_mosfets_enabled(void)
{
	return mosfets_enabled;
}
