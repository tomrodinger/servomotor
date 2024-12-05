#include "stm32g0xx_hal.h"

static uint8_t mosfets_enabled = 0;

void enable_mosfets(void)
{
    mosfets_enabled = 1;
}

void disable_mosfets(void)
{
    mosfets_enabled = 0;
}

uint8_t get_mosfets_enabled(void)
{
	return mosfets_enabled;
}
