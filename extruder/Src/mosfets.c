#include "stm32g0xx_hal.h"
#include "error_handling.h"

static uint8_t mosfets_enabled = 0;
static uint8_t active_mosfets_index = 0;

void disable_mosfets(void)
{
    GPIOA->BSRR = (1 << 1) << 16;
    GPIOB->BSRR = (1 << 8) << 16;
    TIM1->CCR1 = 65535;
    TIM1->CCR2 = 65535;
    TIM1->CCR3 = 65535;
    TIM3->CCR1 = 65535;
    TIM3->CCR2 = 65535;
    TIM3->CCR3 = 65535;
    mosfets_enabled = 0;
}

void enable_mosfets(void)
{
    if(!mosfets_enabled) {
        if(active_mosfets_index == 0) {
            GPIOB->BSRR = (1 << 8) << 16; // disable the second motor
            TIM3->CCR1 = 65535;
            TIM3->CCR2 = 65535;
            TIM3->CCR3 = 65535;
            GPIOA->BSRR = 1 << 1;         // enable the first motor
            mosfets_enabled = 1;
        }
        else if(active_mosfets_index == 1) {
            GPIOA->BSRR = (1 << 1) << 16; // disable the first motor
            TIM1->CCR1 = 65535;
            TIM1->CCR2 = 65535;
            TIM1->CCR3 = 65535;
            GPIOB->BSRR = 1 << 8;         // enable the second motor
            mosfets_enabled = 1;
        }
    }
}

void set_active_mosfets_index(int8_t index)
{
    if(index != active_mosfets_index) {
        if(mosfets_enabled) {
            disable_mosfets();
        }
        active_mosfets_index = index;
    }
}

uint8_t get_mosfets_enabled(void)
{
	return mosfets_enabled;
}

uint8_t get_active_mosfets_index(void)
{
    return active_mosfets_index;
}