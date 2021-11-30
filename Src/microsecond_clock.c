#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <string.h>
#include "error_handling.h"

static volatile uint32_t upper_32_bits = 0;

void microsecond_clock_init(void)
{
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN; // enable the clock to TIM3
    TIM3->PSC = 64 - 1; // prescale it so that we get one count per microsecond. have to subtract 1 to be right
    TIM3->ARR = 0xFFFFFFFF;
    TIM3->DIER = TIM_DIER_UIE; // enable the interrupt for the update event (overflow)
    TIM3->CR1 = TIM_CR1_CEN | TIM_CR1_UIFREMAP; // enable the timer and remap the rollover bit to bit 31 of the counter
    TIM3->EGR |= TIM_EGR_UG; // do an update (load and clear the prescaler)
    NVIC_SetPriority(TIM3_IRQn, 1);
    NVIC_EnableIRQ(TIM3_IRQn); // enable the interrupt to this timer
}



void TIM3_IRQHandler(void)
{
    __disable_irq();
	uint32_t tim3_cnt = TIM3->CNT;
	if(tim3_cnt & (1 << 31)) {
		upper_32_bits++;
	}
    TIM3->SR = 0; // set all interrupt flags for the timer to zero
    __enable_irq();
}


uint64_t get_microsecond_time(void)
{
	static uint64_t previous_64bit_time = 0;
	volatile union {
		volatile uint64_t sixty_four_bit_time;
		volatile struct __attribute__((__packed__)) {
			volatile uint16_t lower_16_bits;
			volatile uint32_t upper_32_bits;
		} fourty_eight_bit_time;
	} returned_time;
	uint32_t tim3_cnt;

    __disable_irq();
	((uint32_t*)&returned_time.sixty_four_bit_time)[1] = 0;
	tim3_cnt = TIM3->CNT;
	returned_time.fourty_eight_bit_time.lower_16_bits = (uint16_t)tim3_cnt;
	if(tim3_cnt & (1 << 31)) {
		upper_32_bits++;
	    TIM3->SR = 0; // clear the bit that indicated an overflow which is re-mapped to bit 31 of the CNT register
	}
	returned_time.fourty_eight_bit_time.upper_32_bits = upper_32_bits;

	if(returned_time.sixty_four_bit_time < previous_64bit_time) {
		fatal_error(1); // "time went backwards" (all error text is defined in error_text.c)
	}
	previous_64bit_time = returned_time.sixty_four_bit_time;

	__enable_irq();

	return returned_time.sixty_four_bit_time;
}

/*
uint64_t get_microsecond_time(void)
{
	static uint64_t previous_64bit_time = 0;
	volatile union {
		volatile uint64_t sixty_four_bit_time;
		volatile struct __attribute__((__packed__)) {
			volatile uint16_t lower_16_bits;
			volatile uint32_t upper_32_bits;
		} fourty_eight_bit_time;
	} returned_time;

	returned_time.sixty_four_bit_time = 0;
	returned_time.fourty_eight_bit_time.upper_32_bits = upper_32_bits;
	returned_time.fourty_eight_bit_time.lower_16_bits = TIM3->CNT;

	while(returned_time.fourty_eight_bit_time.upper_32_bits != upper_32_bits) { // check if the timer rolled over in the middle of reading it
		returned_time.fourty_eight_bit_time.upper_32_bits = upper_32_bits; // if it rolled over, we need to read it again to make sure we have the new rolled over value
		returned_time.fourty_eight_bit_time.lower_16_bits = TIM3->CNT;
	}

	if(returned_time.sixty_four_bit_time < previous_64bit_time) {
		returned_time.sixty_four_bit_time = previous_64bit_time + 1;
	}
	previous_64bit_time = returned_time.sixty_four_bit_time;

	return returned_time.sixty_four_bit_time;
}
*/

void reset_microsecond_time(void)
{
	TIM3->CNT = 0;
	upper_32_bits = 0;
}
