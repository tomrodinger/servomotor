#include "stm32g0xx_hal.h"
#include <stdint.h>

static volatile uint32_t upper_32_bits = 0;

void microsecond_clock_init(void)
{
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN; // enable the clock to TIM3
    TIM3->PSC = 64 - 1; // prescale it so that we get one count per microsecond. have to subtract 1 to be right
    TIM3->ARR = 0xFFFFFFFF;
    TIM3->DIER = TIM_DIER_UIE; // enable the interrupt for the update event (overflow)
    TIM3->CR1 = TIM_CR1_CEN; // enable the timer
    TIM3->EGR |= TIM_EGR_UG; // do an update (load and clear the prescaler)
    NVIC_EnableIRQ(TIM3_IRQn); // enable the interrupt to this timer
}

void TIM3_IRQHandler(void)
{
	upper_32_bits++;
//	sixty_four_bit_time += 65536;
    TIM3->SR = 0; // set all interrupt flags for the timer to zero
}

uint64_t get_microsecond_time(void)
{
	union {
		volatile uint64_t sixty_four_bit_time;
		struct __attribute__((__packed__)) {
			volatile uint16_t lower_16_bits;
			volatile uint32_t upper_32_bits;
		} fourty_eight_bit_time;
	} returned_time;

	returned_time.sixty_four_bit_time = 0;
	returned_time.fourty_eight_bit_time.upper_32_bits = upper_32_bits;
	returned_time.fourty_eight_bit_time.lower_16_bits = TIM3->CNT;
	if(returned_time.fourty_eight_bit_time.upper_32_bits != upper_32_bits) { // check if the timer rolled over in the middle of reading it
		returned_time.fourty_eight_bit_time.upper_32_bits = upper_32_bits; // if it rolled over, we need to read it again to make sure we have the new rolled over value
		returned_time.fourty_eight_bit_time.lower_16_bits = TIM3->CNT;
	}

	return returned_time.sixty_four_bit_time;
}

void reset_microsecond_time(void)
{
	TIM3->CNT = 0;
	upper_32_bits = 0;
}
