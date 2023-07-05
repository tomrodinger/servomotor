#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <string.h>
#include "microsecond_clock.h"
#include "error_handling.h"

static uint64_t previous_64bit_time = 0;
static volatile uint32_t upper_32_bits = 0;
static volatile uint16_t profiler_clock_start_time[N_PROFILERS];

void microsecond_clock_init(void)
{
    RCC->APBENR2 |= RCC_APBENR2_TIM14EN; // enable the clock to TIM14
    TIM14->PSC = 64 - 1; // prescale it so that we get one count per microsecond. have to subtract 1 to be right
    TIM14->ARR = 0xFFFFFFFF;
    TIM14->DIER = TIM_DIER_UIE; // enable the interrupt for the update event (overflow)
    TIM14->CR1 = TIM_CR1_CEN | TIM_CR1_UIFREMAP; // enable the timer and remap the rollover bit to bit 31 of the counter
    TIM14->EGR |= TIM_EGR_UG; // do an update (load and clear the prescaler)
    NVIC_SetPriority(TIM14_IRQn, 1);
    NVIC_EnableIRQ(TIM14_IRQn); // enable the interrupt to this timer
}



void TIM14_IRQHandler(void)
{
    __disable_irq();
	uint32_t tim_count = TIM14->CNT;
	if(tim_count & (1 << 31)) {
		upper_32_bits++;
	}
    TIM14->SR = 0; // set all interrupt flags for the timer to zero
    __enable_irq();
}


uint64_t get_microsecond_time(void)
{
	volatile union {
		volatile uint64_t sixty_four_bit_time;
		volatile struct __attribute__((__packed__)) {
			volatile uint16_t lower_16_bits;
			volatile uint32_t upper_32_bits;
		} fourty_eight_bit_time;
	} returned_time;
	uint32_t tim_count;


    __disable_irq();

	((uint32_t*)&returned_time.sixty_four_bit_time)[1] = 0;
	tim_count = TIM14->CNT;
	returned_time.fourty_eight_bit_time.lower_16_bits = (uint16_t)tim_count;
	if(tim_count & (1 << 31)) {
		upper_32_bits++;
	    TIM14->SR = 0; // clear the bit that indicated an overflow which is re-mapped to bit 31 of the CNT register
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
	returned_time.fourty_eight_bit_time.lower_16_bits = TIM14->CNT;

	while(returned_time.fourty_eight_bit_time.upper_32_bits != upper_32_bits) { // check if the timer rolled over in the middle of reading it
		returned_time.fourty_eight_bit_time.upper_32_bits = upper_32_bits; // if it rolled over, we need to read it again to make sure we have the new rolled over value
		returned_time.fourty_eight_bit_time.lower_16_bits = TIM14->CNT;
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
	uint16_t time_adjustment;
	uint8_t i;
    __disable_irq();
	time_adjustment = TIM14->CNT;
	TIM14->CNT = 0;
	upper_32_bits = 0;
	previous_64bit_time = 0;
	for(i = 0; i < N_PROFILERS; i++) {
		profiler_clock_start_time[i] -= time_adjustment;
	}
    __enable_irq();
}


void microsecond_delay(uint32_t microseconds)
{
	uint64_t start_time = get_microsecond_time();
	while(get_microsecond_time() - start_time < microseconds);
}


uint16_t profiler(uint8_t profiler_clock_number)
{
	uint16_t t = TIM14->CNT;
	uint16_t elapsed_time = t - profiler_clock_start_time[profiler_clock_number];
	profiler_clock_start_time[profiler_clock_number] = t;
	return elapsed_time;
}


//uint16_t get_maximum_profiler_clock_elapsed_time(uint8_t profiler_clock_number)
//{
//	return 0;
//}
