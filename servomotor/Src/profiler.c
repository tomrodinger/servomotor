#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "debug_uart.h"
#include "profiler.h"

static volatile uint16_t profiler_clock_start_time[N_PROFILERS];
static volatile uint16_t profiler_clock_end_time[N_PROFILERS];
static volatile uint16_t profiler_clock_max_time_difference[N_PROFILERS];
static volatile uint32_t debug_counter = 0;


void profiler_start_time(uint8_t profiler_clock_number)
{
	profiler_clock_start_time[profiler_clock_number] = TIM14->CNT;
}

void profiler_end_time(uint8_t profiler_clock_number)
{
	uint16_t t = TIM14->CNT;
	uint16_t time_difference = t - profiler_clock_start_time[profiler_clock_number];
	profiler_clock_end_time[profiler_clock_number] = t;
	if(time_difference > profiler_clock_max_time_difference[profiler_clock_number]) {
		profiler_clock_max_time_difference[profiler_clock_number] = time_difference;
	}
}


uint16_t profiler_get_time_difference(uint8_t profiler_clock_number)
{
	uint16_t start_time;
	uint16_t end_time;
	// Disable interrupts briefly while we read the start and end times as an atomic operation
	__disable_irq();
	start_time = profiler_clock_start_time[profiler_clock_number];
	end_time = profiler_clock_end_time[profiler_clock_number];
	__enable_irq();
	return end_time - start_time;
}


uint16_t profiler_get_max_time_difference(uint8_t profiler_clock_number)
{
	uint16_t t = profiler_clock_max_time_difference[profiler_clock_number];
	profiler_clock_max_time_difference[profiler_clock_number] = 0;
	return t;
}


uint16_t profiler_period(uint8_t profiler_clock_number)
{
	uint16_t t = TIM14->CNT;
	uint16_t elapsed_time;
	profiler_clock_start_time[profiler_clock_number] = profiler_clock_end_time[profiler_clock_number];
	profiler_clock_end_time[profiler_clock_number] = t;
	elapsed_time = t - profiler_clock_start_time[profiler_clock_number];
	if(elapsed_time > profiler_clock_max_time_difference[profiler_clock_number]) {
		profiler_clock_max_time_difference[profiler_clock_number] = elapsed_time;
	}
//	red_LED_off();
	if(elapsed_time >= 40) {
//		red_LED_on();
		debug_counter++;
	}
	return elapsed_time;
}


void print_debug_counter(void)
{
	char buf[50];
	uint32_t d = debug_counter;
	debug_counter = 0;
	snprintf(buf, sizeof(buf), "debug_counter: %lu\n", d);
	print_debug_string(buf);
}

void reset_profilers(void)
{
    uint16_t time_adjustment = TIM14->CNT;
    uint8_t i;

	for(i = 0; i < N_PROFILERS; i++) {
		profiler_clock_start_time[i] -= time_adjustment;
	}
}
