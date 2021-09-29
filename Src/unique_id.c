#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <string.h>
#include "settings.h"

uint64_t get_unique_id(void)
{
	uint64_t unique_id;
	uint32_t unique_id_LSB;
	uint32_t unique_id_MSB;

	unique_id_LSB = calculate_crc32_buffer((uint8_t *)__DATE__, strlen(__DATE__));
	crc32_init();
	unique_id_MSB = calculate_crc32_buffer((uint8_t *)__TIME__, strlen(__TIME__));
	crc32_init();
	unique_id = unique_id_MSB;
	unique_id <<= 32;
	unique_id |= unique_id_LSB;

	return unique_id;
}

uint32_t get_random_number(uint32_t limit)
{
	return SysTick->VAL % (limit + 1); // totally not really a random number, but this should be good enough
}

