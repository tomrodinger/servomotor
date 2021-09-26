#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <string.h>


#define CRC64_ECMA182_POLY 0x42F0E1EBA9EA3693ULL

static void generate_crc64_table(uint64_t *crc64_table)
{
	uint64_t i, j, c, crc;

	for (i = 0; i < 256; i++) {
		crc = 0;
		c = i << 56;

		for (j = 0; j < 8; j++) {
			if ((crc ^ c) & 0x8000000000000000ULL)
				crc = (crc << 1) ^ CRC64_ECMA182_POLY;
			else
				crc <<= 1;
			c <<= 1;
		}

		crc64_table[i] = crc;
	}
}


uint64_t crc64(uint64_t crc, const void *p, uint8_t len)
{
	uint8_t i;
	uint64_t t;
	uint64_t crc64_table[256] = {0};

	generate_crc64_table(crc64_table);

	const unsigned char *_p = p;

	for (i = 0; i < len; i++) {
		t = ((crc >> 56) ^ (*_p++)) & 0xff;
		crc = crc64_table[t] ^ (crc << 8);
	}

	return crc;
}

uint64_t get_unique_id(void)
{
	uint64_t unique_id = 1234567891011121314; // some kind of seed. can be anything

	unique_id = crc64(unique_id, __DATE__, strlen(__DATE__));
	unique_id = crc64(unique_id, __TIME__, strlen(__TIME__));

	return unique_id;
}

uint32_t get_random_number(uint32_t limit)
{
	return SysTick->VAL % (limit + 1); // totally not really a random number, but this should be good enough
}

