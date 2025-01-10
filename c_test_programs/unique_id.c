#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include "settings.h"
#include "product_info.h"

uint64_t get_unique_id(void)
{
    // Just return a fixed ID for simulation
    return 0x0123456789ABCDEF;
}

uint32_t get_random_number(uint32_t limit)
{
    static int initialized = 0;
    if (!initialized) {
        srand(time(NULL));
        initialized = 1;
    }
    return (uint32_t)(rand() % (limit + 1));
}
