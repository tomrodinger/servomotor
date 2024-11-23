#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <string.h>
#include "settings.h"
#include "product_info.h"

uint64_t get_unique_id(void)
{
    struct product_info_struct *product_info = (struct product_info_struct *)(PRODUCT_INFO_MEMORY_LOCATION);
    return product_info->unique_id;
}

uint32_t get_random_number(uint32_t limit)
{
    return SysTick->VAL % (limit + 1); // totally not really a random number, but this should be good enough
}

