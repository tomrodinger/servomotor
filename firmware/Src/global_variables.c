#include <stdint.h>
#include "settings.h"

#define DEFAULT_HALL_MIDLINE 38500
#define DEFAULT_ALIAS 255
#define UINT32_MIDPOINT 2147483648

uint8_t my_alias = DEFAULT_ALIAS;

struct global_settings_struct global_settings = {DEFAULT_ALIAS, DEFAULT_HALL_MIDLINE, DEFAULT_HALL_MIDLINE, DEFAULT_HALL_MIDLINE, UINT32_MIDPOINT, 0};
