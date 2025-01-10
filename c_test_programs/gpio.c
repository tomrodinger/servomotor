#include "gpio.h"
#include "../common_source_files/gpio.h"

// Undefine the macros so we can provide the actual function definitions
#undef GPIO_init
#undef get_button_state

// Provide the actual function definitions that the linker is looking for
void GPIO_init(void) {
    GPIO_init_M3();
}

bool get_button_state(void) {
    return get_button_state_M3() != 0;  // Convert uint8_t to bool
}
