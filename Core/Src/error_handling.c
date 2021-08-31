#include "stm32g0xx_hal.h"
#include <string.h>
#include "mosfets.h"
#include "leds.h"
#include "debug_uart.h"

void fatal_error(char *message, uint16_t error_code)
{
//    __disable_irq();
    disable_mosfets();
    red_LED_on(); // turn on red LED to indicate error
    while(1) { // print out the error message continuously forever
        transmit(message, strlen(message));
        transmit(": ", 2);
        print_number("Code:", error_code);
        transmit("\n", 1);
    }
}
