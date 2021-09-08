#include "stm32g0xx_hal.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "mosfets.h"
#include "leds.h"
#include "debug_uart.h"

void fatal_error(char *message, uint16_t error_code)
{
	volatile uint32_t delay;
	uint32_t e;
	char buf[10];

    __disable_irq();

    disable_mosfets();
    green_LED_off();

    sprintf(buf, ": %hu\n", error_code);
    while(1) { // print out the error message continuously forever
    	for(e = 0; e < error_code + 3; e++) {
    		if((error_code == 0) || (e < error_code)) {
    			red_LED_on(); // flash the red LED to indicate error, or keep it on continuously if the error code is 0
    		}
			for(delay = 0; delay < 1000000; delay++); // add a delay so we don't repeat printing the error message like a mad man
			if(error_code != 0) { // if the error code is 0 then we will just leave the red LED on continuously
				red_LED_off();
			}
			transmit_without_interrupts(message, strlen(message));
			transmit_without_interrupts(buf, strlen(buf));
			for(delay = 0; delay < 1000000; delay++); // add a delay so we don't repeat printing the error message like a mad man
    	}
    }
}
