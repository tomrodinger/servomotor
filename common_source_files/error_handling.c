#include "stm32g0xx_hal.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "heater.h"
#include "mosfets.h"
#include "leds.h"
#include "debug_uart.h"
#include "RS485.h"
#include "commands.h"
#include "global_variables.h"
#include "error_text.h"
#include "device_status.h"

// Error text array initialization using macro from error_text.h
static const char error_text[] = ERROR_TEXT_INITIALIZER;

#define SYSTICK_LOAD_VALUE 16000000 // this must be less than 2^24 because the systick timer is 24 bits
#define FATAL_ERROR_COMMAND_BUFFER_SIZE 3

static volatile uint32_t systick_new_value = SYSTICK_LOAD_VALUE;
static volatile uint32_t systick_previous_value;
static uint8_t fatal_error_occurred = 0;
static uint8_t commandReceived = 0;
static uint16_t valueLength;
static uint16_t nReceivedBytes = 0;
static char volatile selectedAxis;
static uint8_t command;

static void fatal_error_systick_init(void)
{
    SysTick->LOAD  = SYSTICK_LOAD_VALUE;       // set this timer to roll over twice per second
    SysTick->VAL   = 0;       // Load the SysTick Counter Value
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;  // Enable SysTick Timer
}


static void receive(void)
{
    if(USART1->ISR & USART_ISR_RXNE_RXFNE) {
        if(USART1->ISR & USART_ISR_RTOF) {
            nReceivedBytes = 0;
            USART1->ICR |= USART_ICR_RTOCF; // clear the timeout flag
        }

        uint8_t receivedByte = USART1->RDR;
        if(nReceivedBytes < 65535) {
            nReceivedBytes++;
        }

        if(!commandReceived) {
            if(nReceivedBytes == 1) {
				selectedAxis = receivedByte;
            }
            else if(nReceivedBytes == 2) {
                command = receivedByte;
            }
            else if(nReceivedBytes == 3) {
				valueLength = receivedByte;
				if((selectedAxis != RESPONSE_CHARACTER) && (selectedAxis == global_settings.my_alias || selectedAxis == 255) && (valueLength == 0)) {
					commandReceived = 1;
				}
				nReceivedBytes = 0;
            }
        }
    }
}


static void rs485_transmit_status(uint8_t error_code)
{
	struct device_status_struct *device_status = get_device_status(); 
	uint8_t *device_status_uint8_t = (uint8_t *)device_status;
	uint8_t i;

	set_device_error_code(error_code);

	for(i = 0; i < sizeof(struct device_status_struct); i++) {
		while((USART1->ISR & USART_ISR_TXE_TXFNF_Msk) == 0);
        USART1->TDR = device_status_uint8_t[i];
    }
}


static void handle_commands(uint8_t error_code)
{
	receive(); // this will receive any commands through the RS485 interface

	// we are specifically interested in the reset command
	if(commandReceived) {
		switch(command) {
        case SYSTEM_RESET_COMMAND:
            NVIC_SystemReset();
            break;
		case GET_STATUS_COMMAND:
			rs485_transmit_status(error_code); // this indicates that there was a fatal error and the particular error code is made available for query 
			break;
		}
		commandReceived = 0;
	}
}


static void systick_half_cycle_delay_plus_handle_commands(uint8_t error_code)
{
	do {
		handle_commands(error_code);
		systick_previous_value = systick_new_value;
		systick_new_value = SysTick->VAL;
	} while ((systick_new_value < (SYSTICK_LOAD_VALUE >> 1)) == (systick_previous_value < (SYSTICK_LOAD_VALUE >> 1)));
}


void fatal_error(uint16_t error_code)
{
	uint32_t e;
	char buf[10];
	const char *message;

	// Avoid the fatal error recusrice loop. We will enter the fatal error routine only one, and can get
	// out of it only by full system reset, which can be triggered by the reset command.
	if(fatal_error_occurred) { 
		return;
	}

    __disable_irq();
	heater_off();
    disable_mosfets();
    green_LED_off();
	fatal_error_systick_init();
	fatal_error_occurred = 1;

	message = get_error_text(error_code);
    sprintf(buf, ": %hu\n", error_code);
    while(1) { // print out the error message continuously forever
    	for(e = 0; e < error_code + 3; e++) {
    		if((error_code == 0) || (e < error_code)) {
    			red_LED_on(); // flash the red LED to indicate error, or keep it on continuously if the error code is 0
    		}
			systick_half_cycle_delay_plus_handle_commands(error_code);
			if(error_code != 0) { // if the error code is 0 then we will just leave the red LED on continuously
				red_LED_off();
			}
			transmit_without_interrupts(message, strlen(message));
			systick_half_cycle_delay_plus_handle_commands(error_code);
			transmit_without_interrupts(buf, strlen(buf));
			systick_half_cycle_delay_plus_handle_commands(error_code);
    	}
    }
}

const char *get_error_text(uint16_t error_code)
{
    const char *ptr = error_text;
    uint16_t i;

    for(i = 0; i < error_code; i++) {
        while(*ptr != 0) {
            ptr += 1;
        }
        ptr += 1;
        if(*ptr == 0) {
            return "unknown error";
        }
    }
    return ptr;
}
