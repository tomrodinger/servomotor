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

extern void USART1_IRQHandler(void);

// External declarations from servo_simulator.c
#ifdef MOTOR_SIMULATION
//extern volatile int gExitFatalError;
extern volatile int gResetProgress;
#endif

// Error text array initialization using macro from error_text.h
static const char error_text[] = ERROR_TEXT_INITIALIZER;

#define SYSTICK_LOAD_VALUE 16000000 // this must be less than 2^24 because the systick timer is 24 bits
#define FATAL_ERROR_COMMAND_BUFFER_SIZE 3

static volatile uint32_t systick_new_value = SYSTICK_LOAD_VALUE;
static volatile uint32_t systick_previous_value;
static uint8_t fatal_error_occurred = 0;

static void fatal_error_systick_init(void)
{
    SysTick->LOAD  = SYSTICK_LOAD_VALUE;       // set this timer to roll over twice per second
    SysTick->VAL   = 0;       // Load the SysTick Counter Value
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;  // Enable SysTick Timer
}


static void handle_packet(void)
{
    uint8_t command;
    uint16_t payload_size;
    uint8_t *payload;
    uint8_t is_broadcast;

    // There is a distinct possibility that the new packet is not for us. We will know after we see the return value of rs485_get_next_packet.
    if (!rs485_get_next_packet(&command, &payload_size, &payload, &is_broadcast)) {
        // The new packet, whatever it is, is not of interest to us and we need to clear it and return out of here
        rs485_done_with_this_packet();
        return;
    }

    if(!rs485_validate_packet_crc32()) {
        // CRC32 validation failed, allow next command and return
        rs485_done_with_this_packet();
        return;
    }

    switch(command) {
    case SYSTEM_RESET_COMMAND:
        NVIC_SystemReset();
        break;
    case GET_STATUS_COMMAND:
        rs485_done_with_this_packet();
        if(!is_broadcast && rs485_is_transmit_done()) { // we won't transmit if already transmitting (to prevent a deadlock inside the while(transmitCount > 0); statement inside the rs485_transmit function)
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                struct device_status_struct device_status;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } status_reply;
            memcpy(&status_reply.device_status, get_device_status(), sizeof(struct device_status_struct));
            rs485_finalize_and_transmit_packet(&status_reply, sizeof(status_reply));
        }
    }
}


static void systick_half_cycle_delay_plus_handle_commands(uint8_t error_code)
{
    do {
        #ifdef MOTOR_SIMULATION
        // Check if we should exit the fatal error state
        if (gResetProgress > 0) {
            return;
        }
        #endif

        // Since interrupts are now disabled to minimize the chance of any abnormal behaviour during a fata error state,
        // we need to call the interrupt routine that handle receive and transmit over the TS485 interface manually
        USART1_IRQHandler();

        if (rs485_has_a_packet()) {
            handle_packet();
        }
        
        systick_previous_value = systick_new_value;
        systick_new_value = SysTick->VAL;
    } while ((systick_new_value < (SYSTICK_LOAD_VALUE >> 1)) == (systick_previous_value < (SYSTICK_LOAD_VALUE >> 1)));
}


void fatal_error(uint16_t error_code)
{
    uint32_t e;
    char buf[10];
    const char *message;

#ifdef MOTOR_SIMULATION
    // Clear the exit fatal error flag at the beginning
//    gExitFatalError = 0;
    
    const char *debug_message;
    debug_message = get_error_text(error_code);
    printf("\n\n*** FATAL ERROR %d: %s ***\n\n", error_code, debug_message);
    
    // Try to identify what's calling fatal_error during reset
    if (gResetProgress > 0) {
        printf("IMPORTANT: fatal_error called during reset process (gResetProgress = %d)!\n", gResetProgress);
    }
    // exit(1); - Removed to allow the program to continue running and respond to SYSTEM_RESET_COMMAND
#endif

    // Avoid the fatal error recursive loop. We will enter the fatal error routine only once, and can get
    // out of it only by full system reset, which can be triggered by the reset command.
    if(fatal_error_occurred) {
        #ifdef MOTOR_SIMULATION
        printf("Fatal error already occurred, returning early\n");
        #endif
        return;
    }
    
    __disable_irq();
    heater_off();
    disable_mosfets();
    green_LED_off();
    fatal_error_systick_init();
    rs485_init();
    fatal_error_occurred = 1;
    set_device_error_code(error_code);

    message = get_error_text(error_code);
    sprintf(buf, ": %u\n", error_code);
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
            
            #ifdef MOTOR_SIMULATION
            // Check if we should exit the fatal error state
            if (gResetProgress > 0) {
//                fatal_error_occurred = 0;  // Clear the fatal error state
//                gExitFatalError = 0;
                return;
            }
            #endif
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

#ifdef MOTOR_SIMULATION
/**
 * Initialize all static variables in error_handling.c for simulator use
 * This function should be called when the simulator starts to ensure
 * all static variables are properly initialized
 */
void error_handling_simulator_init(void)
{
    printf("error_handling_simulator_init() called\n");
    printf("Before reset: fatal_error_occurred = %d\n", fatal_error_occurred);

    systick_new_value = SYSTICK_LOAD_VALUE;
    fatal_error_occurred = 0;
    
    printf("After reset: fatal_error_occurred = %d\n", fatal_error_occurred);
}
#endif
