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
static volatile uint8_t reset_requested = 0;
static volatile uint8_t if_fatal_error_then_respond_flag = 0;


static void fatal_error_systick_init(void)
{
    SysTick->LOAD  = SYSTICK_LOAD_VALUE;       // set this timer to roll over twice per second
    SysTick->VAL   = 0;       // Load the SysTick Counter Value
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;  // Enable SysTick Timer
}


#ifdef MOTOR_SIMULATION
#include <time.h>
// Get current time in the number of MCU clock cycles (which runs at 64MHz)
static uint64_t get_monotonic_64MHz_tick(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 64000000ULL + (uint64_t)ts.tv_nsec * 1000ULL / 15625ULL;
}


extern uint32_t get_updated_systick_val(void)
{
    uint64_t current_time = get_monotonic_64MHz_tick();
    return current_time % 16000000;
}
#endif


static void handle_packet(void)
{
    uint8_t command;
    uint16_t payload_size;
    uint8_t *payload;
    uint8_t is_broadcast;

    // There is a distinct possibility that the new packet is not for us. We will know after we see the return value of rs485_get_next_packet.
    if (!rs485_get_next_packet(&command, &payload_size, &payload, &is_broadcast)) {
        // The new packet, whatever it is, is not of interest to us and we need to clear it and return out of here
        rs485_done_with_this_packet_dont_disable_enable_irq();
        return;
    }

    if(!rs485_validate_packet_crc32()) {
        // CRC32 validation failed, allow next command and return
        rs485_done_with_this_packet_dont_disable_enable_irq();
        return;
    }

    switch(command) {
    case SYSTEM_RESET_COMMAND:
        rs485_done_with_this_packet_dont_disable_enable_irq();
        if ((payload_size == 0) && !rs485_transmit_not_done()) { // we won't transmit if already transmitting (to prevent a deadlock inside the while(transmitCount > 0); statement inside the rs485_transmit function)
            rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
            reset_requested = 1;
        }
        break;
    case GET_STATUS_COMMAND:
        rs485_done_with_this_packet_dont_disable_enable_irq();
        if((payload_size == 0) && !is_broadcast && !rs485_transmit_not_done()) { // we won't transmit if already transmitting (to prevent a deadlock inside the while(transmitCount > 0); statement inside the rs485_transmit function)
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                struct device_status_struct device_status;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } status_reply;
            memcpy(&status_reply.device_status, get_device_status(), sizeof(struct device_status_struct));
            rs485_finalize_and_transmit_packet(&status_reply, sizeof(status_reply));
        }
        break;
    default:
        rs485_done_with_this_packet_dont_disable_enable_irq();
        if(!is_broadcast && !rs485_transmit_not_done()) { // we won't transmit if already transmitting (to prevent a deadlock inside the while(transmitCount > 0); statement inside the rs485_transmit function)
            struct device_status_struct *device_status = get_device_status();
            rs485_transmit_error_packet(device_status->error_code); // we will send the error response (since we are in a fatal error state) in response to any command given (except the above two commands) and that will let the user know what is the error code
        }
        break;
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

        if (if_fatal_error_then_respond_flag) {
            rs485_transmit_error_packet(error_code);
            if_fatal_error_then_respond_flag = 0;
        }

        // Since interrupts are now disabled to minimize the chance of any abnormal behaviour during a fata error state,
        // we need to call the interrupt routine that handle receive and transmit over the TS485 interface manually
        USART1_IRQHandler();

        if (rs485_has_a_packet()) {
            handle_packet();
        }

        if ((reset_requested) && !rs485_transmit_not_done()) {
            #ifdef MOTOR_SIMULATION
            printf("********** We will reset now\n");
            #endif
            NVIC_SystemReset();
        }
        
        systick_previous_value = systick_new_value;
        #if MOTOR_SIMULATION
        extern uint32_t get_updated_systick_val(void);
        SysTick->VAL = get_updated_systick_val();
        #endif
        systick_new_value = SysTick->VAL;
    } while ((systick_new_value < (SYSTICK_LOAD_VALUE >> 1)) == (systick_previous_value < (SYSTICK_LOAD_VALUE >> 1)));
}


void if_fatal_error_then_respond(void)
{
    if_fatal_error_then_respond_flag = 1;
}


void if_fatal_error_then_dont_respond(void)
{
    if_fatal_error_then_respond_flag = 0;
}


void fatal_error(uint16_t error_code)
{
    uint32_t e;
    char buf[10];
//    const char *message; // DEBUG commented out

#ifdef MOTOR_SIMULATION
//    gExitFatalError = 0; // Clear the exit fatal error flag at the beginning
    
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
    #ifdef MOTOR_SIMULATION
    printf("__disable_irq() g_interrupts_enabled = %hhu\n", g_interrupts_enabled);
    static uint8_t is_red_LED_on = 0;
    #endif
    heater_off();
    disable_mosfets();
    green_LED_off();
    fatal_error_systick_init();
//    rs485_init(); // reinitialize the UART, just in case it got corrupted. maybe don't need to do this if the risk of curruption is low.
    fatal_error_occurred = 1;
    set_device_error_code(error_code);

//    message = get_error_text(error_code); // DEBUG commented out
    sprintf(buf, ": %u\n", error_code);
    while(1) { // print out the error message continuously forever
        for(e = 0; e < error_code + 3; e++) {
            #ifdef MOTOR_SIMULATION
//            printf("__disable_irq() g_interrupts_enabled = %hhu\n", g_interrupts_enabled);
            #endif
            if((error_code == 0) || (e < error_code)) {
                red_LED_on(); // flash the red LED to indicate error, or keep it on continuously if the error code is 0
                #ifdef MOTOR_SIMULATION
                if (is_red_LED_on == 0) {
                    printf("**** RED LED FLASH ***\n");
                    is_red_LED_on = 1;
                }
                #endif
            }
            systick_half_cycle_delay_plus_handle_commands(error_code);
            #ifdef MOTOR_SIMULATION
//            printf("__disable_irq() (2) g_interrupts_enabled = %hhu\n", g_interrupts_enabled);
            #endif
            if(error_code != 0) { // if the error code is 0 then we will just leave the red LED on continuously
                red_LED_off();
                #ifdef MOTOR_SIMULATION
                is_red_LED_on = 0;
                #endif
            }
//            transmit_without_interrupts(message, strlen(message));  // DEBUG commented out
            for (uint32_t led_off_time_counter = 0; led_off_time_counter < 5; led_off_time_counter++) {
                systick_half_cycle_delay_plus_handle_commands(error_code);
            }
            
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

    systick_new_value = 0;
    fatal_error_occurred = 0;
    reset_requested = 0;
    if_fatal_error_then_respond_flag = 0;

    printf("After reset: fatal_error_occurred = %d\n", fatal_error_occurred);
}
#endif
