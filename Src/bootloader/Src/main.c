#include "stm32g0xx_hal.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "leds.h"
#include "debug_uart.h"
#include "../../RS485.h"
#include "error_handling.h"
#include "../../unique_id.h"
#include "../../settings.h"
#include "../../commands.h"
#include "../../product_info.h"

extern uint32_t USART2_timout_timer;
extern char selectedAxis;
extern uint8_t command;
extern uint8_t valueBuffer[MAX_VALUE_BUFFER_LENGTH];
extern volatile uint8_t commandReceived;
extern uint8_t my_alias;

static uint64_t my_unique_id;
static int16_t detect_devices_delay = -1;

#define LAUNCH_APPLICATION_DELAY 50
static int32_t launch_applicaiton = -1;

// This interrupt will be called 100 times per second
void SysTick_Handler(void)
{
	static uint16_t toggle_counter = 0;

	toggle_counter++;
	if(toggle_counter >= 5) {
	    green_LED_toggle();
		toggle_counter = 0;
	}

    if(USART2_timout_timer < USART2_TIMEOUT) {
    	USART2_timout_timer++;
    }

    if(detect_devices_delay > 0) {
        detect_devices_delay--;
    }

    if(launch_applicaiton > 0) {
        launch_applicaiton--;
    }

//	red_LED_off();
}

void clock_init(void)
{
    FLASH->ACR = (2 << FLASH_ACR_LATENCY_Pos) | FLASH_ACR_PRFTEN | // set two wait states, enable prefetch,
                 FLASH_ACR_ICEN;                                   // enable instruction cache

    #define M_DIVISION_FACTOR 1       // division factor (1 to 8 are valid values)
    #define N_MULTIPLICATION_FACTOR 8 // multiplication factor  (8 to 86 are valid numbers)
    #define PLLPCLK_DIVISION_FACTOR 2 // valid range is 2 to 32 // this clock must not exceed 122MHz
    #define PLLRCLK_DIVISION_FACTOR 2 // valid range is 2 to 8  // this clock must not exceed 64MHz
    // Input frequency from HSI clock is 16MHz
    // VCO frequency = input frequency * N / M
    // VCO frequency must be in the range 64 to 344MHz
    // input frequency / M must be in the range 2.66 to 16 MHz
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI | ((M_DIVISION_FACTOR - 1) << RCC_PLLCFGR_PLLM_Pos) | (N_MULTIPLICATION_FACTOR << RCC_PLLCFGR_PLLN_Pos) |
                 RCC_PLLCFGR_PLLPEN | ((PLLPCLK_DIVISION_FACTOR - 1) << RCC_PLLCFGR_PLLP_Pos) |
                 RCC_PLLCFGR_PLLREN | ((PLLRCLK_DIVISION_FACTOR - 1) << RCC_PLLCFGR_PLLR_Pos);
    RCC->CR = RCC_CR_HSION | RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_HSIRDY) == 0);
    RCC->CFGR = 2 << RCC_CFGR_SW_Pos; // set the system clock to use the PLLRCLK
    while((RCC->CFGR & RCC_CFGR_SW_Msk) != 2);

    RCC->APBENR2 |= (1 << RCC_APBENR2_SYSCFGEN_Pos); // enable the clock to the syscfg (may not be necessary to enable this)
    RCC->IOPENR = 0x3f; // enable the clock to all I/O ports
}

void systick_init(void)
{
    SysTick->LOAD  = 640000;       // set this timer to trigger the interrupt 100 times per second (ie. 64000000 / 100)
    NVIC_SetPriority (SysTick_IRQn, 3); /* set Priority for Systick Interrupt */
    SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_TICKINT_Msk   |
                     SysTick_CTRL_ENABLE_Msk;                         /* Enable SysTick IRQ and SysTick Timer */



/*
    SysTick->CTRL = 0; // disable first in case it is already enabled
    SysTick->LOAD  = (uint32_t)(16000000 - 1);              // set reload register to generate interrupt at 4 Hz
//    HAL_NVIC_SetPriority(SysTick_IRQn, TICK_INT_PRIORITY, 0U);
    SysTick->VAL   = 1600000 - 1;                                             // Load the SysTick Counter Value
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_TICKINT_Msk   |
                     SysTick_CTRL_ENABLE_Msk;                         // Enable SysTick IRQ and SysTick Timer
*/
}

#define MODER_DIGITAL_INPUT 0
#define MODER_DIGITAL_OUTPUT 1
#define MODER_ALTERNATE_FUNCTION 2
#define MODER_ANALOG_INPUT 3 // this is the default after power up
#define OTYPER_PUSH_PULL 0 // this is the default after power up
#define OTYPER_OPEN_DRAIN 1
#define OSPEEDR_VERY_LOW_SPEED 0 // this is the default except some pins on port A
#define OSPEEDR_LOW_SPEED 1
#define OSPEEDR_HIGH_SPEED 2
#define OSPEEDR_VERY_HIGH_SPEED 3
#define PUPDR_NO_PULL_UP_OR_DOWN 0 // this is the default except on some pins on port A
#define PUPDR_PULL_UP 1
#define PUPDR_PULL_DOWN 2

void portA_init(void)
{
	#define BUTTON_PORT_A_PIN 13

    GPIOA->MODER =
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE0_Pos)  | // current measurement channel A
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE1_Pos)  | // RS485 drive enable
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE2_Pos)  | // RS485 Data out
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE3_Pos)  | // RS485 Data receive
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE4_Pos)  | // hall sensor 2
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE5_Pos)  | // hall sensor 1
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE6_Pos)  | // hall sensor 3
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE7_Pos)  | // current measurement channel B
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE8_Pos)  | // PWM 1
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE9_Pos)  |
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE10_Pos) | // PWM 3
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE11_Pos) | // PWM 4
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE12_Pos) | // MOSFET switch disable
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE13_Pos) | // Button input and also SWDIO (for programming)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE14_Pos) | // SWCLK (for programming)
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE15_Pos);  // Potentiometer input

    GPIOA->OTYPER = (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT0_Pos) | // make all the pins with analog components connected open drain
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT3_Pos) | // also, make the RS485 receive pin open drain
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT4_Pos) | // may not be necessary
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT5_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT6_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT7_Pos) |
                    (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT15_Pos);
    GPIOA->OSPEEDR = 0xffffffff; // make all pins very high speed
    GPIOA->PUPDR = (PUPDR_PULL_UP << GPIO_PUPDR_PUPD3_Pos); // apply pull up on the RS485 receive pin
}


void portB_init(void)
{
    GPIOB->MODER =
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE0_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE1_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE2_Pos)  |
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE3_Pos)  | // PWM 2
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE4_Pos)  | // Encoder A input or step input
            (MODER_DIGITAL_INPUT      << GPIO_MODER_MODE5_Pos)  | // Encoder B input or direction input
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE6_Pos)  | // serial port TX
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE7_Pos)  | // serial port RX
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE8_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE9_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE10_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE11_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE12_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE13_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE14_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE15_Pos);

    GPIOB->OTYPER = (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT4_Pos) | // step input make as open drain
    	            (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT5_Pos) | // direction input make as open drain
    		        (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT7_Pos);  // RX pin make as open drain
    GPIOB->OSPEEDR = 0xffffffff; // make all pins very high speed
    GPIOB->PUPDR = (PUPDR_PULL_UP << GPIO_PUPDR_PUPD7_Pos); // no pull up or down except RX pin is pull up
}


void portC_init(void)
{
    GPIOC->MODER =
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE0_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE1_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE2_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE3_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE4_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE5_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE6_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE7_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE8_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE9_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE10_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE11_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE12_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE13_Pos) |
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE14_Pos) | // Red LED
            (MODER_DIGITAL_OUTPUT     << GPIO_MODER_MODE15_Pos);  // Green LED

    GPIOC->OTYPER = 0; // no pins are open drain
    GPIOC->OSPEEDR = 0xffffffff; // very high speed
    GPIOC->PUPDR = 0; // no pins have pulling resistors
}


void portD_init(void)
{
    GPIOD->MODER =
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE0_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE1_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE2_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE3_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE4_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE5_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE6_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE7_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE8_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE9_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE10_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE11_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE12_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE13_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE14_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE15_Pos);

    GPIOD->OTYPER = 0;
    GPIOD->OSPEEDR = 0xffffffff; // very high speed
    GPIOD->PUPDR = 0;
}



void processCommand(uint8_t axis, uint8_t command, uint8_t *parameters)
{
	uint64_t unique_id;
	uint8_t new_alias;
    uint8_t error_code;
    char message[100];
//    sprintf(message, "axis: %hu  command: %hu", axis, command);
//    fatal_error(message, 1);
//    print_number("Received a command with length: ", commandLen);
    if((axis == my_alias) || (axis == ALL_ALIAS)) {
        launch_applicaiton = -1; // cancel the launching of the apllicaiton in case it is pending so that we can upload a new firmware
//        print_number("Axis:", axis);
//        print_number("command:", command);
        switch(command) {
        case DETECT_DEVICES_COMMAND:
//            sprintf(message, "DETECT_DEVICES_COMMAND\n");
//            transmit(message, strlen(message));
        	detect_devices_delay = get_random_number(99);
			break;
        case SET_DEVICE_ALIAS_COMMAND:
//            sprintf(message, "SET_DEVICE_ALIAS_COMMAND\n");
//            transmit(message, strlen(message));
            unique_id = ((int64_t*)parameters)[0];
            new_alias = parameters[8];

//            uint32_t my_unique_id_u32_array[2];
//            memcpy(my_unique_id_u32_array, &unique_id, sizeof(unique_id));
//            sprintf(message, "Unique ID: 0x%08lX%08lX\n", my_unique_id_u32_array[1], my_unique_id_u32_array[0]);
//            transmit(message, strlen(message));

        	if(unique_id == my_unique_id) {
                transmit("Match\n", 6);
        		my_alias = new_alias;
           		save_settings(my_alias);
                rs485_transmit(NO_ERROR_RESPONSE, 3);
        	}
        	break;
        case FIRMWARE_UPGRADE_COMMAND:
            sprintf(message, "FIRMWARE_UPGRADE_COMMAND page %hu\n", parameters[0]);
            transmit(message, strlen(message));
            error_code = burn_firmware_page(parameters[0], parameters + 1);
			if((axis != ALL_ALIAS) && (error_code == 0)) {
                rs485_transmit(NO_ERROR_RESPONSE, 3);
			}
			break;
        case GET_PRODUCT_INFO_COMMAND:
			if(axis != ALL_ALIAS) {
				rs485_transmit("R\x01", 2);
                struct product_info_struct *product_info = (struct product_info_struct *)(PRODUCT_INFO_MEMORY_LOCATION);
				uint8_t product_info_length = sizeof(struct product_info_struct);
				rs485_transmit(&product_info_length, 1);
				rs485_transmit(product_info, sizeof(struct product_info_struct));
			}
			break;
        case SYSTEM_RESET_COMMAND:
            NVIC_SystemReset();
            break;
        }
    }
}

void transmit_unique_id(void)
{
    uint32_t crc32 = 0x04030201;
	rs485_transmit("R\x01\x0d", 3);
	rs485_transmit(&my_unique_id, 8);
	rs485_transmit(&my_alias, 1);
	rs485_transmit(&crc32, 4);
}


void process_debug_uart_commands(void)
{
    uint8_t command_uart1 = get_command_uart1();

    if(command_uart1 != 0) {
    	switch(command_uart1) {
    	case 'S':
			transmit("Saving settings\n", 16);
    		save_settings(my_alias);
    		break;
		}
    	command_uart1 = 0;
	}
}

void print_start_message()
{
	char buff[200];
	uint32_t my_unique_id_u32_array[2];

	memcpy(my_unique_id_u32_array, &my_unique_id, sizeof(my_unique_id));

    transmit("Bootloader Start\n", 17);
    sprintf(buff, "Unique ID: 0x%08lX%08lX\n", my_unique_id_u32_array[1], my_unique_id_u32_array[0]);
    transmit(buff, strlen(buff));
    if((my_alias >= 33) && (my_alias <= 126)) {
        sprintf(buff, "Alias: %c\n", my_alias);
    }
    else {
        sprintf(buff, "Alias: 0x%02hx\n", my_alias);
    }
    transmit(buff, strlen(buff));
}

int main(void)
{
//	volatile int i;
    typedef void (*pFunction)(void);
    pFunction jumpToApplication = 0;

    clock_init();
    systick_init();
    portA_init();
    portB_init();
    portC_init();
    portD_init();
    debug_uart_init();
    rs485_init();

    SCB->VTOR = 0; // vector table for the bootloader is located at the normal location

    my_unique_id = get_unique_id();

    load_settings(&my_alias); // load the settings from non-volatile memory

    __enable_irq();

    print_start_message();

//    uint32_t buffer[4];
    char message[100];
//    buffer[0] = 0;
//    uint32_t crc32 = calculate_crc32_buffer(buffer, 1);
//    sprintf(message, "crc32: %08lX\n", crc32);
    sprintf(message, "firmware size: %lu\n", get_firmware_size());
    transmit(message, strlen(message));


    if(firmware_crc_check()) {
        transmit("Firmware CRC check passed\n", 26);
        uint32_t jumpAddress = *(__IO uint32_t*)(APPLICATION_ADDRESS_PTR);
        jumpToApplication = (pFunction)jumpAddress; 
        sprintf(message, "Application start address: %08lx\n", jumpAddress);
        transmit(message, strlen(message));
        launch_applicaiton = LAUNCH_APPLICATION_DELAY; // launch the application after a short delay
    }
    else {
        transmit("Firmware CRC check failed\n", 26);
    }

    rs485_transmit("Start\n", 6);

    while(1) {
    	if(commandReceived) {
            processCommand(selectedAxis, command, valueBuffer);
            commandReceived = 0;
        }

        if(detect_devices_delay == 0) {
            transmit_unique_id();
    		detect_devices_delay--;
    	}

        if(launch_applicaiton == 0) {
            // the bootloader code will no longer be valid. the application code will reset the stack pointer and reinitialize things
            __disable_irq();
            jumpToApplication();
        }

        process_debug_uart_commands();
    }
}
