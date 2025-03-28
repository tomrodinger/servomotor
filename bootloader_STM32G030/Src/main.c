#include "stm32g0xx_hal.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "leds.h"
#include "debug_uart.h"
#include "RS485.h"
#include "error_handling.h"
#include "unique_id.h"
#include "settings.h"
#include "global_variables.h"
#include "commands.h"
#include "product_info.h"
#include "device_status.h"
#include "crc32.h"

extern uint32_t USART1_timout_timer;

static uint64_t my_unique_id;
static int16_t detect_devices_delay = -1;

#define LAUNCH_APPLICATION_DELAY 50
static int32_t launch_applicaiton = -1;


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
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE1_Pos)  | // MOSFET switch disable
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE2_Pos)  | // serial port TX
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE3_Pos)  | // serial port RX
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE4_Pos)  | // hall sensor 2
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE5_Pos)  | // hall sensor 1
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE6_Pos)  | // hall sensor 3
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE7_Pos)  | // current measurement channel B
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE8_Pos)  | // PWM 1
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE9_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE10_Pos) | // PWM 3
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE11_Pos) | // PWM 4
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE12_Pos) | // RS485 drive enable
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
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE3_Pos)  | // PWM 2
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE4_Pos)  | // Encoder A input or step input
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE5_Pos)  | // Encoder B input or direction input
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE6_Pos)  | // RS485 Data out
            (MODER_ALTERNATE_FUNCTION << GPIO_MODER_MODE7_Pos)  | // RS485 Data receive
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE8_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE9_Pos)  |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE10_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE11_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE12_Pos) | // overvoltage digital input
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE13_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE14_Pos) |
            (MODER_ANALOG_INPUT       << GPIO_MODER_MODE15_Pos);

    GPIOB->OTYPER = (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT4_Pos) | // step input make as open drain
    	            (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT5_Pos) | // direction input make as open drain
    		        (OTYPER_OPEN_DRAIN << GPIO_OTYPER_OT7_Pos);  // RX pin make as open drain
    GPIOB->OSPEEDR = 0xffffffff; // make all pins very high speed
    GPIOB->PUPDR = (PUPDR_PULL_UP << GPIO_PUPDR_PUPD7_Pos) | (PUPDR_PULL_DOWN << GPIO_PUPDR_PUPD12_Pos); // RX pin is pull up, overvoltage pin is pull down
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


// This interrupt will be called 100 times per second
void SysTick_Handler(void)
{
	static uint16_t toggle_counter = 0;

	toggle_counter++;
	if(toggle_counter >= 5) {
	    green_LED_toggle();
		toggle_counter = 0;
	}

    if(detect_devices_delay > 0) {
        detect_devices_delay--;
    }

    if(launch_applicaiton > 0) {
        launch_applicaiton--;
    }
}

void process_packet(void)
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

//    print_number("Received a command with length: ", commandLen);
    launch_applicaiton = -1; // cancel the launching of the apllicaiton in case it is pending so that we can upload a new firmware
//        print_number("Axis:", axis);
//        print_number("command:", command);
    switch(command) {
    case DETECT_DEVICES_COMMAND:
        rs485_done_with_this_packet();
//            sprintf(message, "DETECT_DEVICES_COMMAND\n");
//            transmit(message, strlen(message));
        detect_devices_delay = get_random_number(99);
        break;
    case SET_DEVICE_ALIAS_COMMAND:
        {
        //            sprintf(message, "SET_DEVICE_ALIAS_COMMAND\n");
        //            transmit(message, strlen(message));
            uint64_t unique_id = ((int64_t*)payload)[0];
            uint8_t new_alias = payload[8];
            rs485_done_with_this_packet();

        //            uint32_t my_unique_id_u32_array[2];
        //            memcpy(my_unique_id_u32_array, &unique_id, sizeof(unique_id));
        //            sprintf(message, "Unique ID: 0x%08lX%08lX\n", my_unique_id_u32_array[1], my_unique_id_u32_array[0]);
        //            transmit(message, strlen(message));

            if(unique_id == my_unique_id) {
                transmit("Match\n", 6);
                global_settings.my_alias = new_alias;
                save_global_settings();
                rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
            }
        }
        break;
    case FIRMWARE_UPGRADE_COMMAND:
        {
            struct product_info_struct *product_info = (struct product_info_struct *)(PRODUCT_INFO_MEMORY_LOCATION);
            if(memcmp(payload, product_info->model_code, MODEL_CODE_LENGTH) != 0) {
                transmit("This firmware is not for this model\n", 36);
            }
            else {
                if(*(payload + MODEL_CODE_LENGTH) != product_info->firmware_compatibility_code) {
                    transmit("Firmware compatibility code mismatch\n", 37);
                }
                else {
                    uint8_t firmware_page = *(payload + MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH);
                    char message[100];
                    sprintf(message, "Valid firmware page %hu\n", firmware_page);
                    transmit(message, strlen(message));
                    uint8_t error_code = burn_firmware_page(firmware_page, payload + MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH + sizeof(firmware_page));
                    if (error_code == 0) {
                        rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
                    }
                }
            }
            rs485_done_with_this_packet();
        }
        break;
    case GET_PRODUCT_INFO_COMMAND:
        rs485_done_with_this_packet();
        if(!is_broadcast) {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                uint8_t product_info_length;
                struct product_info_struct product_info;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } product_info_reply;
            
            product_info_reply.product_info_length = sizeof(struct product_info_struct);
            memcpy(&product_info_reply.product_info, (struct product_info_struct *)(PRODUCT_INFO_MEMORY_LOCATION), sizeof(struct product_info_struct));
            rs485_finalize_and_transmit_packet(&product_info_reply, sizeof(product_info_reply));
        }
        break;
    case GET_STATUS_COMMAND:
        rs485_done_with_this_packet();
        if(!is_broadcast) {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                struct device_status_struct status;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } status_reply;
            
            set_device_status_flags(1 << STATUS_IN_THE_BOOTLOADER_FLAG_BIT);
            memcpy(&status_reply.status, get_device_status(), sizeof(struct device_status_struct));
            rs485_finalize_and_transmit_packet(&status_reply, sizeof(status_reply));
        }
        break;
    case SYSTEM_RESET_COMMAND:
        NVIC_SystemReset();
        break;
    default:
        rs485_done_with_this_packet();
        break;
    }
}


void transmit_unique_id(void)
{
    struct __attribute__((__packed__)) {
        uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
        uint32_t unique_id;
        uint8_t alias;
        uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
    } detect_devices_reply;
    detect_devices_reply.unique_id = my_unique_id;
    detect_devices_reply.alias = global_settings.my_alias;
    rs485_finalize_and_transmit_packet(&detect_devices_reply, sizeof(detect_devices_reply));
}


void process_debug_uart_commands(void)
{
    uint8_t command_debug_uart = get_command_from_debug_uart();

    if(command_debug_uart != 0) {
    	switch(command_debug_uart) {
    	case 'S':
			transmit("Saving settings\n", 16);
    		save_global_settings();
    		break;
		}
    	command_debug_uart = 0;
	}
}

void print_start_message()
{
	char buff[200];
	uint32_t my_unique_id_u32_array[2];
    struct product_info_struct *product_info = (struct product_info_struct *)(PRODUCT_INFO_MEMORY_LOCATION);

	memcpy(my_unique_id_u32_array, &my_unique_id, sizeof(my_unique_id));

    transmit("Bootloader start\n", 17);
    transmit("Model code: ", 12);
    transmit(&product_info->model_code, 8);
    transmit("\n", 1);
    sprintf(buff, "Firmware compatibility code: %hu\n", product_info->firmware_compatibility_code);
    transmit(buff, strlen(buff));
    sprintf(buff, "Unique ID: 0x%08lX%08lX\n", my_unique_id_u32_array[1], my_unique_id_u32_array[0]);
    transmit(buff, strlen(buff));
    if((global_settings.my_alias >= 33) && (global_settings.my_alias <= 126)) {
        sprintf(buff, "Alias: %c\n", global_settings.my_alias);
    }
    else {
        sprintf(buff, "Alias: 0x%02hx\n", global_settings.my_alias);
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

    load_global_settings(); // load the settings from non-volatile memory

    __enable_irq();

    print_start_message();

//    uint32_t buffer[4];
    char message[100];
//    buffer[0] = 0;
//    uint32_t crc32 = calculate_crc32_buffer(buffer, 1);
//    sprintf(message, "crc32: %08lX\n", crc32);
    uint32_t firmware_size = get_firmware_size();
    if(firmware_size == 0xFFFFFFFF) {
        transmit("No application firmware is present\n", 35);
    }
    else {
        sprintf(message, "Firmware size: %lu bytes\n", (firmware_size << 2));
        transmit(message, strlen(message));
        if(firmware_crc_check()) {
            transmit("Firmware CRC check passed\n", 26);
            uint32_t jumpAddress = *(__IO uint32_t*)(APPLICATION_ADDRESS_PTR);
            jumpToApplication = (pFunction)jumpAddress; 
    //        sprintf(message, "Application start address: %08lx\n", jumpAddress);
    //        transmit(message, strlen(message));
            launch_applicaiton = LAUNCH_APPLICATION_DELAY; // launch the application after a short delay
        }
        else {
            transmit("Firmware CRC check failed\n", 26);
        }
    }


//    rs485_transmit("Start\n", 6);

    while(1) {
        if(rs485_has_a_packet()) {
            if(detect_devices_delay >= 0) { // if a DETECT_DEVICES_COMMAND has been issued then we will ignore all other commands until the delay is over and we send out the unique ID
                rs485_done_with_this_packet();
            }
            else {
                process_packet();
            }
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
