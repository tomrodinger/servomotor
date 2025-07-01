#include "stm32g0xx_hal.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "gpio.h"
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

// Here we store the version information for the bootloader firmware (not the main firmware)
struct __attribute__((__packed__)) firmware_version_struct {
    uint8_t development;
    uint8_t bugfix;
    uint8_t minor;
    uint8_t major;
};
#define MAJOR_FIRMWARE_VERSION 1
#define MINOR_FIRMWARE_VERSION 1
#define BUGFIX_FIRMWARE_VERSION 0
#define DEVELOPMENT_FIRMWARE_VERSION 0 // this is the least significant number when it comes to versioning and is the last number on the right when printed in human readable form
struct firmware_version_struct firmware_version = {DEVELOPMENT_FIRMWARE_VERSION, BUGFIX_FIRMWARE_VERSION, MINOR_FIRMWARE_VERSION, MAJOR_FIRMWARE_VERSION};

extern uint32_t USART1_timout_timer;

static uint64_t my_unique_id;
static int16_t detect_devices_delay = -1;

#define LAUNCH_APPLICATION_DELAY 25 // It was empirically determined that the minimum delay here is 12 (120ms), otherwise firmware upgrade will not work. For a lot of margin, setting this to 250ms
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

void copy_input_parameters_and_check_size(void *destination, const void *source, uint32_t destination_size, uint32_t source_size)
{
    if (destination_size != source_size) {
        fatal_error(ERROR_COMMAND_SIZE_WRONG);
    }
    memcpy(destination, source, destination_size);
}

void check_payload_size_is_zero(uint16_t payload_size)
{
    if (payload_size != 0) {
        fatal_error(ERROR_COMMAND_SIZE_WRONG);
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

    launch_applicaiton = -1; // cancel the launching of the apllicaiton in case it is pending so that we can upload a new firmware

    switch(command) {
    case GET_STATUS_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        if(!is_broadcast) {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                struct device_status_struct status;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } status_reply;
            memcpy(&status_reply.status, get_device_status(), sizeof(struct device_status_struct));
            rs485_finalize_and_transmit_packet(&status_reply, sizeof(status_reply));
        }
        break;
    case DETECT_DEVICES_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        detect_devices_delay = get_random_number(99);
        break;
    case SET_DEVICE_ALIAS_COMMAND:
        {
            uint8_t new_alias = 0;
            copy_input_parameters_and_check_size(&new_alias, payload, sizeof(new_alias), payload_size);
            rs485_done_with_this_packet();
            if ( (new_alias == EXTENDED_ADDRESSING              ) ||
                 (new_alias == RESPONSE_CHARACTER_CRC32_ENABLED ) ||
                 (new_alias == RESPONSE_CHARACTER_CRC32_DISABLED)    ) {
                fatal_error(ERROR_BAD_ALIAS);
            }
            rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
            transmit("Match\n", 6);
            rs485_wait_for_transmit_done(); // make sure that the no error packet is sent out
            global_settings.my_alias = new_alias;
            save_global_settings();
        }
        break;
    case FIRMWARE_UPGRADE_COMMAND:
        {   
            if (payload_size < MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH) {
                transmit("This firmware payload is too small\n", 35);
            }
            else {
                struct product_info_struct *product_info = (struct product_info_struct *)(PRODUCT_INFO_MEMORY_LOCATION);
                // Check model code match
                if(memcmp(payload, product_info->model_code, MODEL_CODE_LENGTH) != 0) {
                    transmit("This firmware is not for this model\n", 36);
                }
                else {
                    // Check firmware compatibility code
                    if(*(payload + MODEL_CODE_LENGTH) != product_info->firmware_compatibility_code) {
                        transmit("Firmware compatibility code mismatch\n", 37);
                    }
                    else {
                        uint8_t firmware_page;
                        // For firmware upgrade, we expect a specific fixed size
                        uint16_t expected_size = MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH + sizeof(firmware_page) + FLASH_PAGE_SIZE;            
                        if (payload_size != expected_size) {
                            fatal_error(ERROR_COMMAND_SIZE_WRONG);
                        }
                        firmware_page = *(payload + MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH);
                        char message[100]; // Local message buffer for this case
                        sprintf(message, "Valid firmware page %hu\n", firmware_page);
                        transmit(message, strlen(message));
                        
                        // Burn the firmware directly from the payload buffer
                        uint8_t error_code = burn_firmware_page(firmware_page, payload + MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH + sizeof(firmware_page));
                        if (error_code != 0) {
                            fatal_error(ERROR_INVALID_FLASH_PAGE);
                        }
                        rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
                    }
                }
            }            
            rs485_done_with_this_packet();
        }
        break;
    case GET_PRODUCT_INFO_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        if(!is_broadcast) {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                struct product_info_struct product_info;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } product_info_reply;
            struct product_info_struct *product_info = (struct product_info_struct *)(PRODUCT_INFO_MEMORY_LOCATION);
            memcpy(&product_info_reply.product_info, product_info, sizeof(struct product_info_struct));
            rs485_finalize_and_transmit_packet(&product_info_reply, sizeof(product_info_reply));
        }
        break;
    case GET_FIRMWARE_VERSION_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        if(!is_broadcast) {
            struct __attribute__((__packed__)) {
                uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
                struct firmware_version_struct firmware_version_data;
                uint8_t in_bootloader;
                uint32_t crc32; // this part will be filled in by rs485_finalize_and_transmit_packet()
            } firmware_version_reply;
            memcpy(&firmware_version_reply.firmware_version_data, &firmware_version, sizeof(firmware_version));
            firmware_version_reply.in_bootloader = 1; // 1 indicates that we are in the bootloader and the firmware version we are returning is the version of the bootloader
            rs485_finalize_and_transmit_packet(&firmware_version_reply, sizeof(firmware_version_reply));
        }
        break;
    case SYSTEM_RESET_COMMAND:
        check_payload_size_is_zero(payload_size);
        rs485_done_with_this_packet();
        rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        rs485_wait_for_transmit_done(); // make sure that the no error packet is sent out before resetting the device
        NVIC_SystemReset();
        break;
    case TEST_MODE_COMMAND:
        {
            uint8_t test_mode;
            copy_input_parameters_and_check_size(&test_mode, payload, sizeof(test_mode), payload_size);
            rs485_done_with_this_packet();
            if ((test_mode >= 12) && (test_mode < 12 + 60)) { // test modes 12 to 71 are for triggering fatal errors 0 to 59, for testing if fatal errors are working correctly
                fatal_error(test_mode - 12);
            }
            else {
                fatal_error(ERROR_INVALID_TEST_MODE);
            }
            rs485_transmit_no_error_packet(is_broadcast); // nothing will be transmitted if is_broadcast is true
        }
        break;
    default:
        rs485_done_with_this_packet();
        break;
    }
}

void transmit_detect_devices_response(void)
{
    struct __attribute__((__packed__)) {
        uint8_t header[3]; // this part will be filled in by rs485_finalize_and_transmit_packet()
        uint64_t unique_id;
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
    typedef void (*pFunction)(void);
    char message[100];
    pFunction jumpToApplication = 0;

    clock_init();
    systick_init();
    GPIO_init(); // Using product-specific GPIO initialization. gpio.h will select a specific init function for the product. Look at files like gpio_M1.c, gpio_M2.c, gpio_M3.c, gpio_M4.c, etc.
    debug_uart_init();
    rs485_init();

    SCB->VTOR = 0; // vector table for the bootloader is located at the normal location

    my_unique_id = get_unique_id();

    load_global_settings(); // load the settings from non-volatile memory

    set_device_status_flags(1 << STATUS_IN_THE_BOOTLOADER_FLAG_BIT); // We are in the bootloader and our device status needs to reflect it

    __enable_irq();

    print_start_message();

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
            launch_applicaiton = LAUNCH_APPLICATION_DELAY; // launch the application after a short delay
        }
        else {
            transmit("Firmware CRC check failed\n", 26);
        }
    }

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
            transmit_detect_devices_response();
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
