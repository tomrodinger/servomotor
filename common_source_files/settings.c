#include <string.h>
#include "stm32g0xx_hal.h"
#include "settings.h"
#include "global_variables.h"
#include "error_handling.h"


void load_global_settings(void)
{
    memcpy(&global_settings, (void *)GLOBAL_SETTINGS_FLASH_ADDRESS, GLOBAL_SETTINGS_STRUCT_SIZE);
}

inline void crc32_init(void)
{
    RCC->AHBENR |= RCC_AHBENR_CRCEN; // enable the clock to the CRC calculation unit
    CRC->CR = CRC_CR_RESET_Msk | CRC_CR_REV_OUT_Msk | CRC_CR_REV_IN_0 | CRC_CR_REV_IN_1; //reset the CRC hardware, 4 byte mode, reversal
}

inline uint32_t calculate_crc32(uint32_t new_value)
{
    CRC->DR = new_value;
    return ~CRC->DR;
}

uint8_t firmware_crc_check(void)
{
    uint32_t firmware_size = *((uint32_t *)(FIRMWARE_START_ADDRESS));
    uint32_t *firmware_pointer = ((uint32_t *)(FIRMWARE_START_ADDRESS)) + 1;
    uint32_t calculated_crc32 = 0;
    uint32_t expected_crc32;
    uint32_t i;

    // A valid firmware must not be bigger than the size of application firmware memory minus the size of the firmware size value minus the size of the crc32 value
    if(firmware_size > (LAST_FIRMWARE_PAGE_NUMBER + 1) * FLASH_PAGE_SIZE - sizeof(uint32_t) * 2) {
        return 0;
    }
    crc32_init();
    for(i = 0; i < firmware_size; i++) {
        calculated_crc32 = calculate_crc32(*(firmware_pointer++));
    }
    expected_crc32 = *firmware_pointer;
    return (calculated_crc32 == expected_crc32);
}

uint32_t calculate_crc32_buffer(uint8_t *buffer, uint32_t len)
{
    uint32_t i;

    crc32_init();
    for(i = 0; i < len; i++) {
        ((uint8_t *)(&CRC->DR))[0] = *(buffer++);
    }
    return CRC->DR;
}

uint32_t get_firmware_size(void)
{
    uint32_t firmware_size = *((uint32_t *)(FIRMWARE_START_ADDRESS));
    return firmware_size;
}

__attribute__ ((long_call, section (".RamFunc")))
static void unlock_flash(void)
{
    FLASH->KEYR = 0x45670123; // unlock sequence
    FLASH->KEYR = 0xCDEF89AB; // unlock sequence
    while(FLASH->SR & FLASH_SR_BSY1_Msk);

    if(FLASH->CR & FLASH_CR_LOCK_Msk) {
        fatal_error(2); // "flash unlock fail" (all error text is defined in error_text.c)
    }
}

__attribute__ ((long_call, section (".RamFunc")))
static inline void lock_flash(void)
{
    FLASH->CR = FLASH_CR_LOCK;
}

__attribute__ ((long_call, section (".RamFunc")))
static void erase_one_page(uint8_t page_number)
{
    // First, erase the page
    FLASH->CR = FLASH_CR_PER | (page_number << FLASH_CR_PNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    while(FLASH->SR & FLASH_SR_BSY1_Msk);
}


__attribute__ ((long_call, section (".RamFunc")))
static void write_one_page(uint8_t page_number, uint8_t data[FLASH_PAGE_SIZE])
{
    __attribute__((aligned (4))) uint32_t data_u32; // this needs to be aligned on a 32-bit (4 byte) memory location
    uint32_t *src_memory_location = (uint32_t *)data;
    uint32_t *dst_memory_location = (uint32_t *)(page_number * FLASH_PAGE_SIZE + FLASH_BASE_ADDRESS);
    uint32_t i;

    // Then, write the values to FLASH
    for(i = 0; i < FLASH_PAGE_SIZE >> 3; i++) {
        FLASH->CR = FLASH_CR_PG | FLASH_CR_EOPIE;
//        while(FLASH->SR & FLASH_SR_BSY1_Msk);
        memcpy(&data_u32, src_memory_location++, sizeof(uint32_t));
        *(dst_memory_location++) = data_u32;
        memcpy(&data_u32, src_memory_location++, sizeof(uint32_t));
        *(dst_memory_location++) = data_u32;
        while(FLASH->SR & FLASH_SR_BSY1_Msk);
        if((FLASH->SR & FLASH_SR_EOP_Msk) == 0) {
            fatal_error(3); // "flash write fail" (all error text is defined in error_text.c)
        }
        FLASH->SR |= FLASH_SR_EOP_Msk;
    }
    FLASH->CR = 0;
}

__attribute__ ((long_call, section (".RamFunc")))
void save_global_settings(void)
{
    #define SINGLE_WRITE_SIZE (sizeof(uint32_t) * 2)
    uint16_t n_writes = (GLOBAL_SETTINGS_STRUCT_SIZE + (SINGLE_WRITE_SIZE - 1)) / SINGLE_WRITE_SIZE; // divide by the write size but round up
    uint16_t i;
    uint32_t *read_ptr = (void *)&global_settings;
    uint16_t write_index = 0;

    __disable_irq();
    unlock_flash();
    erase_one_page(GLOBAL_SETTINGS_FLASH_PAGE_NUMBER);
    // Then, write the values to FLASH
    for(i = 0; i < n_writes; i++) {
        FLASH->CR = FLASH_CR_PG | FLASH_CR_EOPIE;
        ((uint32_t *)(GLOBAL_SETTINGS_FLASH_ADDRESS))[write_index++] = *(read_ptr++);
        ((uint32_t *)(GLOBAL_SETTINGS_FLASH_ADDRESS))[write_index++] = *(read_ptr++);
        while(FLASH->SR & FLASH_SR_BSY1_Msk);
        if((FLASH->SR & FLASH_SR_EOP_Msk) == 0) {
            fatal_error(3); // "flash write fail" (all error text is defined in error_text.c)
        }
    }
    FLASH->SR |= FLASH_SR_EOP_Msk;
    lock_flash();
    __enable_irq();
}

__attribute__ ((long_call, section (".RamFunc")))
int8_t burn_firmware_page(uint8_t page_number, uint8_t data[FLASH_PAGE_SIZE])
{
    if((page_number < FIRST_FIRMWARE_PAGE_NUMBER) || (page_number > LAST_FIRMWARE_PAGE_NUMBER)) {
        return -1;
    }

    __disable_irq();
    unlock_flash();
    erase_one_page(page_number);
    write_one_page(page_number, data);
    lock_flash();
    __enable_irq();
    return 0;
}