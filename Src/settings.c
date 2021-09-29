#include <string.h>
#include "stm32g0xx_hal.h"
#include "settings.h"
#include "error_handling.h"

void load_settings(uint8_t *my_alias)
{
    *my_alias = *((uint8_t *)(SETTINGS_FLASH_ADDRESS));
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
        fatal_error("flash unlock fail", 3);
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
            fatal_error("flash write fail", 4);
        }
        FLASH->SR |= FLASH_SR_EOP_Msk;
    }
    FLASH->CR = 0;
}

__attribute__ ((long_call, section (".RamFunc")))
void save_settings(uint8_t my_alias)
{
    __attribute__((aligned (4))) uint32_t data_u32 = my_alias; // this needs to be aligned on a 32-bit (4 byte) memory location

    __disable_irq();
    unlock_flash();
    erase_one_page(FLASH_SETTINGS_PAGE_NUMBER);
    // Then, write the values to FLASH
    FLASH->CR = FLASH_CR_PG | FLASH_CR_EOPIE;
    ((uint32_t *)(SETTINGS_FLASH_ADDRESS))[0] = data_u32;
    ((uint32_t *)(SETTINGS_FLASH_ADDRESS))[1] = 0xbbbbbbbb;
    while(FLASH->SR & FLASH_SR_BSY1_Msk);
    if((FLASH->SR & FLASH_SR_EOP_Msk) == 0) {
        fatal_error("flash write fail", 4);
    }
    FLASH->SR |= FLASH_SR_EOP_Msk;
    lock_flash();
    __enable_irq();
}

__attribute__ ((long_call, section (".RamFunc")))
int8_t burn_firmware_page(uint8_t page_number, uint8_t *data)
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