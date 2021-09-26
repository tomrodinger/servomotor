#include "stm32g0xx_hal.h"
#include "settings.h"
#include "error_handling.h"

void load_settings(uint8_t *my_alias)
{
    *my_alias = *((uint8_t *)(SETTINGS_FLASH_ADDRESS));
}

static void unlock_flash(void)
{
    FLASH->KEYR = 0x45670123; // unlock sequence
    FLASH->KEYR = 0xCDEF89AB; // unlock sequence
    while(FLASH->SR & FLASH_SR_BSY1_Msk);

    if(FLASH->CR & FLASH_CR_LOCK_Msk) {
        fatal_error("flash unlock fail", 3);
    }
}

static inline void lock_flash(void)
{
    FLASH->CR = FLASH_CR_LOCK;
}

static void erase_one_page(uint8_t page_number)
{
    // First, erase the page
    FLASH->CR = FLASH_CR_PER | (page_number << FLASH_CR_PNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    while(FLASH->SR & FLASH_SR_BSY1_Msk);
}

static void write_one_page(uint8_t page_number, uint8_t data[FLASH_PAGE_SIZE])
{
    uint32_t *src_memory_location = (uint32_t *)data;
    uint32_t *dst_memory_location = (uint32_t *)(page_number * FLASH_PAGE_SIZE + FLASH_BASE_ADDRESS);
    uint32_t i;

    // Then, write the values to FLASH
    FLASH->CR = FLASH_CR_PG | FLASH_CR_EOPIE;
    for(i = 0; i < FLASH_PAGE_SIZE >> 3; i++) {
        *(dst_memory_location++) = *(src_memory_location++);
        *(dst_memory_location++) = *(src_memory_location++);
        while(FLASH->SR & FLASH_SR_BSY1_Msk);
        if((FLASH->SR & FLASH_SR_EOP_Msk) == 0) {
            fatal_error("flash write fail", 4);
        }
    }
    FLASH->SR |= FLASH_SR_EOP_Msk;
}

void save_settings(uint8_t my_alias)
{
    unlock_flash();
    erase_one_page(FLASH_SETTINGS_PAGE_NUMBER);

    // Then, write the values to FLASH
    FLASH->CR = FLASH_CR_PG | FLASH_CR_EOPIE;
    ((uint32_t *)(SETTINGS_FLASH_ADDRESS))[0] = my_alias;
    ((uint32_t *)(SETTINGS_FLASH_ADDRESS))[1] = 0xbbbbbbbb;
    while(FLASH->SR & FLASH_SR_BSY1_Msk);
    if((FLASH->SR & FLASH_SR_EOP_Msk) == 0) {
        fatal_error("flash write fail", 4);
    }
    FLASH->SR |= FLASH_SR_EOP_Msk;

    lock_flash();
}

int8_t burn_firmware_page(uint8_t page_number, uint8_t data[FLASH_PAGE_SIZE])
{
    if((page_number < FIRST_FIRMWARE_PAGE_NUMBER) || (page_number > LAST_FIRMWARE_PAGE_NUMBER)) {
        return -1;
    }

    unlock_flash();
    erase_one_page(page_number);
    write_one_page(page_number, data);
    lock_flash();
    return 0;
}