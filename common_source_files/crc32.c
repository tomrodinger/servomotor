/**
 * @file crc32.c
 * @brief CRC32 calculation functions implementation
 */

#include "crc32.h"
#include "settings.h"  // For firmware-related definitions

void crc32_init(void)
{
    RCC->AHBENR |= RCC_AHBENR_CRCEN; // enable the clock to the CRC calculation unit
    CRC->CR = CRC_CR_RESET_Msk | CRC_CR_REV_OUT_Msk | CRC_CR_REV_IN_0 | CRC_CR_REV_IN_1; //reset the CRC hardware, 4 byte mode, reversal
}

uint32_t calculate_crc32(uint32_t new_value)
{
    CRC->DR = new_value;
    return ~CRC->DR;
}

uint32_t calculate_crc32_u8(uint8_t new_value)
{
    ((uint8_t *)&CRC->DR)[0] = new_value;
    return ~CRC->DR;
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

uint32_t calculate_crc32_buffer_without_reinit(uint8_t *buffer, uint32_t len)
{
    uint32_t i;

    for(i = 0; i < len; i++) {
        ((uint8_t *)(&CRC->DR))[0] = *(buffer++);
    }
    return CRC->DR;
}
