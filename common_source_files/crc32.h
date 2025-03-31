/**
 * @file crc32.h
 * @brief CRC32 calculation functions
 */

#ifndef CRC32_H_
#define CRC32_H_

#include "stm32g0xx_hal.h"

/**
 * @brief Initialize the CRC32 hardware unit
 */
void crc32_init(void);

/**
 * @brief Calculate CRC32 for a 32-bit value
 * @param new_value The 32-bit value to calculate CRC32 for
 * @return The calculated CRC32 value
 */
uint32_t calculate_crc32(uint32_t new_value);

/**
 * @brief Calculate CRC32 for an 8-bit value
 * @param new_value The 8-bit value to calculate CRC32 for
 * @return The calculated CRC32 value
 */
uint32_t calculate_crc32_u8(uint8_t new_value);

/**
 * @brief Calculate CRC32 for a buffer of data
 * @param buffer Pointer to the data buffer
 * @param len Length of the data buffer in bytes
 * @return The calculated CRC32 value
 */
uint32_t calculate_crc32_buffer(void *buffer, uint32_t len);

/**
 * @brief Calculate CRC32 for a buffer of data without initializing the CRC32 hardware unit
 * @brief This is useful if you have several buffers to add to the CRC32 calculation
 * @param buffer Pointer to the data buffer
 * @param len Length of the data buffer in bytes
 * @return The calculated CRC32 value
 */
uint32_t calculate_crc32_buffer_without_reinit(void *buffer, uint32_t len);

/**
 * @brief Get the current CRC32 value
 * @return The current CRC32 value
 */
uint32_t get_crc32(void);

#endif /* CRC32_H_ */
