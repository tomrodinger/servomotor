
#ifndef SETTINGS_H_
#define SETTINGS_H_

#define FLASH_BASE_ADDRESS 0x8000000
//#define FLASH_PAGE_SIZE 2048  // commented this out because this definition is included in "stm32g0xx_hal.h"
#define BOOTLOADER_N_PAGES 5    // 8 kB bootloader
#define FLASH_SETTINGS_PAGE_NUMBER 31
#define SETTINGS_FLASH_ADDRESS (FLASH_SETTINGS_PAGE_NUMBER * FLASH_PAGE_SIZE + FLASH_BASE_ADDRESS)
#define FIRST_FIRMWARE_PAGE_NUMBER (BOOTLOADER_N_PAGES)
#define LAST_FIRMWARE_PAGE_NUMBER 30
#define FIRMWARE_START_ADDRESS  (FIRST_FIRMWARE_PAGE_NUMBER * FLASH_PAGE_SIZE + FLASH_BASE_ADDRESS)
#define APPLICATION_ADDRESS_PTR (FIRMWARE_START_ADDRESS + sizeof(uint32_t))

void load_settings(uint8_t *my_alias, uint16_t *hall1_midline, uint16_t *hall2_midline, uint16_t *hall3_midline);
void save_settings(uint8_t my_alias, uint16_t hall1_midline, uint16_t hall2_midline, uint16_t hall3_midline);
int8_t burn_firmware_page(uint8_t page_number, uint8_t data[FLASH_PAGE_SIZE]);
void crc32_init(void);
uint32_t crc32(uint32_t new_value);
uint8_t firmware_crc_check(void);
uint32_t calculate_crc32_buffer(uint8_t *buffer, uint32_t len);
uint32_t get_firmware_size(void);


#endif /* SETTINGS_H_ */
