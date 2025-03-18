
#ifndef SETTINGS_H_
#define SETTINGS_H_

#include "stm32g0xx_hal.h"
#include "device_specific_settings.h"
#include "crc32.h"  // Include the new CRC32 module

#define FLASH_BASE_ADDRESS 0x8000000
//#define FLASH_PAGE_SIZE 2048  // commented this out because this definition is included in "stm32g0xx_hal.h"
#define BOOTLOADER_N_PAGES 5    // 8 kB bootloader
#define GLOBAL_SETTINGS_FLASH_PAGE_NUMBER 31
#define GLOBAL_SETTINGS_FLASH_ADDRESS (GLOBAL_SETTINGS_FLASH_PAGE_NUMBER * FLASH_PAGE_SIZE + FLASH_BASE_ADDRESS)
#define FIRST_FIRMWARE_PAGE_NUMBER (BOOTLOADER_N_PAGES)
#define LAST_FIRMWARE_PAGE_NUMBER 30
#define FIRMWARE_START_ADDRESS  (FIRST_FIRMWARE_PAGE_NUMBER * FLASH_PAGE_SIZE + FLASH_BASE_ADDRESS)
#define APPLICATION_ADDRESS_PTR (FIRMWARE_START_ADDRESS + sizeof(uint32_t))
#define GLOBAL_SETTINGS_STRUCT_SIZE (sizeof(struct global_settings_struct)) // that device specific structure that defines all the settings for this device is defined in device_specific_settings.h

void load_global_settings(void);
void save_global_settings(void);
int8_t burn_firmware_page(uint8_t page_number, uint8_t data[FLASH_PAGE_SIZE]);
uint8_t firmware_crc_check(void);
uint32_t get_firmware_size(void);


#endif /* SETTINGS_H_ */
