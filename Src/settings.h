
#ifndef SETTINGS_H_
#define SETTINGS_H_

#define FLASH_BASE_ADDRESS 0x8000000
//#define FLASH_PAGE_SIZE 2048  // commented this out because this definition is included in "stm32g0xx_hal.h"
#define FLASH_SETTINGS_PAGE_NUMBER 15
#define SETTINGS_FLASH_ADDRESS (FLASH_SETTINGS_PAGE_NUMBER * FLASH_PAGE_SIZE + FLASH_BASE_ADDRESS)
#define FIRST_FIRMWARE_PAGE_NUMBER (FLASH_SETTINGS_PAGE_NUMBER + 1)
#define LAST_FIRMWARE_PAGE_NUMBER 31

void load_settings(uint8_t *my_alias);
void save_settings(uint8_t my_alias);
int8_t burn_firmware_page(uint8_t page_number, uint8_t data[FLASH_PAGE_SIZE]);


#endif /* SETTINGS_H_ */
