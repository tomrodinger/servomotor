
#ifndef SETTINGS_H_
#define SETTINGS_H_

#include "stm32g0xx_hal.h"

#define FLASH_BASE_ADDRESS 0x8000000
//#define FLASH_PAGE_SIZE 2048  // commented this out because this definition is included in "stm32g0xx_hal.h"
#define BOOTLOADER_N_PAGES 5    // 8 kB bootloader
#define GLOBAL_SETTINGS_FLASH_PAGE_NUMBER 31
#define GLOBAL_SETTINGS_FLASH_ADDRESS (GLOBAL_SETTINGS_FLASH_PAGE_NUMBER * FLASH_PAGE_SIZE + FLASH_BASE_ADDRESS)
#define FIRST_FIRMWARE_PAGE_NUMBER (BOOTLOADER_N_PAGES)
#define LAST_FIRMWARE_PAGE_NUMBER 30
#define FIRMWARE_START_ADDRESS  (FIRST_FIRMWARE_PAGE_NUMBER * FLASH_PAGE_SIZE + FLASH_BASE_ADDRESS)
#define APPLICATION_ADDRESS_PTR (FIRMWARE_START_ADDRESS + sizeof(uint32_t))

#define GLOBAL_SETTINGS_STRUCT_SIZE (sizeof(struct global_settings_struct)) 
struct __attribute__((aligned (4))) __attribute__((__packed__)) global_settings_struct {
	uint8_t my_alias;
	uint16_t hall1_midline;
	uint16_t hall2_midline;
	uint16_t hall3_midline;
    uint16_t max_motor_pwm_voltage;
    uint16_t max_motor_regen_pwm_voltage;
    uint8_t dummys[5]; // this solves a warning. the size of this structure should be divisible by 64 bits (8 bytes).
};

void load_global_settings(void);
void save_global_settings(void);
int8_t burn_firmware_page(uint8_t page_number, uint8_t data[FLASH_PAGE_SIZE]);
void crc32_init(void);
uint32_t crc32(uint32_t new_value);
uint8_t firmware_crc_check(void);
uint32_t calculate_crc32_buffer(uint8_t *buffer, uint32_t len);
uint32_t get_firmware_size(void);


#endif /* SETTINGS_H_ */
