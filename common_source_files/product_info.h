#ifndef PRODUCT_INFO_H_
#define PRODUCT_INFO_H_

#include <stdint.h>

#define MODEL_CODE_LENGTH 8
#define FIRMWARE_COMPATIBILITY_CODE_LENGTH 1

#define PRODUCT_INFO_MEMORY_LOCATION 0x8000010
struct __attribute__((__packed__)) product_info_struct {
    uint8_t model_code[MODEL_CODE_LENGTH];
    uint8_t firmware_compatibility_code;
    uint8_t hardware_version_bugfix;
    uint8_t hardware_version_minor;
    uint8_t hardware_version_major;
    uint32_t serial_number;
    uint64_t unique_id;
    uint32_t not_used;
};

struct product_info_struct *get_product_info(void);

#endif