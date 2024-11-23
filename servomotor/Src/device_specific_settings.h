// This file holds a structure that defines the setting for this device
// These settings are loaded when the device boots up and they are saved to flash memory
// when changed by the applicaiton

#ifndef __DEVICE_SEPECIFIC_SETTINGS_H_
#define __DEVICE_SEPECIFIC_SETTINGS_H_

#include <stdint.h>

struct __attribute__((aligned (4))) __attribute__((__packed__)) global_settings_struct {
    uint8_t my_alias;
    uint16_t hall1_midline;
    uint16_t hall2_midline;
    uint16_t hall3_midline;
    uint32_t commutation_position_offset;
    uint8_t motor_phases_reversed;
    uint8_t dummys[1]; // this solves a warning. the size of this structure should be divisible by 64 bits (8 bytes).
};

#endif