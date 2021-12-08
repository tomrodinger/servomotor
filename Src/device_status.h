#ifndef DEVICE_STATUS_H_
#define DEVICE_STATUS_H_

#include <stdint.h>

// The flags variable can contain up to 8 flags. These are defined so far:
// Bit 0: In the bootloader (if this flag is set then the other flags below will all be 0)
// Bit 1: MOSFETs are enabled
// Bit 2: Motor is in closed loop mode
// Bit 3: Motor is currently executing the calibration command
// Bit 4: Motor is currently executing a homing command
// Bit 5: Not used, set to 0
// Bit 6: Not used, set to 0
// Bit 7: Not used, set to 0

#define STATUS_IN_THE_BOOTLOADER_FLAG_BIT 0
#define STATUS_MOSFETS_ENABLED_FLAG_BIT 1
#define STATUS_CLOSED_LOOP_FLAG_BIT 2
#define STATUS_CALIBRATING_FLAG_BIT 3
#define STATUS_HOMING_FLAG_BIT 4


struct __attribute__((__packed__)) device_status_struct {
    uint8_t alias;
    uint8_t command;
    uint8_t parameter_length;
	uint8_t flags;
	uint8_t error_code;
};

struct device_status_struct *get_device_status(void);
void set_device_status_flags(uint8_t device_status_flags);
void set_device_error_code(uint8_t error_code);

#endif
