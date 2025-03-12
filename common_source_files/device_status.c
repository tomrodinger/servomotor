#include "RS485.h"
#include "device_status.h"

static struct device_status_struct device_status = {ENCODED_RESPONSE_CHARACTER, 1, sizeof(struct device_status_struct) - 3, 0, 0};

struct device_status_struct *get_device_status(void)
{
    return &device_status;
}

void set_device_status_flags(uint8_t device_status_flags)
{
    device_status.flags = device_status_flags;
}


void set_device_error_code(uint8_t error_code)
{
    device_status.error_code = error_code;
}
