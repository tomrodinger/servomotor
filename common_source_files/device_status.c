#include "RS485.h"
#include "device_status.h"

static struct device_status_struct device_status = {0, 0};

struct device_status_struct *get_device_status(void)
{
    return &device_status;
}

void set_device_status_flags(uint16_t device_status_flags)
{
    device_status.flags = device_status_flags;
}


void set_device_error_code(uint8_t error_code)
{
    device_status.error_code = error_code;
}

#ifdef MOTOR_SIMULATION
void device_status_simulator_init(void)
{
    device_status.flags = 0;
    device_status.error_code = 0;
}
#endif