#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>

#define N_RECEIVE_BUFFERS 1
// Be able to handle the command with the biggest input, which is a firmware upgrade packet. Currently the size is 8 + 1 + 2048 + 1 = 2058 bytes
#define MAX_VALUE_BUFFER_LENGTH (MODEL_CODE_LENGTH + FIRMWARE_COMPATIBILITY_CODE_LENGTH + FLASH_PAGE_SIZE + 1)

#endif