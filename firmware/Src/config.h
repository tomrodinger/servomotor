#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>

#define MAX_MULTI_MOVES (sizeof(uint32_t) * 8) // maximum number of moves in a multi move command. this number should be the number of bits in an uint32_t
#define N_RECEIVE_BUFFERS 2
// Be able to handle the command with the biggest input, which is the multimove command. Currently the size is 1 + 4 + 4 * 2 * 32 = 261 bytes
#define MAX_VALUE_BUFFER_LENGTH (sizeof(uint8_t) + sizeof(int32_t) + sizeof(int32_t) * 2 * MAX_MULTI_MOVES)

#endif