/*
 * unique_id.h
 *
 *  Created on: 11 Sep 2021
 *      Author: tom
 */

#ifndef INC_UNIQUE_ID_H_
#define INC_UNIQUE_ID_H_

#include <stdint.h>

// Size of unique ID in bytes
#define UNIQUE_ID_SIZE sizeof(uint64_t)

uint64_t get_unique_id(void);
uint32_t get_random_number(uint32_t limit);


#endif /* INC_UNIQUE_ID_H_ */
