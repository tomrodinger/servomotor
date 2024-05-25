/*
 * step_direction_input.h
 *
 *  Created on: 4 Sep 2021
 *      Author: tom
 */

#ifndef SRC_STEP_DIRECTION_INPUT_H_
#define SRC_STEP_DIRECTION_INPUT_H_

#define OVERVOLTAGE_PROTECTION_SETTING 26

void overvoltage_init(void);
void step_and_direction_init(void);
int32_t get_external_encoder_position(void);
void print_external_encoder_position(void);

#endif /* SRC_STEP_DIRECTION_INPUT_H_ */
