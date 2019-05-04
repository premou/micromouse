/*
 * encoder.h
 *
 *  Created on: 27 févr. 2019
 *      Author: premc
 */


#include "stm32f7xx_hal.h"

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

/*
 *
 */
void encoder_init();
void encoder_reset();
void encoder_update();

float encoder_get_absolute();
void encoder_set_absolute(float dist);
float encoder_get_delta_left();
float encoder_get_delta_right();

#endif /* INC_ENCODER_H_ */
