/*
 * encoder.h
 *
 *  Created on: 27 févr. 2019
 *      Author: premc
 */


#include "stm32f7xx_hal.h"

#define FACTOR_TICK_2_METER_GEN (3.1415 * 0.026 / 12.0)
#define FACTOR_TICK_2_METER_30 (FACTOR_TICK_2_METER_GEN / 30)
#define FACTOR_TICK_2_METER_50 (FACTOR_TICK_2_METER_GEN / 50)
#define FACTOR_TICK_2_METER FACTOR_TICK_2_METER_30


#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

/*
 *
 */
void encoder_init();


void encoder_reset();
float encoder_get_absolute();
float encoder_get_delta_left();
float encoder_get_delta_right();

#endif /* INC_ENCODER_H_ */
