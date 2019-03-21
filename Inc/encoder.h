/*
 * encoder.h
 *
 *  Created on: 27 févr. 2019
 *      Author: premc
 */


#include "stm32f7xx_hal.h"

/*
 * The tick to meter factor is computed by using :
 *  - A generic part FACTOR_TICK_2_METER_GEN
 *    built with :
 *     - Pi : 3.1415
 *
 *     - The wheel diameter (m) : 0.026
 *
 *     - A factor that convert the number of 'ticks' provided by the timer
 *       into a number of motor rounds.
 *       Since a motor has two Hall effect sensors, each composed of 6 polar transitions,
 *       therefore creating 6 clock edges per motor round,
 *       we can deduce that this factor equals 2 * 6 = 12.
 *
 *  - A reduction factor FACTOR_TICK_2_METER_<X>, that depends on the motor we use.
 *    We currently use 2 kinds of motors, with the following reductions factors :
 *     - 30
 *     - 50
 */

#define PI_FLOAT     3.14159265f

#define FACTOR_TICK_2_METER_GEN (PI_FLOAT * 0.026 * 0.995 / 12.0)
#define FACTOR_TICK_2_METER_30 (FACTOR_TICK_2_METER_GEN / 30.0)
#define FACTOR_TICK_2_METER_50 (FACTOR_TICK_2_METER_GEN / 50.0)
#define FACTOR_TICK_2_METER FACTOR_TICK_2_METER_50

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
