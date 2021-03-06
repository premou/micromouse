/*
 * encoder.c
 *
 *  Created on: 27 f�vr. 2019
 *      Author:
 */

#include "encoder.h"
#include "personnalisation.h"

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
#define FACTOR_TICK_2_METER (PI_FLOAT * WHEEL_DIAMETER * GEAR_BOX_RATIO / 12.0)


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

typedef struct {

	int32_t dist_absolute;
	int32_t dist_right_relative;
	int32_t dist_left_relative;

	uint32_t back_left;
	uint16_t back_right;
	uint16_t front_right;
	uint32_t front_left;

} encoder_t;

static encoder_t encoder;

/*
 *
 */
void encoder_init(){

	encoder.dist_absolute = 0;
	encoder.dist_left_relative = 0;
	encoder.dist_right_relative = 0;

	encoder.back_left = 0;
	encoder.back_right = 0;
	encoder.front_right = 0;
	encoder.front_left = 0;

	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);

}

/*
 * Reset the absolute distance
 */
void encoder_reset(){
	encoder.dist_absolute = 0;
	encoder.dist_left_relative = 0;
	encoder.dist_right_relative = 0;

	encoder.back_left = 0;
	encoder.back_right = 0;
	encoder.front_right = 0;
	encoder.front_left = 0;

	__HAL_TIM_SetCounter(&htim2,0U);
	__HAL_TIM_SetCounter(&htim3,0U);
	__HAL_TIM_SetCounter(&htim4,0U);
	__HAL_TIM_SetCounter(&htim5,0U);

}

void encoder_update(){

	uint32_t back_left = __HAL_TIM_GetCounter(&htim2);
	uint16_t back_right = - __HAL_TIM_GetCounter(&htim3);
	uint16_t front_right = __HAL_TIM_GetCounter(&htim4);
	uint32_t front_left = __HAL_TIM_GetCounter(&htim5);

	int32_t relative32_back_left = back_left  - encoder.back_left;
	int16_t relative16_back_right = back_right   - encoder.back_right;
	int16_t relative16_front_right = front_right - encoder.front_right;
	int32_t relative32_front_left = front_left  - encoder.front_left;

	encoder.back_left = back_left;
	encoder.back_right = back_right;
	encoder.front_right = front_right;
	encoder.front_left = front_left;

    encoder.dist_left_relative = (int32_t)relative32_back_left + (int32_t)relative32_front_left;
    encoder.dist_right_relative = (int32_t)relative16_back_right + (int32_t)relative16_front_right;
    encoder.dist_absolute += (encoder.dist_left_relative + encoder.dist_right_relative);

}

/*
 * Return the absolute distance of the 4 encoders
 * in meters
 */
float encoder_get_absolute(){
	return (float)(encoder.dist_absolute) * FACTOR_TICK_2_METER / 4.0;
}

/*
 * Set the absolute distance of the 4 encoders in ticks
 * dist : in m
 */
//dist = distance parcourue en plus que la distance cible
void encoder_set_absolute(float dist){
	encoder.dist_absolute = (int32_t) (dist * 4.0 / FACTOR_TICK_2_METER);
}

/*
 * return last call distance left wheels
 * in meters
 */
float encoder_get_delta_left(){
	return (float)(encoder.dist_left_relative) * FACTOR_TICK_2_METER / 2.0;
}

/*
 * return last call distance right wheels
 * in meters
 */
float encoder_get_delta_right(){
	return (float)(encoder.dist_right_relative) * FACTOR_TICK_2_METER / 2.0;
}


