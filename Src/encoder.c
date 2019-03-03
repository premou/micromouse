/*
 * encoder.c
 *
 *  Created on: 27 févr. 2019
 *      Author:
 */

#include "encoder.h"

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
}

void encoder_update(){

	uint32_t back_left = __HAL_TIM_GetCounter(&htim2);
	uint16_t back_right = - __HAL_TIM_GetCounter(&htim3);
	uint16_t front_right = __HAL_TIM_GetCounter(&htim4);
	uint32_t front_left = __HAL_TIM_GetCounter(&htim5);

	uint32_t relative32_back_left = back_left  - encoder.back_left;
	uint16_t relative16_back_right = back_right   - encoder.back_right;
	uint16_t relative16_front_right = front_right - encoder.front_right;
	uint32_t relative32_front_left = front_left  - encoder.front_left;

	encoder.back_left = back_left;
	encoder.back_right = back_right;
	encoder.front_right = front_right;
	encoder.front_left = front_left;

    encoder.dist_left_relative = (int32_t)relative32_back_left + (int32_t) relative32_front_left;
    encoder.dist_right_relative = (int32_t)relative16_back_right + (int32_t) relative16_front_right;
    encoder.dist_absolute += (encoder.dist_left_relative + encoder.dist_right_relative);

}

/*
 * Return the absolute distance of the 4 encoders
 * in meters
 */
float encoder_get_absolute(){
	return encoder.dist_absolute * FACTOR_TICK_2_METER / 4.0;
}

/*
 * return last call distance left wheels
 * in meters
 */
float encoder_get_delta_left(){
	return encoder.dist_left_relative * FACTOR_TICK_2_METER / 2.0;
}

/*
 * return last call distance right wheels
 * in meters
 */
float encoder_get_delta_right(){
	return encoder.dist_right_relative * FACTOR_TICK_2_METER / 2.0;
}


