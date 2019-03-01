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

	float dist_absolute;
	float dist_right_relative;
	float dist_left_relative;

	uint32_t back_left;
	uint16_t back_right;
	uint16_t front_right;
	uint32_t front_left;

} encoder_t;

static encoder_t encoder;


/*
 * Avoir une commande de reset
 * Une commande de la distance totale
 * Une distance droite et gauche depuis le dernier appel (delta)
 *
 *
 * avoir une fonction d'update qui recalcule les delta
 *
 * avoir une fonction pid
 */


/*
 *
 */
void encoder_init(){

	encoder.dist_absolute = 0;

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

/*
 * Return the absolute distance of the 4 encoders
 */
float encoder_absolute(){
	return encoder.dist_absolute;
}


void encoder_update(){

	uint32_t back_left = htim2.Instance->CNT;
	uint16_t back_right = - htim3.Instance->CNT;
	uint16_t front_right = htim4.Instance->CNT;
	uint32_t front_left = htim5.Instance->CNT;

	uint32_t relative32_back_left = back_left  - encoder.back_left;
	uint16_t relative16_back_right = back_right   - encoder.back_right;
	uint16_t relative16_front_right = front_right - encoder.front_right;
	uint32_t relative32_front_left = front_left  - encoder.front_left;

	int32_t relative_left = (int32_t)relative32_back_left + (int32_t) relative32_front_left;
	int32_t relative_right = (int32_t)relative16_back_right + (int32_t) relative16_front_right;

    encoder.dist_absolute += (relative_left + relative_right) * FACTOR_TICK_2_METER / 4.0;
    encoder.dist_left_relative = relative_left * FACTOR_TICK_2_METER / 2.0;
    encoder.dist_right_relative = relative_right * FACTOR_TICK_2_METER / 2.0;

}

/*
 * return m
 */
float encoder_get_absolute(){
	return encoder.dist_absolute;
}

/*
 * return m
 */
float encoder_get_delta_left(){
	return encoder.dist_left_relative;
}

/*
 * return m
 */
float encoder_get_delta_right(){
	return encoder.dist_right_relative;
}


