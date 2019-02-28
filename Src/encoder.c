/*
 * encoder.c
 *
 *  Created on: 27 févr. 2019
 *      Author:
 */

#include "encoder.h"


static TIM_HandleTypeDef* htim2;
static TIM_HandleTypeDef* htim3;
static TIM_HandleTypeDef* htim4;
static TIM_HandleTypeDef* htim5;


/*
 *
 */
uint32_t init_encoders(TIM_HandleTypeDef* p_htim2, TIM_HandleTypeDef* p_htim3, TIM_HandleTypeDef* p_htim4, TIM_HandleTypeDef* p_htim5){
	  htim2 = p_htim2;
	  htim3 = p_htim3;
	  htim4 = p_htim4;
	  htim5 = p_htim5;
	  return 0;
}


/*
 * Encoder in 32bits
 */
uint32_t get_back_left_encoder_value(){
	return htim2->Instance->CNT;
}

/*
 * Encoder in 16bits
 * reversed
 */
uint32_t get_back_right_encoder_value(){
	return -(htim3->Instance->CNT);
}

/*
 * Encoder in 16bits
 */
uint32_t get_front_right_encoder_value(){
	return htim4->Instance->CNT;
}

/*
 * Encoder in 32bits
 */
uint32_t get_front_left_encoder_value(){
	return htim5->Instance->CNT;
}
