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
uint32_t init_encoders(TIM_HandleTypeDef* p_htim2, TIM_HandleTypeDef* p_htim3, TIM_HandleTypeDef* p_htim4, TIM_HandleTypeDef* p_htim5);

/*
 * Encoder in 32bits
 */
uint32_t get_back_left_encoder_value();

/*
 * Encoder in 16bits
 * reversed
 */
uint32_t get_back_right_encoder_value();

/*
 * Encoder in 16bits
 */
uint32_t get_front_right_encoder_value();

/*
 * Encoder in 32bits
 */
uint32_t get_front_left_encoder_value();



#endif /* INC_ENCODER_H_ */
