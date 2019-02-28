/*
 * encoder.h
 *
 *  Created on: 27 févr. 2019
 *      Author: premc
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

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
