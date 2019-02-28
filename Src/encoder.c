/*
 * encoder.c
 *
 *  Created on: 27 févr. 2019
 *      Author:
 */


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
