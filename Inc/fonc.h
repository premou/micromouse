/*
 * fonc.h
 *
 *  Created on: 8 nov. 2018
 *      Author: Invite
 */

#ifndef FONC_H_
#define FONC_H_

#include "stm32f7xx_hal.h"

/*
void marche_avant(uint32_t speed);
void marche_arriere(uint32_t speed);
void arret();
*/
void run(int32_t speed_right, int32_t speed_left);


void init_avance();
uint32_t get_encoder_raw_value(TIM_HandleTypeDef* htim2,TIM_HandleTypeDef* htim3,TIM_HandleTypeDef* htim4,TIM_HandleTypeDef* htim5);
uint32_t avance(float t, float * vitesse);
#endif /* FONC_H_ */
