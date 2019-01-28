/*
 * tone.h
 *
 *  Created on: 28 janv. 2019
 *      Author: Prem
 */

#ifndef TONE_H_
#define TONE_H_

#include "stm32f7xx_hal.h"

void tone(TIM_HandleTypeDef* htim,uint32_t frequence, uint32_t delay, uint32_t channel);
void play_startup_song(TIM_HandleTypeDef* htim, uint32_t channel);
void play_startup_song2(TIM_HandleTypeDef* htim, uint32_t channel);

#endif /* TONE_H_ */
