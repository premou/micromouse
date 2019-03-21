/*
 * timer_us.c
 *
 *  Created on: 21 mars 2019
 *      Author: Invite
 */

#include "timer_us.h"

// use TIME10 ² on APB2 (216MHz) ==> 1MHz clock counter
TIM_HandleTypeDef htim10;

// initialise microsecond timer
void timer_us_init()
{
	HAL_TIM_Base_Start(&htim10);
}

// get time in microsecond from boot
uint16_t timer_us_get()
{
	return __HAL_TIM_GET_COUNTER(&htim10);
}

// wait for a few microseconds
void timer_us_delay(uint16_t delay_us)
{
	uint16_t start_time = __HAL_TIM_GET_COUNTER(&htim10);
	while((int16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim10)-start_time)<delay_us);
}
