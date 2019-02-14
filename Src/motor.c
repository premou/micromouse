/*
 * motor.c
 *
 *  Created on: 29 janv. 2019
 *      Author: Invite
 */

#include "motor.h"

// GLOBAL VARIABLES

action_target_t actions_targets[] = {
		{0,0},
		{SPEED_TARGET, DIST_START},
		{SPEED_TARGET, DIST_RUN_1},
		{0,DIST_STOP}
};


uint32_t update_motor_ctx (motor_ctx_t *p_motors, TIM_HandleTypeDef* htim2,TIM_HandleTypeDef* htim3,TIM_HandleTypeDef* htim4,TIM_HandleTypeDef* htim5){

	if (!p_motors){
		return -1;
	}

	//TIM5
	static uint32_t front_left = 0;
	//TIM2
	static uint32_t back_left = 0;
	//TIM4
	static uint16_t front_right = 0;
	//TIM3  reversed
	static uint16_t back_right = 0;


	//TIM5
	static uint32_t last_front_left = 0;
	//TIM2
	static uint32_t last_back_left = 0;
	//TIM4
	static uint16_t last_front_right = 0;
	//TIM3  reversed
	static uint16_t last_back_right = 0;


	//TIM5
	static int32_t diff_front_left = 0;
	//TIM2
	static int32_t diff_back_left = 0;
	//TIM4
	static int16_t diff_front_right = 0;
	//TIM3  reversed
	static int16_t diff_back_right = 0;

	//get current time
	uint32_t current_tick = HAL_GetTick() ;

	//Récupération du delta entre le temps précédent et le courant
	uint32_t delta_time = current_tick - p_motors->time;
	p_motors->time = current_tick;

	//last_front_left = front_left;
	//last_back_left = back_left;
	//last_front_right = front_right;
	//last_back_right = back_right;

	//! TIM2 et TIM5 sur 32 bits, les autres timers sur 16 bits
	dist_ref_back_left = htim2->Instance->CNT;
	dist_ref_back_right = - htim3->Instance->CNT;
	dist_ref_front_right = htim4->Instance->CNT;
	dist_ref_front_left = htim5->Instance->CNT;

	dist_front_left_diff  = dist_ref_front_left  - p_motors->left.dist_ref_front;
	dist_back_left_diff   = dist_ref_back_left   - p_motors->left.dist_ref_back;
	dist_front_right_diff = dist_ref_front_right - p_motors->right.dist_ref_front;
	dist_back_right_diff  = dist_ref_back_right  - p_motors->right.dist_ref_back;

	// Carefull: Here we go from 'tick' unit to 'm' unit
    dist_total_diff += (dist_front_left_diff + dist_back_left_diff + dist_front_right_diff + dist_back_right_diff) / (4.0 * 12.0 * 30.0) * 3.1415 * 0.026;
	dist_right_diff = (dist_front_right_diff + dist_back_right_diff) / (2.0 * 12.0 * 30.0) * 3.1415 * 0.026;
	dist_left_diff = (dist_front_left_diff + dist_back_left_diff) / (2.0 * 12.0 * 30.0) * 3.1415 * 0.026;

	p_motors->dist       += dist_total_diff
	p_motors->right.dist += dist_right_diff
	p_motors->left.dist  += dist_left_diff

	p_motors->speed       = (float) (dist_total_diff * 1000) / delta_time;
	p_motors->right.speed = (float) (dist_right_diff * 1000) / delta_time;
	p_motors->left.speed = (float) (dist_left_diff * 1000) / delta_time;

	return 0;
}
