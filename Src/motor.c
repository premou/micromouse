/*
 * motor.c
 *
 *  Created on: 29 janv. 2019
 *      Author: Invite
 */

#include "motor.h"

/*
 * !!!!!!! What's the fuck ?
 * why do we define this include only for htim1 ?
 */
extern TIM_HandleTypeDef htim1;


motors_t* motors_ctx_init ()
{
	motors_t *p_motors = (motors_t*) calloc(sizeof(motors_t), 0);

	p_motors->time = HAL_GetTick();

    return p_motors;
}

void motors_ctx_free (motors_t *p_motors)
{
	if (!p_motors) {
		return;
	}

	// TODO Deal later with free of :
	//  - Action structure
	//  - Table of actions structures (if not statically allocated inside the table)
	//
	// Maybe by implementing a free on those kinds of structures.

	free (p_motors);
}

uint32_t motors_ctx_update (motors_t *p_motors, TIM_HandleTypeDef* htim2,TIM_HandleTypeDef* htim3,TIM_HandleTypeDef* htim4,TIM_HandleTypeDef* htim5){

	if (!p_motors){
		return -1;
	}

	uint32_t dist_ref_front_left = 0;
	uint32_t dist_ref_back_left = 0;
	uint32_t dist_ref_front_right = 0;
	uint32_t dist_ref_back_right = 0;

	int32_t dist_ref_front_left_diff = 0;
	int32_t dist_ref_back_left_diff = 0;
	int32_t dist_ref_front_right_diff = 0;
	int32_t dist_ref_back_right_diff = 0;

	int32_t dist_total_diff = 0;
	int32_t dist_right_diff = 0;
	int32_t dist_left_diff = 0;

	//get current time
	uint32_t current_tick = HAL_GetTick() ;

	//Récupération du delta entre le temps précédent et le courant
	uint32_t delta_time = current_tick - p_motors->time;
	p_motors->time = current_tick;

	// ! TIM2 et TIM5 sur 32 bits, les autres timers sur 16 bits
	// ! TIM3 is coutning in reverse order, that's why we use a "-" in the expression below
	dist_ref_back_left   = htim2->Instance->CNT;
	dist_ref_back_right  = - (uint32_t)htim3->Instance->CNT;
	dist_ref_front_right = (uint32_t)htim4->Instance->CNT;
	dist_ref_front_left  = htim5->Instance->CNT;

	dist_ref_front_left_diff  = dist_ref_front_left  - p_motors->left.dist_ref_front;
	dist_ref_back_left_diff   = dist_ref_back_left   - p_motors->left.dist_ref_back;
	dist_ref_front_right_diff = dist_ref_front_right - p_motors->right.dist_ref_front;
	dist_ref_back_right_diff  = dist_ref_back_right  - p_motors->right.dist_ref_back;

	// Carefull: Here we go from 'tick' unit to 'm' unit
    dist_total_diff += (dist_ref_front_left_diff + dist_ref_back_left_diff + dist_ref_front_right_diff + dist_ref_back_right_diff) * FACTOR_TICK_2_METER / 4.0;
	dist_right_diff = (dist_ref_front_right_diff + dist_ref_back_right_diff) * FACTOR_TICK_2_METER / 2.0;
	dist_left_diff = (dist_ref_front_left_diff + dist_ref_back_left_diff) * FACTOR_TICK_2_METER / 2.0;

	p_motors->dist       += dist_total_diff;
	p_motors->right.dist += dist_right_diff;
	p_motors->left.dist  += dist_left_diff;

	p_motors->speed       = (float) (dist_total_diff * 1000) / delta_time;
	p_motors->right.speed = (float) (dist_right_diff * 1000) / delta_time;
	p_motors->left.speed = (float) (dist_left_diff * 1000) / delta_time;

	p_motors->left.dist_ref_front = dist_ref_front_left;
	p_motors->left.dist_ref_back = dist_ref_back_left;
	p_motors->right.dist_ref_front = dist_ref_front_right;
	p_motors->right.dist_ref_back = dist_ref_back_right;

	return 0;
}

uint32_t motors_load_actions (motors_t *p_motors, action_target_t **p_actions_targets_table) {

	if (!p_motors || !p_actions_targets_table) {
		return 1;
	}

	if (p_motors->p_actions_table) {
		// Preventing to load new actions if there are still pending ones.
		// TODO decide what to do in this case :
		//  - Remove older ones ?
		//  - Append new ones at the end ?
		return 2;
	}

	p_motors->p_actions_table = p_actions_targets_table;
	p_motors->p_action_curr = *p_actions_targets_table;
}
