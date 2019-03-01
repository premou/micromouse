/*
 * controller.c
 *
 *  Created on: 1 mars 2019
 *      Author: Invite
 */

#include "controller.h"


/*
 * !!!!!!! What's the ?
 * why do we define this include only for htim1 ?
 */
extern TIM_HandleTypeDef htim1;


/* //TODO
 * Definir une fonction qui init les encoders
 * Definir une fonction qui applique le PWM (droite et gauche)
 * Definir plusieurs fonctions qui détermine la vitesse suivante en fonction de l'état (calcule avec les pentes acc et dec)
 */

controller_t* controller_ctx_init ()
{
	controller_t *p_motors = (controller_t*) calloc(sizeof(controller_t), 0);

	p_motors->time = HAL_GetTick();

    return p_motors;
}

void controller_ctx_free (controller_t *p_motors)
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

uint32_t controller_ctx_update (controller_t *p_motors, TIM_HandleTypeDef* htim2,TIM_HandleTypeDef* htim3,TIM_HandleTypeDef* htim4,TIM_HandleTypeDef* htim5){

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
  //  dist_total_diff += (dist_ref_front_left_diff + dist_ref_back_left_diff + dist_ref_front_right_diff + dist_ref_back_right_diff) * FACTOR_TICK_2_METER / 4.0;
//	dist_right_diff = (dist_ref_front_right_diff + dist_ref_back_right_diff) * FACTOR_TICK_2_METER / 2.0;
//	dist_left_diff = (dist_ref_front_left_diff + dist_ref_back_left_diff) * FACTOR_TICK_2_METER / 2.0;

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
/*
 * return values :
 * 0 : ok, table loaded
 * 1 : ko, empty input(s)
 * 2 : ...
 */
uint32_t controller_load_actions (controller_t *p_motors, action_target_t **p_actions_targets_table) {

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
	return 0;
}


/*
 * return values :
 * 1 : error : could not get current action from p_motors
 *
 *
 */
action_t controller_find_state(controller_t *p_motors)
{
	action_t current_state;
	//get current state from speed and distance of current action
		if(p_motors->p_action_curr->speed == 0)
		{
			current_state=ACTION_STOP;
		}
		else if(p_motors->p_action_curr->speed == SPEED_TARGET)
		{
			if(p_motors->p_action_curr->distance == DIST_START)
			{
				current_state=ACTION_START;
			}
			else if(p_motors->p_action_curr->distance == DIST_RUN_1)
			{
				current_state=ACTION_RUN_1;
			}
			else
			{
				return ACTION_IDLE;
			}
		}
		else
		{
			return ACTION_IDLE;
		}
	return current_state;
}

int controller_fsm(controller_t *p_motors)
{
	action_t current_state = controller_find_state(p_motors);

	switch(current_state)
	{
	case ACTION_IDLE :
	{
		motor_speed_left(0);
		motor_speed_right(0);
		//do nothing ?
	}
	break;
	case ACTION_START :
	{
		//
		p_motors->speed = next_speed(p_motors->p_action_curr->speed, SLOPE_ACC, SLOPE_DEC, 0.001, p_motors->speed);
		encoder_update();
		float real_speed = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2) / 0.001;
		float pwm = pid_output(p_motors->speed_pid, real_speed - p_motors->speed);

		motor_speed_left(pwm);
		motor_speed_right(pwm);

		p_motors->dist = encoder_get_absolute();

		//if dist_remontée > dist consigne --> transition
		//current speed :
		//p_motors->speed
		if(p_motors->dist > p_motors->p_action_curr->distance)
		{
			//current_state=ACTION_IDLE;//transition
			//comment passer a l'action suivante ?
			//p_motors->p_action_curr = (p_motors->p_action_curr)+1; //go to next action
			current_state = controller_find_state(p_motors);
		}
	}
	break;
	case ACTION_RUN_1 :
	{
		//
	}
	break;
	case ACTION_STOP :
	{
		//
	}
	break;
	case ACTION_CTR :
	{
		//
	}
	break;
	}
}
