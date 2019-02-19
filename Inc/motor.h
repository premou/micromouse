/*
 * motor.h
 *
 *  Created on: 29 janv. 2019
 *      Author: Invite
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "stm32f7xx_hal.h"

// GLOBAL VARIABLES

/*
 * The tick to meter factor is computed by using :
 *  - A generic part FACTOR_TICK_2_METER_GEN
 *    built with :
 *     - Pi : 3.1415
 *
 *     - The wheel diameter (m) : 0.026
 *
 *     - A factor that convert the number of 'ticks' provided by the timer
 *       into a number of motor rounds.
 *       Since a motor has two Hall effect sensors, each composed of 6 polar transitions,
 *       therefore creating 6 clock edges per motor round,
 *       we can deduce that this factor equals 2 * 6 = 12.
 *
 *  - A reduction factor FACTOR_TICK_2_METER_<X>, that depends on the motor we use.
 *    We currently use 2 kinds of motors, with the following reductions factors :
 *     - 30
 *     - 50
 */
#define FACTOR_TICK_2_METER_GEN (3.1415 * 0.026 / 12.0)
#define FACTOR_TICK_2_METER_30 (FACTOR_TICK_2_METER_GEN / 30)
#define FACTOR_TICK_2_METER_50 (FACTOR_TICK_2_METER_GEN / 50)
#define FACTOR_TICK_2_METER FACTOR_TICK_2_METER_30

/* slopes for speed (in m/s-2) */
#define SLOPE_ACC 2
#define SLOPE_DEC 2
#define SPEED_TARGET 0.5 //in m/s
#define DIST_START 0.09 //in m
#define DIST_RUN_1 0.18 //in m
#define DIST_STOP 0.09 //in m


// ENUM

typedef  enum {
	LEFT,
	RIGHT
} side_t;

typedef enum {
	ACTION_IDLE,
	ACTION_START, //avance de 8 cm puis RUN_1
	ACTION_RUN_1,
	ACTION_STOP,
	ACTION_CTR
} action_t;


// STRUCTURES DEFINITIONS

typedef struct {
	float speed;
	float distance;

} action_target_t;

typedef struct {
	float dist;
	float speed;

	// TIM front
	// TIM bask

	uint32_t dist_ref_front;
	uint32_t dist_ref_back;

} motors_side_t;

typedef struct  {

	float dist;
	float speed;

	motors_side_t left;
	motors_side_t right;

	uint32_t time;

	// Note that here we store a pointer to the table,
	// rather than directly the table,
	// so that we can clear memory in the end
	action_target_t **p_actions_table;
	// TODO Do we really need this ?
	uint32_t          actions_nb;
	action_target_t  *p_action_curr;

} motors_t;


// GLOBAL VARIABLES

action_target_t actions_targets[] = {
		{0,0},
		{SPEED_TARGET, DIST_START},
		{SPEED_TARGET, DIST_RUN_1},
		{0,DIST_STOP}
};


// FUNCTIONS

motors_t *motors_ctx_init ();

void motors_ctx_init ();

/*
* Update state variables
*/
uint32_t motors_ctx_update (motors_t* p_motors, TIM_HandleTypeDef* htim2,TIM_HandleTypeDef* htim3,TIM_HandleTypeDef* htim4,TIM_HandleTypeDef* htim5);

uint32_t motors_load_actions (motors_t *p_motors, action_target_t **p_actions_targets_table);





#endif /* MOTOR_H_ */
