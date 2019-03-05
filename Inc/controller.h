/*
 * controller.h
 *
 *  Created on: 1 mars 2019
 *      Author: Invite
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "pid.h"
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

} controller_side_t;

typedef struct  {

	float dist;  //current distance
	float speed; //current speed

	controller_side_t left;
	controller_side_t right;

	uint32_t time;

	// Note that here we store a pointer to the table,
	// rather than directly the table,
	// so that we can clear memory in the end
	action_target_t **p_actions_table;
	// TODO Do we really need this ?
	uint32_t          actions_nb;
	action_target_t  *p_action_curr;

	pid_context_t* speed_pid;
} controller_t;


// GLOBAL VARIABLES

action_target_t actions_targets[] = {
		{0,0},
		{SPEED_TARGET, DIST_START},
		{SPEED_TARGET, DIST_RUN_1},
		{0,DIST_STOP}
};


// FUNCTIONS

controller_t *controller_ctx_init ();


/*
* Update state variables
*/
uint32_t controller_ctx_update (controller_t* p_motors, TIM_HandleTypeDef* htim2,TIM_HandleTypeDef* htim3,TIM_HandleTypeDef* htim4,TIM_HandleTypeDef* htim5);

uint32_t controller_load_actions (controller_t *p_motors, action_target_t **p_actions_targets_table);


action_t controller_find_state(controller_t *p_motors);
int controller_fsm(controller_t *p_motors);




#endif /* CONTROLLER_H_ */
