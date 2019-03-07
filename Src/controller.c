/*
 * controller.c
 *
 *  Created on: 1 mars 2019
 *      Author: Invite
 */
#include "controller.h"
#include "serial.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "datalogger.h"
#include "math.h"

extern HAL_Serial_Handler com;

// CONSTANT

/* slopes for speed (in m/s-2) */
#define SLOPE_ACC 2
#define SLOPE_DEC 2
#define SPEED_TARGET 0.5 //in m/s
#define DIST_START 0.09 //in m
#define DIST_RUN_1 0.18 //in m
#define DIST_STOP 0.09 //in m

#define SPEED_KP 5.0
#define SPEED_KI 1.0
#define SPEED_KD 1.0

// ENUM

typedef enum {
	ACTION_IDLE,
	ACTION_START,
	ACTION_RUN_1,
	ACTION_STOP,
	ACTION_CTR
} action_t;

// STRUCTURES DEFINITIONS

typedef struct  {

	uint32_t actions_nb; // index of current action in the scenario array
	uint32_t time;

	int32_t pwm;


	float speed_target;
	float speed_setpoint;
	float speed_current;
	float speed_error;
	float speed_pwm;



/*
	float dist;  //current distance
	float speed; //current speed



	controller_side_t left;
	controller_side_t right;



	// Note that here we store a pointer to the table,
	// rather than directly the table,
	// so that we can clear memory in the end
	action_target_t **p_actions_table;

	action_target_t  *p_action_curr;

	*/
	pid_context_t speed_pid;
} controller_t;

// GLOBAL VARIABLES

static action_t actions_scenario[] = {
	ACTION_START,
	ACTION_RUN_1,
	ACTION_RUN_1,
	ACTION_STOP,
	ACTION_IDLE
  };

typedef struct {
	float dist;
	float speed;
} action_values_t;

static controller_t ctx;

// PUBLIC FUNCTIONS

void controller_init ()
{
	/*revenir au d�but de la liste des actions*/
	ctx.actions_nb = 0;
	ctx.time = 0;

	ctx.speed_target = 0;
	ctx.speed_current = 0;
	ctx.speed_error = 0;
	ctx.speed_setpoint = 0;
	ctx.speed_pwm = 0;

	motor_init();
	encoder_init();

	pid_init(&ctx.speed_pid, SPEED_KP, SPEED_KI, SPEED_KD);

	HAL_DataLogger_Init(5,
			4,
			4,
			4,
			4,
			1); // TODO : to be completed with each recorded field size
}

void controller_start(){
	/*revenir au d�but de la liste des actions*/
	ctx.actions_nb = 0;
	ctx.time = HAL_GetTick();

	ctx.speed_target = 0;
	ctx.speed_current = 0;
	ctx.speed_error = 0;
	ctx.speed_setpoint = 0;
	ctx.speed_pwm = 0;

	pid_reset(&ctx.speed_pid);
	encoder_reset();
	HAL_DataLogger_Clear();
}

void controller_fsm(); // forward declaration

void controller_update(){
	//cadence a 1ms
	uint32_t time_temp = HAL_GetTick();
	if(time_temp > ctx.time)
	{
		ctx.time = time_temp;
		//HAL_Serial_Print(&com,"current time is %d, cuurent action is %d\r\n", time_temp, ctx.actions_nb);
		encoder_update();
		controller_fsm();
		HAL_DataLogger_Record(5,
				(int32_t)(ctx.speed_target * 1000.0),
				(int32_t)(ctx.speed_setpoint * 1000.0),
				(int32_t)(ctx.speed_current * 1000.0),
				(int32_t)(ctx.speed_error * 1000.0),
				(int32_t)ctx.speed_pwm);
	}

}

bool controller_is_end(){
	return actions_scenario[ctx.actions_nb] == ACTION_IDLE;
}

// PRIVATE FUNCTIONS

void controller_fsm()
{
	switch(actions_scenario[ctx.actions_nb])
	{
	case ACTION_IDLE :
	{
		motor_speed_left(0);
		motor_speed_right(0);
	}
	break;

	case ACTION_START :
	{

//		ctx.pwm = 10;
		ctx.speed_target = SPEED_TARGET;
		ctx.speed_setpoint = next_speed(ctx.speed_target, SLOPE_ACC, SLOPE_DEC, 0.001, ctx.speed_setpoint);

		encoder_update();
		ctx.speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2) / 0.001;

		ctx.speed_error = ctx.speed_target - ctx.speed_current;

		ctx.speed_pwm = pid_output(&ctx.speed_pid, ctx.speed_error);
		/*
		 */
		motor_speed_left(ctx.speed_pwm);
		motor_speed_right(ctx.speed_pwm);

		float dist = encoder_get_absolute();
		if(dist > DIST_START)
		{
			++ctx.actions_nb;
			encoder_reset();
			// TODO : positionner distance au d�but du mouvement (remaining distance)
			// TODO : remplacer reset par incr�mentation de la distance pour conserver l'�ventuelle erreur de position
		}
		/*
		//if dist_remont�e > dist consigne --> transition
		//current speed :
		//p_motors->speed
		if(p_motors->dist > p_motors->p_action_curr->distance)
		{
			//current_state=ACTION_IDLE;//transition
			//comment passer a l'action suivante ?
			//p_motors->p_action_curr = (p_motors->p_action_curr)+1; //go to next action
			current_state = controller_find_state(p_motors);
		}
		*/
	}
	break;

	case ACTION_RUN_1 :
	{
		ctx.pwm = 10;

		motor_speed_left(ctx.pwm);
		motor_speed_right(ctx.pwm);
		float dist = encoder_get_absolute();
		if(dist > DIST_RUN_1)
		{
			++ctx.actions_nb;
			encoder_reset();
		}
	}
	break;

	case ACTION_STOP :
	{
		ctx.pwm = 10;

		motor_speed_left(ctx.pwm);
		motor_speed_right(ctx.pwm);

		float dist = encoder_get_absolute();
		if(dist > DIST_STOP)
		{
			++ctx.actions_nb;
			encoder_reset();
			motor_speed_left(0);
			motor_speed_right(0);
		}
	}
	break;

	case ACTION_CTR :
	{
		// nop
	}
	break;

	}
}



//typedef  enum {
//	LEFT,
//	RIGHT
//} side_t;


/*
 * uint32_t controller_ctx_update (controller_t* p_motors, TIM_HandleTypeDef* htim2,TIM_HandleTypeDef* htim3,TIM_HandleTypeDef* htim4,TIM_HandleTypeDef* htim5);

uint32_t controller_load_actions (controller_t *p_motors, action_target_t **p_actions_targets_table);


action_t controller_find_state(controller_t *p_motors);
int controller_fsm(controller_t *p_motors);
*/
/*
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
	//Temps d execution
	//update du context

	return 0;
}
*/
/*
 * return values :
 * 0 : ok, table loaded
 * 1 : ko, empty input(s)
 * 2 : ...
 */
/*
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
*/

/*
 * return values :
 * 1 : error : could not get current action from p_motors
 *
 *
 */
/*
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

bool controller_is_end(){
	return true;
}
*/


//typedef struct {
//	float speed;
//	float distance;
//
//} action_target_t;

//
//typedef struct {
//	float dist;
//	float speed;
//
//	// TIM front
//	// TIM bask
//
//	uint32_t dist_ref_front;
//	uint32_t dist_ref_back;
//
//} controller_side_t;

