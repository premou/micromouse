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

#define SPEED_KP 100.0
#define SPEED_KI 0.0
#define SPEED_KD 0.0

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
	// TODO : add sub_action_state

	// speed
	float speed_target;
	float speed_setpoint;
	float speed_current;
	float speed_error;
	float speed_pwm;

	pid_context_t speed_pid;

	// rotation speed
	// TODO : To Be Completed

} controller_t;

// GLOBAL VARIABLES

static action_t actions_scenario[] = {
	ACTION_START,
	ACTION_RUN_1,
	ACTION_RUN_1,
	ACTION_STOP,
	ACTION_IDLE
  };

static controller_t ctx;

// PUBLIC FUNCTIONS

void controller_init ()
{
	/*revenir au début de la liste des actions*/
	ctx.actions_nb = 0;
	ctx.time = 0;

	// speed
	ctx.speed_target = 0;
	ctx.speed_current = 0;
	ctx.speed_error = 0;
	ctx.speed_setpoint = 0;
	ctx.speed_pwm = 0;

	pid_init(&ctx.speed_pid, SPEED_KP, SPEED_KI, SPEED_KD);

	// rotation speed
	// TODO : To Be Completed

	motor_init();
	encoder_init();

	HAL_DataLogger_Init(5,
			4,
			4,
			4,
			4,
			1); // TODO : to be completed with each recorded field size
}

void controller_start(){
	/*revenir au début de la liste des actions*/
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
	// cadence a 1ms
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
		ctx.speed_target = SPEED_TARGET;
		ctx.speed_setpoint = next_speed(ctx.speed_target, SLOPE_ACC, SLOPE_DEC, 0.001, ctx.speed_setpoint);
		ctx.speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
		ctx.speed_error = ctx.speed_target - ctx.speed_current;
		ctx.speed_pwm = pid_output(&ctx.speed_pid, ctx.speed_error);
		motor_speed_left(ctx.speed_pwm);
		motor_speed_right(ctx.speed_pwm);


		float dist = encoder_get_absolute();
		if(dist > DIST_START)
		{
			++ctx.actions_nb;
			encoder_reset();
			// TODO : positionner distance au début du mouvement (remaining distance)
			// TODO : remplacer reset par incrémentation de la distance pour conserver l'éventuelle erreur de position
		}
	}
	break;

	case ACTION_RUN_1 :
	{
		ctx.speed_target = SPEED_TARGET;
		ctx.speed_setpoint = next_speed(ctx.speed_target, SLOPE_ACC, SLOPE_DEC, 0.001, ctx.speed_setpoint);
		ctx.speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
		ctx.speed_error = ctx.speed_target - ctx.speed_current;
		ctx.speed_pwm = pid_output(&ctx.speed_pid, ctx.speed_error);
		motor_speed_left(ctx.speed_pwm);
		motor_speed_right(ctx.speed_pwm);


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
		// TODO : STOP is a two-phase action
		// TODO : add a sub_action_state into ctx
		// TODO : reset sub_action_state at the beginning of each action
		// TODO : use sub_action_index to make a local fsm : switch(sub_action_state) { case 0: /.../ case 1: /..../}
		// TODO : first phase : keep running at cruise speed
		// TODO : second phase : decrease speed until stop
		// TODO : first to second action transition : use have_to_break()



		ctx.speed_target = SPEED_TARGET;
		ctx.speed_setpoint = next_speed(ctx.speed_target, SLOPE_ACC, SLOPE_DEC, 0.001, ctx.speed_setpoint);
		ctx.speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
		ctx.speed_error = ctx.speed_target - ctx.speed_current;
		//ctx.speed_pwm = pid_output(&ctx.speed_pid, ctx.speed_error);
		ctx.speed_pwm = 10; // To Be Deleted
		motor_speed_left(ctx.speed_pwm);
		motor_speed_right(ctx.speed_pwm);

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
