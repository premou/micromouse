/*
 * controller.c
 *
 *  Created on: 1 mars 2019
 *      Author: Invite
 */

// TODO : finalise U Turn packing STOP/U/START in one move
// TODO : build command interpreter
// TODO : wall following PID

// TODO : tune x speed PID (Kp and Ki)
// TODO : tune w speed filter and PID (Kp and Ki)

// TODO : log absolute distance

// TODO : filter x_speed using EWMA and high alpha (0.5)
// TODO : use unfiltered x speed error for Ki
// TODO : use filtered x speed error for Kp and Kd


#include <robot_math.h>
#include "controller.h"
#include "serial.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "datalogger.h"
#include "imu.h"
#include "main.h"
#include "timer_us.h"
#include "WallSensor.h"
#include <math.h>
#include <stdlib.h>     /* qsort */
#include "maze.h"

// globals
extern HAL_Serial_Handler com;

////////////
// constants
////////////

#define FIXED_MOVES
//#define ALGO_MOVES

#define CONTROLLER_PERIOD 1200U // us microseconds (= 833Hz ODR GYRO)
#define X_MAX_ACCELERATION 5.0 		// m/s-2
#define X_MAX_DECELERATION 3.0		// m/s-2
#define W_MAX_ACCELERATION 5000		// °/s-2
#define W_MAX_DECELERATION 5000		// °/s-2
#define X_SPEED_LEARNING_RUN 0.34 	// m/s
#define W_SPEED_LEARNING_RUN 205 	// °/s

#define X_SPEED_FAST_RUN 0.5 // m/s
#define W_SPEED_FAST_RUN 300 // °/s

#define X_SPEED_FAST_RUN_2 0.7 // m/s
#define W_SPEED_FAST_RUN_2 400 // °/s


#define X_SPEED_CALIBRATION -0.08 	// m/s
#define WALL_POSITION_OFFSET 0
#define DIST_START 0.09 			// m
#define DIST_RUN_1 0.18 			// m
#define DIST_STOP 0.09 				// m
#define DIST_CALIBRATION -0.20 		// m

#define W_T1 439 					//in ms
#define W_T2 480					//in ms

//#define W_U_T1 890 					//in ms
#define W_U_T1 445 					//in ms
//#define W_U_T2 930					//in ms
#define W_U_T2 465					//in ms

#define X_BREAK 500					//in ms

// speed
#define X_SPEED_KP 600.0
#define X_SPEED_KI 10.0
#define X_SPEED_KD 0.0

// rotation
#define W_SPEED_KP 0.1
#define W_SPEED_KI 0.004
#define W_SPEED_KD 0.0

// wall position
#define WALL_POSITION_KP 0.3
#define WALL_POSITION_KI 0.0
#define WALL_POSITION_KD 1.0


// wall pos calibration
#define X_WALL_FRONT_KP 0.03
#define X_WALL_FRONT_KI 0.0003
#define X_WALL_FRONT_KD 0.0

// wall pos calibration
#define W_WALL_FRONT_KP 0.03
#define W_WALL_FRONT_KI 0.0005
#define W_WALL_FRONT_KD 0.0

// ENUM
typedef enum {
	PID_TYPE_GYRO,
	PID_TYPE_WALL,
	PID_TYPE_CTR
} pid_type_t;

// STRUCTURES DEFINITIONS

typedef struct  {
	// controller fsm
	uint16_t time_us;
	action_t current_state;
	uint32_t actions_index; // index of current action in the scenario array
	uint32_t action_time;
	uint32_t sub_action_index;
	uint32_t gyro_state;
	pid_type_t current_pid_type;

	// forward speed
	float x_speed_target;
	float x_speed_setpoint;
	float x_speed_current;
	float x_speed_error;
	float x_speed_pwm;
	pid_context_t x_speed_pid;

	// rotation speed
	float w_speed_target;
	float w_speed_setpoint;
	float w_speed_current;
	float w_speed_error;
	float w_speed_pwm;
	pid_context_t w_speed_pid;

	// wall position
	float wall_position_target;
	float wall_position_setpoint;
	float wall_position_current;
	float wall_position_error;
	float wall_position_pwm;
	pid_context_t wall_position_pid;

	// wall pos calibration
	float x_wall_front_target;
	float x_wall_front_setpoint;
	float x_wall_front_current;
	float x_wall_front_error;
	float x_wall_front_pwm;
	pid_context_t x_wall_front_pid;

	// wall rotation calibration
	float w_wall_front_target;
	float w_wall_front_setpoint;
	float w_wall_front_current;
	float w_wall_front_error;
	float w_wall_front_pwm;
	pid_context_t w_wall_front_pid;

	maze_ctx_t maze;
} controller_t;

//////////
// Globals
//////////
static controller_t ctx;

////////////
// Functions
////////////

#if 0
static void led_turn_on_left(){
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // droite OFF
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET); // gauche ON
}
static void led_turn_on_right(){
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET); // droite ON
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // gauche OFF
}
static void led_turn_on(){
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET); // droite ON
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET); // gauche ON
}
static void led_turn_off(){
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // droite OFF
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // gauche OFF
}
#endif

static void led_toggle(){
	HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
	HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
}

///////////////////
// PUBLIC FUNCTIONS
///////////////////

uint32_t controller_init () // return GYRO ERROR (ZERO is GYRO OK)
{
	// reset controller fsm
	ctx.current_state = ACTION_START;
	ctx.time_us = 0;
	ctx.actions_index = 0;
	ctx.action_time = 0;
	ctx.sub_action_index = 0;

	// forward speed
	ctx.x_speed_target = 0;
	ctx.x_speed_current = 0;
	ctx.x_speed_setpoint = 0;
	ctx.x_speed_error = 0;
	ctx.x_speed_pwm = 0;
	pid_init(&ctx.x_speed_pid, X_SPEED_KP, X_SPEED_KI, X_SPEED_KD);

	// rotation speed
	ctx.w_speed_target = 0;
	ctx.w_speed_current = 0;
	ctx.w_speed_setpoint = 0;
	ctx.w_speed_error = 0;
	ctx.w_speed_pwm = 0;
	pid_init(&ctx.w_speed_pid, W_SPEED_KP, W_SPEED_KI, W_SPEED_KD);

	// wall position
	ctx.wall_position_target = 0;
	ctx.wall_position_setpoint = 0;
	ctx.wall_position_current = 0;
	ctx.wall_position_error = 0;
	ctx.wall_position_pwm = 0;
	pid_init(&ctx.wall_position_pid, WALL_POSITION_KP, WALL_POSITION_KI, WALL_POSITION_KD);


	// wall pos calibration
	ctx.x_wall_front_target = 0;
	ctx.x_wall_front_setpoint = 0;
	ctx.x_wall_front_current = 0;
	ctx.x_wall_front_error = 0;
	ctx.x_wall_front_pwm = 0;
	pid_init(&ctx.x_wall_front_pid, X_WALL_FRONT_KP, X_WALL_FRONT_KI, X_WALL_FRONT_KD);


	// wall rotation calibration
	ctx.w_wall_front_target = 0;
	ctx.w_wall_front_setpoint = 0;
	ctx.w_wall_front_current = 0;
	ctx.w_wall_front_error = 0;
	ctx.w_wall_front_pwm = 0;
	pid_init(&ctx.w_wall_front_pid, W_WALL_FRONT_KP, W_WALL_FRONT_KI, W_WALL_FRONT_KD);

	motor_init();
	encoder_init();
	ctx.gyro_state = gyro_init();
	wall_sensor_init();

	ctx.current_pid_type = PID_TYPE_WALL;

	HAL_DataLogger_Init(12, // number of fields
			1,  // size in bytes of each field
			1, 	// size in bytes of each field
			4, 	// size in bytes of each field
			4, 	// size in bytes of each field
			4, 	// size in bytes of each field
			1, 	// size in bytes of each field
			4, 	// size in bytes of each field
			4, 	// size in bytes of each field
			4, 	// size in bytes of each field
			4, 	// size in bytes of each field
			1, 	// size in bytes of each field
			4 	// size in bytes of each field

	);

	maze_ctx_init(&ctx.maze);

	return ctx.gyro_state;
}

void controller_start()
{
	// reset controller fsm
	ctx.current_state = ACTION_START;
	ctx.time_us = 0;
	ctx.actions_index = 0;
	ctx.action_time = HAL_GetTick();
	ctx.sub_action_index = 0;

	// forward speed
	ctx.x_speed_target = 0;
	ctx.x_speed_current = 0;
	ctx.x_speed_error = 0;
	ctx.x_speed_setpoint = 0;
	ctx.x_speed_pwm = 0;
	pid_reset(&ctx.x_speed_pid);

	// rotation speed
	ctx.w_speed_target = 0;
	ctx.w_speed_current = 0;
	ctx.w_speed_setpoint = 0;
	ctx.w_speed_error = 0;
	ctx.w_speed_pwm = 0;
	pid_reset(&ctx.w_speed_pid);

	// wall position
	ctx.wall_position_target = 0;
	ctx.wall_position_setpoint = 0;
	ctx.wall_position_current = 0;
	ctx.wall_position_error = 0;
	ctx.wall_position_pwm = 0;
	pid_reset(&ctx.wall_position_pid);

	// wall pos calibration
	ctx.x_wall_front_target = 0;
	ctx.x_wall_front_setpoint = 0;
	ctx.x_wall_front_current = 0;
	ctx.x_wall_front_error = 0;
	ctx.x_wall_front_pwm = 0;
	pid_reset(&ctx.x_wall_front_pid);

	// wall rotation calibration
	ctx.w_wall_front_target = 0;
	ctx.w_wall_front_setpoint = 0;
	ctx.w_wall_front_current = 0;
	ctx.w_wall_front_error = 0;
	ctx.w_wall_front_pwm = 0;
	pid_reset(&ctx.w_wall_front_pid);

	encoder_reset();

	HAL_DataLogger_Clear();

	maze_ctx_start(&ctx.maze) ;
}

void controller_stop()
{
	// reset controller fsm
	ctx.current_state = ACTION_START;
	ctx.time_us = 0;
	ctx.actions_index = 0;
	ctx.action_time = 0;
	ctx.sub_action_index = 0;


	// forward speed
	ctx.x_speed_target = 0;
	ctx.x_speed_current = 0;
	ctx.x_speed_setpoint = 0;
	ctx.x_speed_error = 0;
	ctx.x_speed_pwm = 0;
	pid_reset(&ctx.x_speed_pid);

	// rotation speed
	ctx.w_speed_target = 0;
	ctx.w_speed_current = 0;
	ctx.w_speed_setpoint = 0;
	ctx.w_speed_error = 0;
	ctx.w_speed_pwm = 0;
	pid_reset(&ctx.w_speed_pid);

	// wall position
	ctx.wall_position_target = 0;
	ctx.wall_position_setpoint = 0;
	ctx.wall_position_current = 0;
	ctx.wall_position_error = 0;
	ctx.wall_position_pwm = 0;
	pid_reset(&ctx.wall_position_pid);

	// wall pos calibration
	ctx.x_wall_front_target = 0;
	ctx.x_wall_front_setpoint = 0;
	ctx.x_wall_front_current = 0;
	ctx.x_wall_front_error = 0;
	ctx.x_wall_front_pwm = 0;
	pid_reset(&ctx.x_wall_front_pid);

	// wall rotation calibration
	ctx.w_wall_front_target = 0;
	ctx.w_wall_front_setpoint = 0;
	ctx.w_wall_front_current = 0;
	ctx.w_wall_front_error = 0;
	ctx.w_wall_front_pwm = 0;
	pid_reset(&ctx.w_wall_front_pid);

	ctx.current_pid_type = PID_TYPE_GYRO;

	encoder_reset();

	motor_speed_left(0);
	motor_speed_right(0);
}

void controller_fsm(); // forward declaration

void controller_update(){
	// controller period
	uint16_t time_us_current = timer_us_get();
	if( (time_us_current-ctx.time_us) >= CONTROLLER_PERIOD ) // wait for PERIOD, then update sensors and call controller fsm
	{
		ctx.time_us = time_us_current;
		//HAL_Serial_Print(&com,"|");

		// sensor update
		encoder_update();
		gyro_update();
		wall_sensor_update();

		// motor control update
		controller_fsm();

		// data logger
		HAL_DataLogger_Record(12, 						 // number of fields
				(int32_t)(ctx.actions_index), 				 // integer value of each field
				(int32_t)(ctx.sub_action_index),		 // integer value of each field
				(int32_t)(ctx.wall_position_target * 1.0),	 // integer value of each field
				(int32_t)(ctx.wall_position_setpoint * 1.0),// integer value of each field
				(int32_t)(ctx.wall_position_current* 1.0),	 // integer value of each field
				(int32_t)(ctx.wall_position_pwm),				 // integer value of each field
				(int32_t)(ctx.wall_position_error * 10.0),	 // integer value of each field
				(int32_t)(ctx.w_speed_target),	 // integer value of each field
				(int32_t)(ctx.w_speed_setpoint),// integer value of each field
				(int32_t)(ctx.w_speed_current),	 // integer value of each field
				(int32_t)(ctx.w_speed_pwm),				 // integer value of each field
				(int32_t)(ctx.w_speed_error)	 // integer value of each field
		);
		/*
		static uint32_t counter=0;
		if(counter++%10==0)
		{
			HAL_Serial_Print(&com,"GYRO %d\r\n",(int32_t)(gyro_get_dps()*1000));
		}
		*/

//		static uint32_t counter=0;
//		if(counter++%100==0)
//		{
//			HAL_Serial_Print(&com,"WALL %d %d %d %d\r\n",
//					wall_sensor_get(WALL_SENSOR_LEFT_STRAIGHT),
//					wall_sensor_get(WALL_SENSOR_RIGHT_STRAIGHT),
//					wall_sensor_get(WALL_SENSOR_LEFT_DIAG),
//					wall_sensor_get(WALL_SENSOR_RIGHT_DIAG)
//					);
//		}

	}
}

bool controller_is_end(){
	return ctx.current_state == ACTION_IDLE;
}

#if 1

action_t actions_scenario[] =
{
		ACTION_RUN_1,
		ACTION_RUN_1,
		ACTION_RUN_1,
		ACTION_STOP,
		ACTION_IDLE
};


action_t get_next_move()
{
//	if(actions_scenario[0] == ACTION_IDLE)
//	{
//		if(ctx.current_state == ACTION_STOP)
//		{
//			return ACTION_IDLE;
//		}
//		else
//		{
//			if(!wall_sensor_wall_left_presence())
//			{
//				HAL_Serial_Print(&com,"turn left");
//				return ACTION_TURN_LEFT;
//			}
//			else
//			{
//				if(!wall_sensor_left_front_presence())
//				{
//					HAL_Serial_Print(&com,"run 1");
//					return ACTION_RUN_1;
//				}
//				else
//				{
//					if(!wall_sensor_wall_right_presence())
//					{
//						HAL_Serial_Print(&com,"turn right");
//						return ACTION_TURN_RIGHT;
//					}
//					else
//					{
//						HAL_Serial_Print(&com,"stop");
//						return ACTION_STOP;
//					}
//				}
//			}
//		}
//	}
//	else
//	{
	HAL_Serial_Print(&com,"\nget_next_move() returns %d\n", actions_scenario[ctx.actions_index]);

		return actions_scenario[ctx.actions_index++];
//	}
}
#endif

// PRIVATE FUNCTIONS

void controller_fsm()
{
	switch(ctx.current_state)
	{
	case ACTION_IDLE :
	{
		HAL_Serial_Print(&com,"CONTROLLER ACTION_IDLE\n");
		// forward speed
		ctx.x_speed_target = 0;
		ctx.x_speed_setpoint = 0;
		ctx.x_speed_current = 0;
		ctx.x_speed_error = 0;
		ctx.x_speed_pwm = 0;

		// rotation speed
		ctx.w_speed_target = 0;
		ctx.w_speed_setpoint = 0;
		ctx.w_speed_current = 0;
		ctx.w_speed_error = 0;
		ctx.w_speed_pwm = 0;

		// wall position
		ctx.wall_position_target = 0;
		ctx.wall_position_setpoint = 0;
		ctx.wall_position_current = 0;
		ctx.wall_position_error = 0;
		ctx.wall_position_pwm = 0;

		motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
		motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

	}
	break;

	case ACTION_START :
	{
		// forward speed
		ctx.x_speed_target = X_SPEED_LEARNING_RUN;
		ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
		ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
		ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
		ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

		// rotation speed
		ctx.w_speed_target = 0;
		ctx.w_speed_setpoint = 0;
		ctx.w_speed_current = gyro_get_dps();
		ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
		ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

		motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
		motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);


		float dist = encoder_get_absolute();

		if(dist >= DIST_START)
		{
			encoder_set_absolute(dist - DIST_START);

			ctx.sub_action_index = 0;

#ifdef FIXED_MOVES
			ctx.current_state = get_next_move();
#else
			ctx.current_state = update_maze_ctx(&ctx.maze);
#endif
			ctx.action_time = HAL_GetTick();

			ctx.current_pid_type = PID_TYPE_GYRO;
			pid_reset(&ctx.wall_position_pid);

			led_toggle();
		}

	}
	break;

	case ACTION_RUN_1 :
	{
		if (get_next_next_action(&ctx.maze) == ACTION_RUN_1){
			ctx.x_speed_target = X_SPEED_LEARNING_RUN * 2;
		}
		else
		{
			ctx.x_speed_target = X_SPEED_LEARNING_RUN;
		}
		// forward speed
		ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
		ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
		ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
		ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

		// rotation speed
		ctx.w_speed_target = 0;
		ctx.w_speed_setpoint = 0;
		ctx.w_speed_current = gyro_get_dps();
		ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
		ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

		// wall position
		ctx.wall_position_target = WALL_POSITION_OFFSET;
		ctx.wall_position_setpoint = WALL_POSITION_OFFSET;
		ctx.wall_position_current = (float) wall_sensor_get_side_error();
		ctx.wall_position_error = ctx.wall_position_setpoint - ctx.wall_position_current;
		ctx.wall_position_pwm = pid_output(&ctx.wall_position_pid, ctx.wall_position_error);

		float dist = encoder_get_absolute();
		// wall side following algorithm
		if(dist < DIST_RUN_1/2.0 )
		{
			if(wall_sensor_is_left_wall_detected() || wall_sensor_is_right_wall_detected())
			{
				HAL_Serial_Print(&com,"|");
				pid_reset(&ctx.w_speed_pid);
				ctx.current_pid_type = PID_TYPE_WALL;
				//HAL_Serial_Print(&com,"wall: PID_TYPE_GYRO->PID_TYPE_WALL, dist:%d\n",(int) (dist*1000.0));
				motor_speed_left(ctx.x_speed_pwm - ctx.wall_position_pwm);
				motor_speed_right(ctx.x_speed_pwm + ctx.wall_position_pwm);
			}
			else
			{
				//HAL_Serial_Print(&com,",");
				//HAL_Serial_Print(&com,"wall presence : PID_TYPE_WALL->PID_TYPE_GYRO\n");
				pid_reset(&ctx.wall_position_pid);
				ctx.current_pid_type = PID_TYPE_GYRO;
				//HAL_Serial_Print(&com,"wall: PID_TYPE_WALL->PID_TYPE_GYRO, dist:%d\n", (int) (dist*1000.0));
				motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
				motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);
			}

		}
		else
		{
			//HAL_Serial_Print(&com,",");
			pid_reset(&ctx.wall_position_pid);
			ctx.current_pid_type = PID_TYPE_GYRO;
			//HAL_Serial_Print(&com,"no wall : PID_TYPE_WALL->PID_TYPE_GYRO\n");
			motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
			motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);
		}

		dist = encoder_get_absolute();

		if(dist >= DIST_RUN_1)
		{
			encoder_set_absolute(dist - DIST_RUN_1);

			ctx.sub_action_index = 0;

#ifdef FIXED_MOVES
			ctx.current_state = get_next_move();
#else
			ctx.current_state = update_maze_ctx(&ctx.maze);
#endif
			ctx.action_time = HAL_GetTick();

			ctx.current_pid_type = PID_TYPE_GYRO;
			pid_reset(&ctx.wall_position_pid);

			led_toggle();
		}
	}
	break;

	case ACTION_TURN_RIGHT :
	{
		switch (ctx.sub_action_index) {
		//ACCELERATION
		case 0 :
			// forward speed
			ctx.x_speed_target = X_SPEED_LEARNING_RUN;
			ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
			ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
			ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
			ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);


			// rotation speed
			ctx.w_speed_target = -W_SPEED_LEARNING_RUN;
			ctx.w_speed_setpoint = next_speed(ctx.w_speed_target, W_MAX_ACCELERATION, W_MAX_DECELERATION, 0.001, ctx.w_speed_setpoint);
			ctx.w_speed_current = gyro_get_dps();
			ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
			ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

			motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
			motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

			if(HAL_GetTick() > ctx.action_time + W_T1)
			{
				ctx.sub_action_index++;
			}
			break;
		//DECELERATION
		case 1 :
			// forward speed
			ctx.x_speed_target = X_SPEED_LEARNING_RUN;
			ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
			ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
			ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
			ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);


			// rotation speed
			ctx.w_speed_target = 0;
			ctx.w_speed_setpoint = next_speed(ctx.w_speed_target, W_MAX_ACCELERATION, W_MAX_DECELERATION, 0.001, ctx.w_speed_setpoint);
			ctx.w_speed_current = gyro_get_dps();
			ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
			ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

			motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
			motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

			if(HAL_GetTick() > ctx.action_time + W_T2)
			{
				ctx.sub_action_index = 0;

#ifdef FIXED_MOVES
			ctx.current_state = get_next_move();
#else
			ctx.current_state = update_maze_ctx(&ctx.maze);
#endif
				ctx.action_time = HAL_GetTick();

				ctx.current_pid_type = PID_TYPE_GYRO;
				pid_reset(&ctx.wall_position_pid);

				encoder_reset();

				led_toggle();
			}
			break;
		default:
			break;
		}
	}
	break;

	case ACTION_TURN_LEFT :
	{
		switch (ctx.sub_action_index) {
		//ACCELARATION
		case 0 :
			// forward speed
			ctx.x_speed_target = X_SPEED_LEARNING_RUN;
			ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
			ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
			ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
			ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);


			// rotation speed
			ctx.w_speed_target = W_SPEED_LEARNING_RUN;
			ctx.w_speed_setpoint = next_speed(ctx.w_speed_target, W_MAX_ACCELERATION, W_MAX_DECELERATION, 0.001, ctx.w_speed_setpoint);
			ctx.w_speed_current = gyro_get_dps();
			ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
			ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

			motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
			motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

			if(HAL_GetTick() > ctx.action_time + W_T1)
			{
				ctx.sub_action_index++;
			}
			break;
		//DECELERATION
		case 1 :
			// forward speed
			ctx.x_speed_target = X_SPEED_LEARNING_RUN;
			ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
			ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
			ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
			ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);


			// rotation speed
			ctx.w_speed_target = 0;
			ctx.w_speed_setpoint = next_speed(ctx.w_speed_target, W_MAX_ACCELERATION, W_MAX_DECELERATION, 0.001, ctx.w_speed_setpoint);
			ctx.w_speed_current = gyro_get_dps();
			ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
			ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

			motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
			motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

			if(HAL_GetTick() > ctx.action_time + W_T2)
			{
				ctx.sub_action_index = 0;

#ifdef FIXED_MOVES
			ctx.current_state = get_next_move();
#else
			ctx.current_state = update_maze_ctx(&ctx.maze);
#endif
				ctx.action_time = HAL_GetTick();

				ctx.current_pid_type = PID_TYPE_GYRO;
				pid_reset(&ctx.wall_position_pid);

				encoder_reset();

				led_toggle();
			}
			break;
		default:
			break;
		}
	}
	break;
	//action new u_turn is concatenation of action stop and action break and action old u_turn and action break and action start
	case ACTION_U_TURN_RIGHT :
	{
		switch (ctx.sub_action_index) {
				//STOP
				case 0 :
				{
					// forward speed
					ctx.x_speed_target = X_SPEED_LEARNING_RUN;
					ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
					ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
					ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
					ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

					// rotation speed
					ctx.w_speed_target = 0;
					ctx.w_speed_setpoint = 0;
					ctx.w_speed_current = gyro_get_dps();
					ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
					ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

					motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
					motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

					if(have_to_break(0, ctx.x_speed_setpoint, DIST_STOP-encoder_get_absolute(), X_MAX_DECELERATION))
					{
						ctx.sub_action_index++;
					}
				}
				break;
				//BREAK
					//SUB_ACTION_BREAK
				case 1 :
				{
					// forward speed
					ctx.x_speed_target = X_SPEED_LEARNING_RUN / 4.0;
					ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
					ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
					ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
					ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

					// rotation speed
					ctx.w_speed_target = 0;
					ctx.w_speed_setpoint = 0;
					ctx.w_speed_current = gyro_get_dps();
					ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
					ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

					motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
					motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

					if(ctx.x_speed_setpoint==ctx.x_speed_target)
					{
						ctx.sub_action_index++;
						ctx.action_time = HAL_GetTick();
						pid_reset(&ctx.x_wall_front_pid);
						pid_reset(&ctx.w_wall_front_pid);
					}
				}
					break;
					//SUB_ACTION_STOP
				case 2 :
				{
					// straight calibration
					// wall position
					ctx.x_wall_front_target = 2900;
					ctx.x_wall_front_setpoint = 2900;
					ctx.x_wall_front_current = wall_sensor_get_straight_adc();
					ctx.x_wall_front_error = ctx.x_wall_front_setpoint - ctx.x_wall_front_current;
					ctx.x_wall_front_pwm = pid_output(&ctx.x_wall_front_pid, ctx.x_wall_front_error);

					// rotation calibration
					// wall position
					ctx.w_wall_front_target = 0;
					ctx.w_wall_front_setpoint = -800;
					ctx.w_wall_front_current = wall_sensor_get_straight_diff_adc();
					ctx.w_wall_front_error = ctx.w_wall_front_setpoint - ctx.w_wall_front_current;
					ctx.w_wall_front_pwm = pid_output(&ctx.w_wall_front_pid, ctx.w_wall_front_error);

					motor_speed_left(ctx.x_wall_front_pwm + ctx.w_wall_front_pwm);
					motor_speed_right(ctx.x_wall_front_pwm - ctx.w_wall_front_pwm);

					if(HAL_GetTick() > ctx.action_time + 1000)
					{
						ctx.sub_action_index++;
						encoder_reset();
						led_toggle();
						ctx.action_time = HAL_GetTick();
						pid_reset(&ctx.x_speed_pid);
						pid_reset(&ctx.w_speed_pid);
					}

				}
				break;

				//FIRST TURN TO THE RIGHT AT 90°
				case 3 :
				{
					// forward speed
					ctx.x_speed_target = 0;
					ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
					ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
					ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
					ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

					// rotation speed
					ctx.w_speed_target = -W_SPEED_LEARNING_RUN;
					ctx.w_speed_setpoint = next_speed(ctx.w_speed_target, W_MAX_ACCELERATION, W_MAX_DECELERATION, 0.001, ctx.w_speed_setpoint);
					ctx.w_speed_current = gyro_get_dps();
					ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
					ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

					motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
					motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

					if(HAL_GetTick() > ctx.action_time + W_U_T1)
					{
						ctx.sub_action_index++;
					}
				}
				break;
				case 4 :
				{
					motor_speed_left(0);
					motor_speed_right(0);

					if(HAL_GetTick() > ctx.action_time + X_BREAK)
					{
						ctx.sub_action_index++;
						encoder_reset();

					}
				}
				break;
				case 5 :
				{
					// straight calibration
					// wall position
					ctx.x_wall_front_target = 2900;
					ctx.x_wall_front_setpoint = 2900;
					ctx.x_wall_front_current = wall_sensor_get_straight_adc();
					ctx.x_wall_front_error = ctx.x_wall_front_setpoint - ctx.x_wall_front_current;
					ctx.x_wall_front_pwm = pid_output(&ctx.x_wall_front_pid, ctx.x_wall_front_error);

					// rotation calibration
					// wall position
					ctx.w_wall_front_target = 0;
					ctx.w_wall_front_setpoint = -800;
					ctx.w_wall_front_current = wall_sensor_get_straight_diff_adc();
					ctx.w_wall_front_error = ctx.w_wall_front_setpoint - ctx.w_wall_front_current;
					ctx.w_wall_front_pwm = pid_output(&ctx.w_wall_front_pid, ctx.w_wall_front_error);

					motor_speed_left(ctx.x_wall_front_pwm + ctx.w_wall_front_pwm);
					motor_speed_right(ctx.x_wall_front_pwm - ctx.w_wall_front_pwm);

					if(HAL_GetTick() > ctx.action_time + 1000)
					{
						ctx.sub_action_index++;
						encoder_reset();
						led_toggle();
						ctx.action_time = HAL_GetTick();
						pid_reset(&ctx.x_speed_pid);
						pid_reset(&ctx.w_speed_pid);
					}

				}
				break;

				//SECOND TURN TO THE RIGHT AT 90°
				case 6 :
				{
					// forward speed
					ctx.x_speed_target = 0;
					ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
					ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
					ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
					ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

					// rotation speed
					ctx.w_speed_target = -W_SPEED_LEARNING_RUN;
					ctx.w_speed_setpoint = next_speed(ctx.w_speed_target, W_MAX_ACCELERATION, W_MAX_DECELERATION, 0.001, ctx.w_speed_setpoint);
					ctx.w_speed_current = gyro_get_dps();
					ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
					ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

					motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
					motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

					if(HAL_GetTick() > ctx.action_time + W_U_T1)
					{
						ctx.sub_action_index++;
					}
				}
				break;
				case 7 :
				{
					// forward speed
					ctx.x_speed_target = 0;
					ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
					ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
					ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
					ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);


					// rotation speed
					ctx.w_speed_target = 0;
					ctx.w_speed_setpoint = next_speed(ctx.w_speed_target, W_MAX_ACCELERATION, W_MAX_DECELERATION, 0.001, ctx.w_speed_setpoint);
					ctx.w_speed_current = gyro_get_dps();
					ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
					ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

					motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
					motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

					if(HAL_GetTick() > ctx.action_time + W_U_T2)
					{
						//ctx.sub_action_index = 0;
						ctx.sub_action_index++;
						ctx.action_time = HAL_GetTick();
						encoder_reset();

						led_toggle();
					}
				}
					break;
					//BREAK
//				case 8 :
//				{
//					motor_speed_left(0);
//					motor_speed_right(0);
//
//					if(HAL_GetTick() > ctx.action_time + X_BREAK)
//					{
//						ctx.sub_action_index++;
//						encoder_reset();
//
//					}
//				}
//				break;
				//START
				case 8 :
				{
						// forward speed
						ctx.x_speed_target = X_SPEED_LEARNING_RUN;
						ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
						ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
						ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
						ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

						// rotation speed
						ctx.w_speed_target = 0;
						ctx.w_speed_setpoint = 0;
						ctx.w_speed_current = gyro_get_dps();
						ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
						ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

						// wall position
						ctx.wall_position_target = WALL_POSITION_OFFSET;
						ctx.wall_position_setpoint = WALL_POSITION_OFFSET;
						ctx.wall_position_current = (float) wall_sensor_get_side_error();
						ctx.wall_position_error = ctx.wall_position_setpoint - ctx.wall_position_current;
						ctx.wall_position_pwm = pid_output(&ctx.wall_position_pid, ctx.wall_position_error);

						motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm - ctx.wall_position_pwm);
						motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm + ctx.wall_position_pwm);

						float dist = encoder_get_absolute();

						if(dist >= DIST_START)
						{
							encoder_set_absolute(dist - DIST_START);

							ctx.sub_action_index = 0;

#ifdef FIXED_MOVES
			ctx.current_state = get_next_move();
#else
			ctx.current_state = update_maze_ctx(&ctx.maze);
#endif
							ctx.action_time = HAL_GetTick();

							ctx.current_pid_type = PID_TYPE_GYRO;
							pid_reset(&ctx.wall_position_pid);

							led_toggle();
						}
					}
					break;
				default:
					break;
				}
	}
	break;

	case ACTION_STOP :
	{
		switch (ctx.sub_action_index) {
			//SUB_ACTION_RUN
			case 0 :
			{

				// forward speed
				ctx.x_speed_target = X_SPEED_LEARNING_RUN;
				ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
				ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
				ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
				ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

				// rotation speed
				ctx.w_speed_target = 0;
				ctx.w_speed_setpoint = 0;
				ctx.w_speed_current = gyro_get_dps();
				ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
				ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

				motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
				motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

				if(have_to_break(0, ctx.x_speed_setpoint, DIST_STOP-encoder_get_absolute(), X_MAX_DECELERATION))
				{
					//HAL_Serial_Print(&com,"Have to break!!\n");
					ctx.sub_action_index++;
				}
			}
			break;

			//SUB_ACTION_BREAK
			case 1 :
			{
				// forward speed
				ctx.x_speed_target = 0;
				ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
				ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
				ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
				ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

				// rotation speed
				ctx.w_speed_target = 0;
				ctx.w_speed_setpoint = 0;
				ctx.w_speed_current = gyro_get_dps();
				ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
				ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

				motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
				motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

				if(ctx.x_speed_setpoint==ctx.x_speed_target)
				{
					ctx.sub_action_index++;
				}

			}
			break;

			//SUB_ACTION_STOP
			case 2 :
			{
				// forward speed
				ctx.x_speed_target = 0;
				ctx.x_speed_setpoint = 0;
				ctx.x_speed_current = 0;
				ctx.x_speed_error = 0;
				ctx.x_speed_pwm = 0;

				// rotation speed
				ctx.w_speed_target = 0;
				ctx.w_speed_setpoint = 0;
				ctx.w_speed_current = 0;
				ctx.w_speed_error = 0;
				ctx.w_speed_pwm = 0;

				motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
				motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

				ctx.sub_action_index = 0;

				ctx.current_state = ACTION_IDLE ;
#ifdef FIXED_MOVES
			ctx.current_state = get_next_move();
#else
			ctx.current_state = ACTION_IDLE ;
#endif
				ctx.action_time = HAL_GetTick();

				encoder_reset();

				led_toggle();
				HAL_Serial_Print(&com,".");

				if(ctx.maze.mode == SOLVE)
				{
					display_maze_ctx(&ctx.maze);
				}
			}
			break;
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

///// wall calibration helper
//enum {
//	CALIBRATION_IDLE,
//	CALIBRATION_NO_WALL,
//	CALIBRATION_POST_LEFT,
//	CALIBRATION_POST_RIGHT,
//	CALIBRATION_POST_BOTH,
//	CALIBRATION_WALL_LEFT,
//	CALIBRATION_WALL_RIGHT,
//	CALIBRATION_WALL_BOTH,
//	CALIBRATION_WALL_LEFT_with_RIGHT_POST,
//	CALIBRATION_WALL_RIGHT_with_LEFT_POST,
//	CALIBRATION_END,
//};
//uint32_t calibration_state = CALIBRATION_IDLE;
//float ewma_alpha_calibration_wall_distance = 0.25f;
//ewma_handler ewma_calibration_wall_distance_left = {&ewma_alpha_calibration_wall_distance, 0.0};
//ewma_handler ewma_calibration_wall_distance_right = {&ewma_alpha_calibration_wall_distance, 0.0};
//float calibration_wall_distance_left = 0.0f;
//float calibration_wall_distance_right = 0.0f;
//
//void calibration_reset()
//{
//	calibration_state = CALIBRATION_IDLE;
//	reset_ewma(&ewma_calibration_wall_distance_left);
//	reset_ewma(&ewma_calibration_wall_distance_right);
//	calibration_wall_distance_left = 0.0f;
//	calibration_wall_distance_right = 0.0f;
//}
//
//bool calibration_process()
//{
//	calibration_wall_distance_left = process_ewma(&ewma_calibration_wall_distance_left,HAL_WallSensor_Get(WALL_SENSOR_LEFT_DIAG));
//	calibration_wall_distance_right = process_ewma(&ewma_calibration_wall_distance_right,HAL_WallSensor_Get(WALL_SENSOR_RIGHT_DIAG));
//	switch(calibration_state)
//	{
//	case CALIBRATION_IDLE:
//		{
//			if(remaining_distance<0.15f)
//			{
//				if(calibration_wall_distance_left<120 && calibration_wall_distance_right<120)
//				{
//					calibration_state = CALIBRATION_WALL_BOTH;
//				}
//				else if(calibration_wall_distance_left<120)
//				{
//					calibration_state = CALIBRATION_WALL_LEFT;
//				}
//				else if(calibration_wall_distance_right<120)
//				{
//					calibration_state = CALIBRATION_WALL_RIGHT;
//				}
//				else
//				{
//					calibration_state = CALIBRATION_NO_WALL;
//				}
//			}
//		}
//		break;
//	case CALIBRATION_NO_WALL:
//		{
//			if(calibration_wall_distance_left<140 && calibration_wall_distance_right<140)
//			{
//				calibration_state = CALIBRATION_POST_BOTH;
//			}
//			else if(calibration_wall_distance_left<140)
//			{
//				calibration_state = CALIBRATION_POST_LEFT;
//			}
//			else if(calibration_wall_distance_right<140)
//			{
//				calibration_state = CALIBRATION_POST_RIGHT;
//			}
//		}
//		break;
//	case CALIBRATION_POST_LEFT:
//		{
//			if(calibration_wall_distance_left>140)
//			{
//				remaining_distance = 0.070;
//				calibration_state = CALIBRATION_END;
//				return true;
//			}
//			if(calibration_wall_distance_right<140)
//			{
//				calibration_state = CALIBRATION_POST_BOTH;
//			}
//		}
//		break;
//	case CALIBRATION_POST_RIGHT:
//		{
//			if(calibration_wall_distance_right>140)
//			{
//				remaining_distance = 0.070;
//				calibration_state = CALIBRATION_END;
//				return true;
//			}
//			if(calibration_wall_distance_left<140)
//			{
//				calibration_state = CALIBRATION_POST_BOTH;
//			}
//		}
//		break;
//	case CALIBRATION_POST_BOTH:
//		{
//			if(calibration_wall_distance_right>140 || calibration_wall_distance_left>140)
//			{
//				remaining_distance = 0.070;
//				calibration_state = CALIBRATION_END;
//				return true;
//			}
//		}
//		break;
//	case CALIBRATION_WALL_LEFT:
//		{
//			if(calibration_wall_distance_left>140)
//			{
//				remaining_distance = 0.065;
//				calibration_state = CALIBRATION_END;
//				return true;
//			}
//			if(calibration_wall_distance_right<140)
//			{
//				calibration_state = CALIBRATION_WALL_LEFT_with_RIGHT_POST;
//			}
//		}
//		break;
//	case CALIBRATION_WALL_RIGHT:
//		{
//			if(calibration_wall_distance_right>140)
//			{
//				remaining_distance = 0.065;
//				calibration_state = CALIBRATION_END;
//				return true;
//			}
//			if(calibration_wall_distance_left<140)
//			{
//				calibration_state = CALIBRATION_WALL_RIGHT_with_LEFT_POST;
//			}
//		}
//		break;
//	case CALIBRATION_WALL_BOTH:
//		{
//			if(calibration_wall_distance_right>140 || calibration_wall_distance_left>140)
//			{
//				remaining_distance = 0.065;
//				calibration_state = CALIBRATION_END;
//				return true;
//			}
//		}
//		break;
//	case CALIBRATION_WALL_LEFT_with_RIGHT_POST:
//		{
//			if(calibration_wall_distance_left>140)
//			{
//				remaining_distance = 0.065;
//				calibration_state = CALIBRATION_END;
//				return true;
//			}
//			if(calibration_wall_distance_right>140)
//			{
//				remaining_distance = 0.070;
//				calibration_state = CALIBRATION_END;
//				return true;
//			}
//
//		}
//		break;
//	case CALIBRATION_WALL_RIGHT_with_LEFT_POST:
//		{
//			if(calibration_wall_distance_right>140)
//			{
//				remaining_distance = 0.065;
//				calibration_state = CALIBRATION_END;
//				return true;
//			}
//			if(calibration_wall_distance_left>140)
//			{
//				remaining_distance = 0.070;
//				calibration_state = CALIBRATION_END;
//				return true;
//			}
//
//		}
//		break;
//
//	case CALIBRATION_END:
//		{
//
//		}
//		break;
//	}
//	return false;
//}
