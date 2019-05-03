/*
 * controller.c
 *
 *  Created on: 1 mars 2019
 *      Author: Invite
 */

#include "robot_math.h"
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
#include "maze.h"

#include <math.h>

// globals
extern HAL_Serial_Handler com;

////////////
// constants
////////////

// move debug
#define FIXED_MOVES

// period
#define CONTROLLER_PERIOD 1200U // us microseconds (= 833Hz ODR GYRO)

// accelerations
#define X_MAX_ACCELERATION 5.0 		// m/s-2
#define X_MAX_DECELERATION 3.0		// m/s-2
#define W_MAX_ACCELERATION 5000		// °/s-2
#define W_MAX_DECELERATION 5000		// °/s-2

// speed
#define X_SPEED 0.34 	// m/s
#define W_SPEED 205.0 	// dps
#define X_SPEED_FAST_RUN 0.5 // m/s
#define X_SPEED_FAST_RUN_IMPROVED 0.7 // m/s

// distance
#define DIST_START 0.09 			// m
#define DIST_RUN_1 0.18 			// m
#define DIST_STOP 0.09 				// m
#define REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL 0.110 // m
#define REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST 0.115 // m

// L curve
#define W_T1 439 					//in ms
#define W_T2 480					//in ms

// U turn
//#define W_U_T1 890 					//in ms
#define W_U_T1 445 					//in ms
//#define W_U_T2 930					//in ms
#define W_U_T2 465					//in ms

// speed PID
#define X_SPEED_KP 600.0
#define X_SPEED_KI 10.0
#define X_SPEED_KD 0.0

// rotation speed  PID
#define W_SPEED_KP 0.1
#define W_SPEED_KI 0.004
#define W_SPEED_KD 0.0

// wall following position  PID
#define WALL_POSITION_KP 0.3
#define WALL_POSITION_KI 0.0
#define WALL_POSITION_KD 1.0
#define WALL_POSITION_OFFSET 0

// front wall distance position  PID
#define X_WALL_FRONT_KP 1.0
#define X_WALL_FRONT_KI 0.0001
#define X_WALL_FRONT_KD 0.0
#define WALL_FRONT_DISTANCE_mm 32.0 // mm

// front wall angle position  PID
#define W_WALL_FRONT_KP 1.0
#define W_WALL_FRONT_KI 0.0001
#define W_WALL_FRONT_KD 0.0
#define WALL_FRONT_ANGLE_mm 0.0 // mm

// ENUM
typedef enum {
	PID_TYPE_GYRO,
	PID_TYPE_WALL,
	PID_TYPE_CTR
} pid_type_t;

enum {
	CALIBRATION_IDLE,
	CALIBRATION_NO_WALL,
	CALIBRATION_POST_LEFT,
	CALIBRATION_POST_RIGHT,
	CALIBRATION_POST_BOTH,
	CALIBRATION_WALL_LEFT,
	CALIBRATION_WALL_RIGHT,
	CALIBRATION_WALL_BOTH,
	CALIBRATION_WALL_LEFT_with_RIGHT_POST,
	CALIBRATION_WALL_RIGHT_with_LEFT_POST,
	CALIBRATION_END,
} calibration_state_t;

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

	// forward speed PID
	float x_speed_target;
	float x_speed_setpoint;
	float x_speed_current;
	float x_speed_error;
	float x_speed_pwm;
	pid_context_t x_speed_pid;

	// rotation speed PID
	float w_speed_target;
	float w_speed_setpoint;
	float w_speed_current;
	float w_speed_error;
	float w_speed_pwm;
	pid_context_t w_speed_pid;

	// wall following position PID
	float wall_position_target;
	float wall_position_setpoint;
	float wall_position_current;
	float wall_position_error;
	float wall_position_pwm;
	pid_context_t wall_position_pid;

	// front wall distance position PID
	float x_wall_front_target;
	float x_wall_front_setpoint;
	float x_wall_front_current;
	float x_wall_front_error;
	float x_wall_front_pwm;
	pid_context_t x_wall_front_pid;

	// front wall angle position PID
	float w_wall_front_target;
	float w_wall_front_setpoint;
	float w_wall_front_current;
	float w_wall_front_error;
	float w_wall_front_pwm;
	pid_context_t w_wall_front_pid;

	// longitudinal calibration (wall-to-no-wall, post-to-no-post)
	uint32_t calibration_state;
	filter_ctx_t filter_calibration_wall_distance_left;
	filter_ctx_t filter_calibration_wall_distance_right;

	// AI
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
	ctx.time_us = 0;
	ctx.current_state = ACTION_START;
	ctx.actions_index = 0;
	ctx.action_time = 0;
	ctx.sub_action_index = 0;
	ctx.gyro_state = gyro_init();
	ctx.current_pid_type = PID_TYPE_GYRO;

	// forward speed PID
	ctx.x_speed_target = 0;
	ctx.x_speed_current = 0;
	ctx.x_speed_setpoint = 0;
	ctx.x_speed_error = 0;
	ctx.x_speed_pwm = 0;
	pid_init(&ctx.x_speed_pid, X_SPEED_KP, X_SPEED_KI, X_SPEED_KD);

	// rotation speed PID
	ctx.w_speed_target = 0;
	ctx.w_speed_current = 0;
	ctx.w_speed_setpoint = 0;
	ctx.w_speed_error = 0;
	ctx.w_speed_pwm = 0;
	pid_init(&ctx.w_speed_pid, W_SPEED_KP, W_SPEED_KI, W_SPEED_KD);

	// wall following position PID
	ctx.wall_position_target = 0;
	ctx.wall_position_setpoint = 0;
	ctx.wall_position_current = 0;
	ctx.wall_position_error = 0;
	ctx.wall_position_pwm = 0;
	pid_init(&ctx.wall_position_pid, WALL_POSITION_KP, WALL_POSITION_KI, WALL_POSITION_KD);

	// front wall distance position PID
	ctx.x_wall_front_target = 0;
	ctx.x_wall_front_setpoint = 0;
	ctx.x_wall_front_current = 0;
	ctx.x_wall_front_error = 0;
	ctx.x_wall_front_pwm = 0;
	pid_init(&ctx.x_wall_front_pid, X_WALL_FRONT_KP, X_WALL_FRONT_KI, X_WALL_FRONT_KD);

	// front wall angle position PID
	ctx.w_wall_front_target = 0;
	ctx.w_wall_front_setpoint = 0;
	ctx.w_wall_front_current = 0;
	ctx.w_wall_front_error = 0;
	ctx.w_wall_front_pwm = 0;
	pid_init(&ctx.w_wall_front_pid, W_WALL_FRONT_KP, W_WALL_FRONT_KI, W_WALL_FRONT_KD);

	// longitudinal calibration (wall-to-no-wall, post-to-no-post)
	ctx.calibration_state = CALIBRATION_IDLE;
	filter_init(&ctx.filter_calibration_wall_distance_left,0.25f);
	filter_init(&ctx.filter_calibration_wall_distance_right,0.25f);

	// AI
	maze_ctx_init(&ctx.maze);

	// action
	motor_init();
	encoder_init();
	wall_sensor_init();
	motor_speed_left(0);
	motor_speed_right(0);

	HAL_DataLogger_Init(12, // number of fields
			1,  // size in bytes of each field
			4, 	// size in bytes of each field
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

	return ctx.gyro_state;
}

void calibration_reset(); // forward declaration

void controller_start()
{
	// reset controller fsm
	ctx.time_us = 0;
	ctx.current_state = ACTION_START;
	ctx.actions_index = 0;
	ctx.action_time = HAL_GetTick();
	ctx.sub_action_index = 0;
	ctx.current_pid_type = PID_TYPE_GYRO;

	// forward speed PID
	ctx.x_speed_target = 0;
	ctx.x_speed_current = 0;
	ctx.x_speed_error = 0;
	ctx.x_speed_setpoint = 0;
	ctx.x_speed_pwm = 0;
	pid_reset(&ctx.x_speed_pid);

	// rotation speed PID
	ctx.w_speed_target = 0;
	ctx.w_speed_current = 0;
	ctx.w_speed_setpoint = 0;
	ctx.w_speed_error = 0;
	ctx.w_speed_pwm = 0;
	pid_reset(&ctx.w_speed_pid);

	// wall following position PID
	ctx.wall_position_target = 0;
	ctx.wall_position_setpoint = 0;
	ctx.wall_position_current = 0;
	ctx.wall_position_error = 0;
	ctx.wall_position_pwm = 0;
	pid_reset(&ctx.wall_position_pid);

	// front wall distance position PID
	ctx.x_wall_front_target = 0;
	ctx.x_wall_front_setpoint = 0;
	ctx.x_wall_front_current = 0;
	ctx.x_wall_front_error = 0;
	ctx.x_wall_front_pwm = 0;
	pid_reset(&ctx.x_wall_front_pid);

	// front wall angle position PID
	ctx.w_wall_front_target = 0;
	ctx.w_wall_front_setpoint = 0;
	ctx.w_wall_front_current = 0;
	ctx.w_wall_front_error = 0;
	ctx.w_wall_front_pwm = 0;
	pid_reset(&ctx.w_wall_front_pid);

	// longitudinal calibration (wall-to-no-wall, post-to-no-post)
	ctx.calibration_state = CALIBRATION_IDLE;
	filter_reset(&ctx.filter_calibration_wall_distance_left);
	filter_reset(&ctx.filter_calibration_wall_distance_right);

	// AI
	maze_ctx_start(&ctx.maze) ;

	// action
	encoder_reset();
	calibration_reset();

	motor_speed_left(0);
	motor_speed_right(0);

	HAL_DataLogger_Clear();
}

void controller_stop()
{
	// reset controller fsm
	ctx.time_us = 0;
	ctx.current_state = ACTION_START;
	ctx.actions_index = 0;
	ctx.action_time = 0;
	ctx.sub_action_index = 0;
	ctx.current_pid_type = PID_TYPE_GYRO;

	// forward speed PID
	ctx.x_speed_target = 0;
	ctx.x_speed_current = 0;
	ctx.x_speed_setpoint = 0;
	ctx.x_speed_error = 0;
	ctx.x_speed_pwm = 0;
	pid_reset(&ctx.x_speed_pid);

	// rotation speed PID
	ctx.w_speed_target = 0;
	ctx.w_speed_current = 0;
	ctx.w_speed_setpoint = 0;
	ctx.w_speed_error = 0;
	ctx.w_speed_pwm = 0;
	pid_reset(&ctx.w_speed_pid);

	// wall following position PID
	ctx.wall_position_target = 0;
	ctx.wall_position_setpoint = 0;
	ctx.wall_position_current = 0;
	ctx.wall_position_error = 0;
	ctx.wall_position_pwm = 0;
	pid_reset(&ctx.wall_position_pid);

	// front wall distance position PID
	ctx.x_wall_front_target = 0;
	ctx.x_wall_front_setpoint = 0;
	ctx.x_wall_front_current = 0;
	ctx.x_wall_front_error = 0;
	ctx.x_wall_front_pwm = 0;
	pid_reset(&ctx.x_wall_front_pid);

	// front wall angle position PID
	ctx.w_wall_front_target = 0;
	ctx.w_wall_front_setpoint = 0;
	ctx.w_wall_front_current = 0;
	ctx.w_wall_front_error = 0;
	ctx.w_wall_front_pwm = 0;
	pid_reset(&ctx.w_wall_front_pid);

	// longitudinal calibration (wall-to-no-wall, post-to-no-post)
	ctx.calibration_state = CALIBRATION_IDLE;
	filter_reset(&ctx.filter_calibration_wall_distance_left);
	filter_reset(&ctx.filter_calibration_wall_distance_right);

	// AI
	// nop

	// action
	encoder_reset();
	calibration_reset();

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
				//(int32_t)(ctx.sub_action_index),		 // integer value of each field
				(int32_t)(encoder_get_absolute()*1000.0),		 // integer value of each field
				(int32_t)(ctx.x_wall_front_target * 1000.0),	 // target speed
				(int32_t)(ctx.x_wall_front_setpoint * 1000.0),// setpoint speed
				(int32_t)(ctx.x_wall_front_current* 1000.0),	 // current speed
				(int32_t)(ctx.x_wall_front_pwm),				 // pwm
				(int32_t)(ctx.x_wall_front_error * 1000.0),	 //speed error
				(int32_t)(ctx.w_wall_front_target),	 // integer value of each field
				(int32_t)(ctx.w_wall_front_setpoint),// integer value of each field
				(int32_t)(ctx.w_wall_front_current),	 // integer value of each field
				(int32_t)(ctx.w_wall_front_pwm),				 // integer value of each field
				(int32_t)(ctx.w_wall_front_error)	 // integer value of each field
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
//		ACTION_RUN_1,
//		ACTION_RUN_1,
//		ACTION_RUN_1,
		ACTION_U_TURN_RIGHT,
		ACTION_STILL,
		//ACTION_TURN_RIGHT,
		//ACTION_RUN_1,
//		ACTION_STOP,
		ACTION_IDLE
};


action_t get_next_move()
{
	HAL_Serial_Print(&com,"\nget_next_move() returns %d\n", actions_scenario[ctx.actions_index]);
	return actions_scenario[ctx.actions_index++];
}
#endif

// PRIVATE FUNCTIONS

bool calibration_update(); // forward declaration

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

		motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
		motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

	}
	break;

	case ACTION_START :
	{
		// forward speed
		ctx.x_speed_target = X_SPEED;
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

		if(encoder_get_absolute() >= DIST_START)
		{
#ifdef FIXED_MOVES
			ctx.current_state = get_next_move();
#else
			ctx.current_state = update_maze_ctx(&ctx.maze);
#endif
			ctx.sub_action_index = 0;
			ctx.action_time = HAL_GetTick();
			ctx.current_pid_type = PID_TYPE_GYRO;

			encoder_set_absolute(encoder_get_absolute() - DIST_START);
			calibration_reset();
			pid_reset(&ctx.wall_position_pid);

			led_toggle();
		}

	}
	break;

	case ACTION_RUN_1 :
	{
		// forward speed
#ifdef FIXED_MOVES
		if( actions_scenario[ctx.actions_index] ==  ACTION_RUN_1 )
#else
		if (get_next_next_action(&ctx.maze) == ACTION_RUN_1)
#endif
		{
			ctx.x_speed_target = X_SPEED_FAST_RUN;
		}
		else
		{
			ctx.x_speed_target = X_SPEED;
		}
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

		// wall side following algorithm
		if(encoder_get_absolute() <= DIST_RUN_1/2.0 )
		{
			if(wall_sensor_is_left_wall_detected() || wall_sensor_is_right_wall_detected())
			{
				//HAL_Serial_Print(&com,"|");
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

		calibration_update();
		if(encoder_get_absolute() >= DIST_RUN_1)
		{
#ifdef FIXED_MOVES
			ctx.current_state = get_next_move();
#else
			ctx.current_state = update_maze_ctx(&ctx.maze);
#endif
			ctx.sub_action_index = 0;
			ctx.action_time = HAL_GetTick();
			ctx.current_pid_type = PID_TYPE_GYRO;

			encoder_set_absolute(encoder_get_absolute() - DIST_RUN_1);
			calibration_reset();
			pid_reset(&ctx.wall_position_pid);

			led_toggle();
		}
	}
	break;

	case ACTION_TURN_RIGHT :
	{
		switch (ctx.sub_action_index)
		{
		//ACCELERATION
		case 0 :
			// forward speed
			ctx.x_speed_target = X_SPEED;
			ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
			ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
			ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
			ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

			// rotation speed
			ctx.w_speed_target = -W_SPEED;
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
			ctx.x_speed_target = X_SPEED;
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
#ifdef FIXED_MOVES
			ctx.current_state = get_next_move();
#else
			ctx.current_state = update_maze_ctx(&ctx.maze);
#endif
				ctx.sub_action_index = 0;
				ctx.action_time = HAL_GetTick();
				ctx.current_pid_type = PID_TYPE_GYRO;

				encoder_reset();
				calibration_reset();
				pid_reset(&ctx.wall_position_pid);

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
			ctx.x_speed_target = X_SPEED;
			ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
			ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
			ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
			ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);


			// rotation speed
			ctx.w_speed_target = W_SPEED;
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
			ctx.x_speed_target = X_SPEED;
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
#ifdef FIXED_MOVES
			ctx.current_state = get_next_move();
#else
			ctx.current_state = update_maze_ctx(&ctx.maze);
#endif
				ctx.sub_action_index = 0;
				ctx.action_time = HAL_GetTick();
				ctx.current_pid_type = PID_TYPE_GYRO;

				encoder_reset();
				calibration_reset();
				pid_reset(&ctx.wall_position_pid);

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
				// CRUISE
				case 0 :
				{
					// forward speed
					ctx.x_speed_target = X_SPEED;
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
				//BRAKE
				case 1 :
				{
					// forward speed
					ctx.x_speed_target = 0.1;
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
					if(dist >= DIST_STOP)
					{
						ctx.sub_action_index++;
						ctx.action_time = HAL_GetTick();
						encoder_reset();
						pid_reset(&ctx.x_speed_pid);
						pid_reset(&ctx.w_speed_pid);
						pid_reset(&ctx.x_wall_front_pid);
						pid_reset(&ctx.w_wall_front_pid);
					}
				}
					break;
				//FRONT WALL CALIBRATE
				case 2 :
				{
					// straight calibration
					// wall position
					ctx.x_wall_front_target = WALL_FRONT_DISTANCE_mm; // mm
					ctx.x_wall_front_setpoint = WALL_FRONT_DISTANCE_mm; // mm
					ctx.x_wall_front_current = ( wall_sensor_get_dist(WALL_SENSOR_LEFT_STRAIGHT) + wall_sensor_get_dist(WALL_SENSOR_RIGHT_STRAIGHT) ) / 2.0;
					ctx.x_wall_front_error = ctx.x_wall_front_setpoint - ctx.x_wall_front_current;
					ctx.x_wall_front_pwm = pid_output(&ctx.x_wall_front_pid, ctx.x_wall_front_error);

					// rotation calibration
					// wall position
					ctx.w_wall_front_target = WALL_FRONT_ANGLE_mm; // mm
					ctx.w_wall_front_setpoint = WALL_FRONT_ANGLE_mm; // mm
					ctx.w_wall_front_current = wall_sensor_get_dist(WALL_SENSOR_LEFT_STRAIGHT) - wall_sensor_get_dist(WALL_SENSOR_RIGHT_STRAIGHT);
					ctx.w_wall_front_error = ctx.w_wall_front_setpoint - ctx.w_wall_front_current;
					ctx.w_wall_front_pwm = pid_output(&ctx.w_wall_front_pid, ctx.w_wall_front_error);

					motor_speed_left(-ctx.x_wall_front_pwm - ctx.w_wall_front_pwm);
					motor_speed_right(-ctx.x_wall_front_pwm + ctx.w_wall_front_pwm);

					if(HAL_GetTick() > ctx.action_time + 1000)
					{
						ctx.sub_action_index++;
						ctx.action_time = HAL_GetTick();
						encoder_reset();
						pid_reset(&ctx.x_speed_pid);
						pid_reset(&ctx.w_speed_pid);
						pid_reset(&ctx.x_wall_front_pid);
						pid_reset(&ctx.w_wall_front_pid);
					}
				}
				break;

				// TURN 90° (ACC+CRUISE)
				case 3 :
				{
					// forward speed
					ctx.x_speed_target = 0;
					ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
					ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
					ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
					ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

					// rotation speed
					ctx.w_speed_target = -W_SPEED;
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
				// TURN 90° (DECC)
				case 4 :
					{
						// forward speed
						ctx.x_speed_target = 0.0;
						ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
						ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
						ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
						ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);


						// rotation speed
						ctx.w_speed_target = 0.0;
						ctx.w_speed_setpoint = next_speed(ctx.w_speed_target, W_MAX_ACCELERATION, W_MAX_DECELERATION, 0.001, ctx.w_speed_setpoint);
						ctx.w_speed_current = gyro_get_dps();
						ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
						ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

						motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
						motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

						if(HAL_GetTick() > ctx.action_time + W_T2)
						{
							ctx.sub_action_index++;
							ctx.action_time = HAL_GetTick();
							encoder_reset();
							pid_reset(&ctx.x_speed_pid);
							pid_reset(&ctx.w_speed_pid);
							pid_reset(&ctx.x_wall_front_pid);
							pid_reset(&ctx.w_wall_front_pid);
						}
					}
					break;
				//FRONT WALL CALIBRATE
				case 5 :
				{
					// straight calibration
					// wall position
					ctx.x_wall_front_target = WALL_FRONT_DISTANCE_mm; // mm
					ctx.x_wall_front_setpoint = WALL_FRONT_DISTANCE_mm; // mm
					ctx.x_wall_front_current = ( wall_sensor_get_dist(WALL_SENSOR_LEFT_STRAIGHT) + wall_sensor_get_dist(WALL_SENSOR_RIGHT_STRAIGHT) ) / 2.0;
					ctx.x_wall_front_error = ctx.x_wall_front_setpoint - ctx.x_wall_front_current;
					ctx.x_wall_front_pwm = pid_output(&ctx.x_wall_front_pid, ctx.x_wall_front_error);

					// rotation calibration
					// wall position
					ctx.w_wall_front_target = WALL_FRONT_ANGLE_mm;
					ctx.w_wall_front_setpoint = WALL_FRONT_ANGLE_mm;
					ctx.w_wall_front_current =wall_sensor_get_dist(WALL_SENSOR_LEFT_STRAIGHT) - wall_sensor_get_dist(WALL_SENSOR_RIGHT_STRAIGHT);
					ctx.w_wall_front_error = ctx.w_wall_front_setpoint - ctx.w_wall_front_current;
					ctx.w_wall_front_pwm = pid_output(&ctx.w_wall_front_pid, ctx.w_wall_front_error);

					motor_speed_left(-ctx.x_wall_front_pwm - ctx.w_wall_front_pwm);
					motor_speed_right(-ctx.x_wall_front_pwm + ctx.w_wall_front_pwm);

					if(HAL_GetTick() > ctx.action_time + 1000)
					{
						ctx.sub_action_index++;
						ctx.action_time = HAL_GetTick();
						encoder_reset();
						pid_reset(&ctx.x_speed_pid);
						pid_reset(&ctx.w_speed_pid);
						pid_reset(&ctx.x_wall_front_pid);
						pid_reset(&ctx.w_wall_front_pid);
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
					ctx.w_speed_target = -W_SPEED;
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
				// TURN 90° (DECC)
				case 7 :
					{
						// forward speed
						ctx.x_speed_target = 0.0;
						ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
						ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
						ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
						ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

						// rotation speed
						ctx.w_speed_target = 0.0;
						ctx.w_speed_setpoint = next_speed(ctx.w_speed_target, W_MAX_ACCELERATION, W_MAX_DECELERATION, 0.001, ctx.w_speed_setpoint);
						ctx.w_speed_current = gyro_get_dps();
						ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
						ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

						motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
						motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

						if(HAL_GetTick() > ctx.action_time + W_T2)
						{
							ctx.sub_action_index++;
							ctx.action_time = HAL_GetTick();
						}
					}
					break;
					// PAUSE
				case 8:
					{
						// forward speed
						ctx.x_speed_target = 0.0;
						ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
						ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
						ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
						ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

						// rotation speed
						ctx.w_speed_target = 0.0;
						ctx.w_speed_setpoint = next_speed(ctx.w_speed_target, W_MAX_ACCELERATION, W_MAX_DECELERATION, 0.001, ctx.w_speed_setpoint);
						ctx.w_speed_current = gyro_get_dps();
						ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
						ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

						motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
						motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

						if(HAL_GetTick() > ctx.action_time + 500)
						{
							ctx.sub_action_index++;
							ctx.action_time = HAL_GetTick();
							encoder_reset();
						}
					}
					break;

				//START
				case 9 :
				{
						// forward speed
						ctx.x_speed_target = X_SPEED;
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

						if(encoder_get_absolute() >= DIST_START)
						{
#ifdef FIXED_MOVES
							ctx.current_state = get_next_move();
#else
							ctx.current_state = update_maze_ctx(&ctx.maze);
#endif
							ctx.sub_action_index = 0;
							ctx.action_time = HAL_GetTick();
							ctx.current_pid_type = PID_TYPE_GYRO;

							encoder_set_absolute(encoder_get_absolute() - DIST_START);
							calibration_reset();
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
		switch (ctx.sub_action_index)
		{
			// RUN
			case 0 :
			{

				// forward speed
				ctx.x_speed_target = X_SPEED;
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

			// BRAKE
			case 1 :
			{
				// forward speed
				ctx.x_speed_target = 0.1;
				ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
				ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
				ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
				ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

				// rotation speed
				ctx.w_speed_target = 0.0;
				ctx.w_speed_setpoint = 0.0;
				ctx.w_speed_current = gyro_get_dps();
				ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
				ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

				motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
				motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

				if(encoder_get_absolute() >= DIST_STOP)
				{
					ctx.sub_action_index++;
					ctx.action_time = HAL_GetTick();
				}

			}
			break;

			// STILL
			case 2 :
			{
				// forward speed
				ctx.x_speed_target = 0.0;
				ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
				ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
				ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
				ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

				// rotation speed
				ctx.w_speed_target = 0.0;
				ctx.w_speed_setpoint = 0.0;
				ctx.w_speed_current = gyro_get_dps();
				ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
				ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

				motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
				motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

				if(HAL_GetTick() > ctx.action_time + 1000)
				{
#ifdef FIXED_MOVES
					ctx.current_state = get_next_move();
#else
					ctx.current_state = ACTION_IDLE ;
#endif
					ctx.sub_action_index = 0;
					ctx.action_time = HAL_GetTick();
					ctx.current_pid_type = PID_TYPE_GYRO;

					encoder_reset();
					calibration_reset();
					pid_reset(&ctx.x_speed_pid);
					pid_reset(&ctx.w_speed_pid);
					pid_reset(&ctx.wall_position_pid);

					motor_speed_left(0);
					motor_speed_right(0);

					led_toggle();

#ifdef FIXED_MOVES

#else
					if(ctx.maze.mode == SOLVE)
					{
						display_maze_ctx(&ctx.maze);
					}
#endif
				}

			}
			break;
		}
	}
	break;

	case ACTION_STILL:
	{
		// STILL
		switch (ctx.sub_action_index)
		{
			// RUN
			case 0 :
			{
				// forward speed
				ctx.x_speed_target = 0.0;
				ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
				ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
				ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
				ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

				// rotation speed
				ctx.w_speed_target = 0.0;
				ctx.w_speed_setpoint = 0.0;
				ctx.w_speed_current = gyro_get_dps();
				ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
				ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

				motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
				motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

				ctx.action_time = HAL_GetTick();
				++ctx.sub_action_index;
			}
			break;

			case 1 :
			{
				// forward speed
				ctx.x_speed_target = 0.0;
				ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
				ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
				ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
				ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

				// rotation speed
				ctx.w_speed_target = 0.0;
				ctx.w_speed_setpoint = 0.0;
				ctx.w_speed_current = gyro_get_dps();
				ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
				ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);

				motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
				motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);

				if(HAL_GetTick() > ctx.action_time + 1000)
				{
					ctx.current_state = get_next_move();
					ctx.sub_action_index = 0;
					ctx.action_time = HAL_GetTick();
					ctx.current_pid_type = PID_TYPE_GYRO;

					encoder_reset();
					calibration_reset();
					pid_reset(&ctx.x_speed_pid);
					pid_reset(&ctx.w_speed_pid);
					pid_reset(&ctx.wall_position_pid);

					motor_speed_left(0);
					motor_speed_right(0);

					led_toggle();

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

/// wall calibration helper
void calibration_reset()
{
	ctx.calibration_state = CALIBRATION_IDLE;
	filter_reset(&ctx.filter_calibration_wall_distance_left);
	filter_reset(&ctx.filter_calibration_wall_distance_right);
}

bool calibration_update()
{
	float calibration_wall_distance_left = filter_output(&ctx.filter_calibration_wall_distance_left,wall_sensor_get_dist(WALL_SENSOR_LEFT_DIAG));
	float calibration_wall_distance_right = filter_output(&ctx.filter_calibration_wall_distance_right,wall_sensor_get_dist(WALL_SENSOR_RIGHT_DIAG));
	switch(ctx.calibration_state)
	{
	case CALIBRATION_IDLE:
		{
			if(encoder_get_absolute()>0.03f) // skip first 3 centimeter of current cell (position error)
			{
				if(calibration_wall_distance_left<120 && calibration_wall_distance_right<120)
				{
					ctx.calibration_state = CALIBRATION_WALL_BOTH;
				}
				else if(calibration_wall_distance_left<120)
				{
					ctx.calibration_state = CALIBRATION_WALL_LEFT;
				}
				else if(calibration_wall_distance_right<120)
				{
					ctx.calibration_state = CALIBRATION_WALL_RIGHT;
				}
				else
				{
					ctx.calibration_state = CALIBRATION_NO_WALL;
				}
			}
		}
		break;
	case CALIBRATION_NO_WALL:
		{
			if(calibration_wall_distance_left<140 && calibration_wall_distance_right<140)
			{
				ctx.calibration_state = CALIBRATION_POST_BOTH;
			}
			else if(calibration_wall_distance_left<140)
			{
				ctx.calibration_state = CALIBRATION_POST_LEFT;
			}
			else if(calibration_wall_distance_right<140)
			{
				ctx.calibration_state = CALIBRATION_POST_RIGHT;
			}
		}
		break;
	case CALIBRATION_POST_LEFT:
		{
			if(calibration_wall_distance_left>140)
			{
				encoder_set_absolute(REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}
			if(calibration_wall_distance_right<140)
			{
				ctx.calibration_state = CALIBRATION_POST_BOTH;
			}
		}
		break;
	case CALIBRATION_POST_RIGHT:
		{
			if(calibration_wall_distance_right>140)
			{
				encoder_set_absolute(REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}
			if(calibration_wall_distance_left<140)
			{
				ctx.calibration_state = CALIBRATION_POST_BOTH;
			}
		}
		break;
	case CALIBRATION_POST_BOTH:
		{
			if(calibration_wall_distance_right>140 || calibration_wall_distance_left>140)
			{
				encoder_set_absolute(REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}
		}
		break;
	case CALIBRATION_WALL_LEFT:
		{
			if(calibration_wall_distance_left>140)
			{
				encoder_set_absolute(REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}
			if(calibration_wall_distance_right<140)
			{
				ctx.calibration_state = CALIBRATION_WALL_LEFT_with_RIGHT_POST;
			}
		}
		break;
	case CALIBRATION_WALL_RIGHT:
		{
			if(calibration_wall_distance_right>140)
			{
				encoder_set_absolute(REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}
			if(calibration_wall_distance_left<140)
			{
				ctx.calibration_state = CALIBRATION_WALL_RIGHT_with_LEFT_POST;
			}
		}
		break;
	case CALIBRATION_WALL_BOTH:
		{
			if(calibration_wall_distance_right>140 || calibration_wall_distance_left>140)
			{
				encoder_set_absolute(REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}
		}
		break;
	case CALIBRATION_WALL_LEFT_with_RIGHT_POST:
		{
			if(calibration_wall_distance_left>140)
			{
				encoder_set_absolute(REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}
			if(calibration_wall_distance_right>140)
			{
				encoder_set_absolute(REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}

		}
		break;
	case CALIBRATION_WALL_RIGHT_with_LEFT_POST:
		{
			if(calibration_wall_distance_right>140)
			{
				encoder_set_absolute(REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}
			if(calibration_wall_distance_left>140)
			{
				encoder_set_absolute(REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL);
				ctx.calibration_state = CALIBRATION_END;
				return true;
			}

		}
		break;

	case CALIBRATION_END:
		{

		}
		break;
	}
	return false;
}
