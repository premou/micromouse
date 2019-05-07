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
#include "personnalisation.h"

#include <math.h>
#include <time.h>
#include <stdlib.h>

// globals
extern HAL_Serial_Handler com;

////////////
// constants
////////////

// period
#define CONTROLLER_PERIOD 1200U // us microseconds (= 833Hz ODR GYRO)
#define CONTROLLER_FREQ_F 833.0F // us microseconds (= 833Hz ODR GYRO)
#define CONTROLLER_PERDIO_F 0.0012F // us microseconds (= 833Hz ODR GYRO)

// distance
#define DIST_START 0.09 			// m
#define DIST_RUN_1 0.18 			// m
#define DIST_STOP 0.09 				// m

// ENUM
typedef enum {
	PID_TYPE_GYRO,
	PID_TYPE_WALL,
	PID_TYPE_CTR
} pid_type_t;

typedef enum {
	UTURN_NO_WALL,
	UTURN_FRONT_WALL,
	UTURN_FRONT_RIGHT_WALLS,
	UTURN_RIGHT_WALL,
	UTURN_FRONT_LEFT_WALLS,
	UTURN_LEFT_WALL
} uturn_type_t;

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
	uturn_type_t current_uturn_type;

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

///////////////////
// HELPER FUNCTIONS
///////////////////

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

#include "controller_helper.h"

///////////////////
// PUBLIC FUNCTIONS
///////////////////

uint32_t controller_init () // return GYRO ERROR (ZERO is GYRO OK)
{
	srand(gyro_get_dps());

	// reset controller fsm
	ctx.time_us = 0;
	ctx.current_state = ACTION_START;
	ctx.actions_index = 0;
	ctx.action_time = 0;
	ctx.sub_action_index = 0;
	ctx.gyro_state = gyro_init();
	ctx.current_pid_type = PID_TYPE_GYRO;
	ctx.current_uturn_type = UTURN_NO_WALL;

	// forward speed PID
	ctx.x_speed_target = 0;
	ctx.x_speed_current = 0;
	ctx.x_speed_setpoint = 0;
	ctx.x_speed_error = 0;
	ctx.x_speed_pwm = 0;
	pid_init(&ctx.x_speed_pid, X_SPEED_KP, X_SPEED_KI, X_SPEED_KD, 0.5);

	// rotation speed PID
	ctx.w_speed_target = 0;
	ctx.w_speed_current = 0;
	ctx.w_speed_setpoint = 0;
	ctx.w_speed_error = 0;
	ctx.w_speed_pwm = 0;
	pid_init(&ctx.w_speed_pid, W_SPEED_KP, W_SPEED_KI, W_SPEED_KD, 0.5);

	// wall following position PID
	ctx.wall_position_target = 0;
	ctx.wall_position_setpoint = 0;
	ctx.wall_position_current = 0;
	ctx.wall_position_error = 0;
	ctx.wall_position_pwm = 0;
	pid_init(&ctx.wall_position_pid, WALL_POSITION_KP, WALL_POSITION_KI, WALL_POSITION_KD, 0.9);

	// front wall distance position PID
	ctx.x_wall_front_target = 0;
	ctx.x_wall_front_setpoint = 0;
	ctx.x_wall_front_current = 0;
	ctx.x_wall_front_error = 0;
	ctx.x_wall_front_pwm = 0;
	pid_init(&ctx.x_wall_front_pid, X_WALL_FRONT_KP, X_WALL_FRONT_KI, X_WALL_FRONT_KD, 0.9);

	// front wall angle position PID
	ctx.w_wall_front_target = 0;
	ctx.w_wall_front_setpoint = 0;
	ctx.w_wall_front_current = 0;
	ctx.w_wall_front_error = 0;
	ctx.w_wall_front_pwm = 0;
	pid_init(&ctx.w_wall_front_pid, W_WALL_FRONT_KP, W_WALL_FRONT_KI, W_WALL_FRONT_KD, 0.9);

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

	HAL_DataLogger_Init(18, // number of fields
			1,  // size in bytes of each field
			4, 	// size in bytes of each field
			4, 	// size in bytes of each field

			4, 	// size in bytes of each field
			4, 	// size in bytes of each field

			4, 	// size in bytes of each field
			4, 	// size in bytes of each field

			1, 	// size in bytes of each field
			4, 	// size in bytes of each field

			4, 	// size in bytes of each field
			4, 	// size in bytes of each field

			4, 	// size in bytes of each field
			4, 	// size in bytes of each field

			1, 	// size in bytes of each field
			1, 	// size in bytes of each field
			1, 	// size in bytes of each field
			1, 	// size in bytes of each field

			1 	// size in bytes of each field
	);

	return ctx.gyro_state;
}

void controller_start()
{
	// reset controller fsm
	ctx.time_us = 0;
	ctx.current_state = ACTION_START;
	ctx.actions_index = 0;
	ctx.action_time = HAL_GetTick();
	ctx.sub_action_index = 0;
	ctx.current_pid_type = PID_TYPE_GYRO;
	ctx.current_uturn_type = UTURN_NO_WALL;

	// reset all PID and setpoint
	reset_speed_control();

	// reset longitudinal calibration (wall-to-no-wall, post-to-no-post)
	calibration_reset();

	// reset encoder
	encoder_reset();

	// AI
	maze_ctx_start(&ctx.maze) ;


	HAL_DataLogger_Clear();
}

void controller_stop()
{
	// reset controller fsm
	ctx.time_us = 0;
	ctx.current_state = ACTION_START;
	ctx.actions_index = 0;
	ctx.action_time = HAL_GetTick();
	ctx.sub_action_index = 0;
	ctx.current_pid_type = PID_TYPE_GYRO;
	ctx.current_uturn_type = UTURN_NO_WALL;

	// reset all PID and setpoint
	reset_speed_control();

	// reset longitudinal calibration (wall-to-no-wall, post-to-no-post)
	calibration_reset();

	// reset encoder
	encoder_reset();

	// AI
	// nop
}

void controller_fsm(); // forward declaration

void controller_update(){
	// controller period
	uint16_t time_us_current = timer_us_get();
	if( (time_us_current-ctx.time_us) >= CONTROLLER_PERIOD ) // wait for PERIOD, then update sensors and call controller fsm
	{
		ctx.time_us = time_us_current;

		// sensor update
		encoder_update();
		gyro_update(CONTROLLER_PERDIO_F);
		wall_sensor_update();

		// motor control update
		controller_fsm();

		// data logger
		HAL_DataLogger_Record(18, 						 // number of fields
				(int32_t)(ctx.actions_index), 				 // integer value of each field
				//(int32_t)(ctx.sub_action_index),		 // integer value of each field
				(int32_t)(encoder_get_absolute()*1000.0),		 // integer value of each field
				(int32_t)(gyro_get_heading()),

				(int32_t)(ctx.x_speed_setpoint * 1000.0),// setpoint speed
				(int32_t)(ctx.x_speed_current* 1000.0),	 // current speed
				//(int32_t)(ctx.x_speed_pid.err_filtered* 1000.0),	 // current speed
				//(int32_t)(ctx.x_speed_pwm * 10.0),	 // current speed

				(int32_t)(ctx.w_speed_setpoint * 1.0),// setpoint speed
				(int32_t)(ctx.w_speed_current* 1.0),	 // current speed
				//(int32_t)(ctx.w_speed_pid.err_filtered* 1.0),	 // current speed
				//(int32_t)(ctx.w_speed_pwm * 10.0),	 // current speed

				(int32_t)(ctx.current_pid_type * 1.0),// setpoint speed
				(int32_t)(ctx.wall_position_current* 1000.0),	 // current speed

				(int32_t)(ctx.x_wall_front_setpoint * 1.0),// setpoint speed
				(int32_t)(ctx.x_wall_front_current* 1.0),	 // current speed

				(int32_t)(ctx.w_wall_front_setpoint * 1.0F),// integer value of each field
				(int32_t)(ctx.w_wall_front_current * 1.0F),	 // integer value of each field

				  (int32_t)wall_sensor_get_dist(WALL_SENSOR_LEFT_DIAG),
				  (int32_t)wall_sensor_get_dist(WALL_SENSOR_LEFT_STRAIGHT),
				  (int32_t)wall_sensor_get_dist(WALL_SENSOR_RIGHT_STRAIGHT),
				  (int32_t)wall_sensor_get_dist(WALL_SENSOR_RIGHT_DIAG),

				  ((int32_t)wall_sensor_is_left_wall_detected()<<0) |
				  ((int32_t)wall_sensor_is_front_wall_detected()<<1) |
				  ((int32_t)wall_sensor_is_right_wall_detected()<<2)
		);
	}
}

bool controller_is_end(){
	return ctx.current_state == ACTION_IDLE;
}

///////////////////
// PRIVATE FUNCTIONS
///////////////////

void controller_fsm()
{
	switch(ctx.current_state)
	{
	case ACTION_IDLE :
	case ACTION_RAND :
	{
		// keep position
		speed_control(0.0F,0.0F);
	}
	break;

	case ACTION_START :
	{
		// move forward (acc+cruise)
		speed_control(X_SPEED,0.0F);

		// transition / condition
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

			led_toggle();
		}
	}
	break;

	case ACTION_RUN_1 :
	{

		// forward speed
		float x_speed_target = X_SPEED;
		// use fast speed if possible
#ifdef FIXED_MOVES
		if( actions_scenario[ctx.actions_index] ==  ACTION_RUN_1 )
#else
		if (get_next_next_action(&ctx.maze) == ACTION_RUN_1)
#endif
		{
			x_speed_target = X_SPEED_FAST_RUN;
		}

		// move forward and use side walls
		// middle cell alignement (abandonned)
		//if( (encoder_get_absolute() <= DIST_RUN_1/2.0) && (wall_sensor_is_left_wall_detected() || wall_sensor_is_right_wall_detected()) )
		// wall collision avoidance
		if( wall_sensor_get_side_error() != 0.0F )
		{
			ctx.current_pid_type = PID_TYPE_WALL;
			speed_control_with_wall_following(x_speed_target);
		}
		else
		{
			ctx.current_pid_type = PID_TYPE_GYRO;
			speed_control(x_speed_target,0.0F);
		}
		calibration_update();

		// transition / condition
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

			led_toggle();
		}
	}
	break;

	case ACTION_TURN_RIGHT :
	case ACTION_TURN_LEFT :
	{
		switch (ctx.sub_action_index)
		{
		// FORWARD 2 cm
		case 0:
			{
				// move forward (acc+cruise)
				speed_control(X_SPEED,0.0F);

				// transition / condition
				if( wall_sensor_is_front_wall_detected() )
				{
					if( (wall_sensor_get_dist(WALL_SENSOR_LEFT_STRAIGHT)+wall_sensor_get_dist(WALL_SENSOR_RIGHT_STRAIGHT)) <= WALL_FRONT_ANGLE_TURNING_mm )
					{
						ctx.sub_action_index++;
						ctx.action_time = HAL_GetTick();
						ctx.current_pid_type = PID_TYPE_GYRO;

						encoder_reset();
						calibration_reset();

						led_toggle();
					}
				}
				else if(encoder_get_absolute() >= 0.02)
				{
					ctx.sub_action_index++;
					ctx.action_time = HAL_GetTick();
					ctx.current_pid_type = PID_TYPE_GYRO;

					encoder_reset();
					calibration_reset();

					led_toggle();
				}
			}
			break;
		//ACCELERATION
		case 1 :
			// curve speed
			if(ctx.current_state == ACTION_TURN_RIGHT)
			{
				speed_control(X_SPEED,-W_SPEED);
			}
			else
			{
				speed_control(X_SPEED,W_SPEED);
			}

			// transition / condition
			if(HAL_GetTick() > ctx.action_time + W_T1)
			{
				ctx.sub_action_index++;
			}
			break;
		//DECELERATION
		case 2 :
			// curve speed
			speed_control(X_SPEED,0.0F);

			// transition / condition
			if(HAL_GetTick() > ctx.action_time + W_T2)
			{
				++ctx.sub_action_index;
				ctx.action_time = HAL_GetTick();
				ctx.current_pid_type = PID_TYPE_GYRO;

				encoder_reset();
				calibration_reset();

				led_toggle();
			}
			break;
		// FORWARD 2 cm
		case 3 :
			{
				// move forward (acc+cruise)
				speed_control(X_SPEED,0.0F);

				// transition / condition
				if(encoder_get_absolute() >= 0.02)
				{
#ifdef FIXED_MOVES
			ctx.current_state = get_next_move();
#else
			ctx.current_state = update_maze_ctx(&ctx.maze);
#endif
					ctx.sub_action_index = 0;
					ctx.action_time = HAL_GetTick();
					ctx.current_pid_type = PID_TYPE_GYRO;

					encoder_set_absolute(encoder_get_absolute() - 0.02F);
					calibration_reset();

					led_toggle();
				}
			}
			break;
		default:
			break;
		}
	}
	break;

	case ACTION_U_TURN_RIGHT :
	{
		switch (ctx.sub_action_index)
		{
			// STEP1 : RUN until BRAKE
			case 0 :
			{
				// check wall and decide wich kind of dead end turn to do
				if( wall_sensor_is_front_wall_detected() &&
						wall_sensor_is_right_wall_detected() ) 	// 111 + 011
				{
					ctx.current_uturn_type = UTURN_FRONT_RIGHT_WALLS;
				}
				else if( wall_sensor_is_front_wall_detected() &&
						wall_sensor_is_left_wall_detected() ) 	// 110
				{
					ctx.current_uturn_type = UTURN_FRONT_LEFT_WALLS;
				}
				else if(wall_sensor_is_front_wall_detected() ) 	// 010
				{
					ctx.current_uturn_type = UTURN_FRONT_WALL;
				}
				else if( wall_sensor_is_right_wall_detected() ) // 001 + 101
				{
					ctx.current_uturn_type = UTURN_RIGHT_WALL;
				}
				else if( wall_sensor_is_left_wall_detected() ) // 100
				{
					ctx.current_uturn_type = UTURN_LEFT_WALL;
				}
				else // 000
				{
					ctx.current_uturn_type = UTURN_NO_WALL;
				}

				// move forward and use side walls
				if( (encoder_get_absolute() <= DIST_RUN_1/2.0) && (wall_sensor_is_left_wall_detected() || wall_sensor_is_right_wall_detected()) )
				{
					ctx.current_pid_type = PID_TYPE_WALL;
					speed_control_with_wall_following(X_SPEED);
				}
				else
				{
					ctx.current_pid_type = PID_TYPE_GYRO;
					speed_control(X_SPEED,0.0F);
				}

				// transition / condition
				if(have_to_break(0, ctx.x_speed_setpoint, DIST_STOP-encoder_get_absolute(), X_MAX_DECELERATION))
				{
					ctx.sub_action_index++;
					ctx.action_time = HAL_GetTick();
					ctx.current_pid_type = PID_TYPE_GYRO;
					HAL_Serial_Print(&com," STEP1->2, type=%d\r\n",ctx.current_uturn_type);
				}
			}
			break;
			// STEP1 : BRAKE
			case 1 :
			{
				switch(ctx.current_uturn_type)
				{
					case UTURN_NO_WALL:
					case UTURN_RIGHT_WALL:
					case UTURN_LEFT_WALL:
					{
						// brake and use side walls
						if( (encoder_get_absolute() <= DIST_RUN_1/2.0) && (wall_sensor_is_left_wall_detected() || wall_sensor_is_right_wall_detected()) )
						{
							ctx.current_pid_type = PID_TYPE_WALL;
							speed_control_with_wall_following(0.05);
						}
						else
						{
							ctx.current_pid_type = PID_TYPE_GYRO;
							speed_control(0.05F,0.0F);
						}

						// transition / condition
						if(encoder_get_absolute() >= DIST_STOP)
						{
							ctx.sub_action_index++;
							ctx.action_time = HAL_GetTick();
							ctx.current_pid_type = PID_TYPE_GYRO;
						}
					}
					break;
					case UTURN_FRONT_WALL:
					case UTURN_FRONT_RIGHT_WALLS:
					case UTURN_FRONT_LEFT_WALLS:
					{
						// brake (keep a little speed) and use side walls
						if( (encoder_get_absolute() <= DIST_RUN_1/2.0) && (wall_sensor_is_left_wall_detected() || wall_sensor_is_right_wall_detected()) )
						{
							ctx.current_pid_type = PID_TYPE_WALL;
							speed_control_with_wall_following(0.1F);
						}
						else
						{
							ctx.current_pid_type = PID_TYPE_GYRO;
							speed_control(0.1F,0.0F);
						}

						// transition / condition
						if(encoder_get_absolute() >= DIST_STOP)
						{
							ctx.sub_action_index++;
							ctx.action_time = HAL_GetTick();
							ctx.current_pid_type = PID_TYPE_GYRO;
						}
					}
					break;
				}
			}
			break;
			// STEP2 : FRONT WALL CALIBRATION or just STILL for a little time
			case 2 :
			{
				switch(ctx.current_uturn_type)
				{
					case UTURN_NO_WALL:
					case UTURN_RIGHT_WALL:
					case UTURN_LEFT_WALL:
					{
						// keep position
						speed_control(0.0F,0.0F);
						// transition / condition
						if(HAL_GetTick() > ctx.action_time + 250)
						{
							ctx.sub_action_index++;
							ctx.action_time = HAL_GetTick();
							ctx.current_pid_type = PID_TYPE_GYRO;

							encoder_reset();
						}
					}
					break;
					case UTURN_FRONT_WALL:
					case UTURN_FRONT_RIGHT_WALLS:
					case UTURN_FRONT_LEFT_WALLS:
					{
						// front wall calibration
						speed_control_with_front_wall_calibration();
						// transition / condition
						if(HAL_GetTick() > ctx.action_time + 1000)
						{
							ctx.sub_action_index++;
							ctx.action_time = HAL_GetTick();
							ctx.current_pid_type = PID_TYPE_GYRO;

							encoder_reset();
						}
					}
					break;
				}
			}
			break;
			// STEP3 : TURN (accelerate+cruise)
			case 3 :
			{
				switch(ctx.current_uturn_type)
				{
					case UTURN_NO_WALL:
					case UTURN_FRONT_WALL:
					{
						// curve speed
						speed_control(0.0F,-W_SPEED);
						// transition / condition
						if(HAL_GetTick() > ctx.action_time + W_U_T1_180)
						{
							ctx.sub_action_index++;
						}
					}
					break;
					case UTURN_RIGHT_WALL:
					case UTURN_LEFT_WALL:
					case UTURN_FRONT_RIGHT_WALLS:
					case UTURN_FRONT_LEFT_WALLS:
					{
						// curve speed
						if( (ctx.current_uturn_type == UTURN_RIGHT_WALL) ||
							(ctx.current_uturn_type == UTURN_FRONT_RIGHT_WALLS) )
						{
							speed_control(0.0F,-W_SPEED);
						}
						else
						{
							speed_control(0.0F,W_SPEED);
						}
						// transition / condition
						if(HAL_GetTick() > ctx.action_time + W_U_T1_90)
						{
							ctx.sub_action_index++;
						}
					}
					break;
				}
			}
			// STEP4 : TURN (decelerate)
			case 4 :
			{
				switch(ctx.current_uturn_type)
				{
					case UTURN_NO_WALL:
					case UTURN_FRONT_WALL:
					{
						// curve speed
						speed_control(0.0F,0.0F);
						// transition / condition
						if(HAL_GetTick() > ctx.action_time + W_U_T2_180)
						{
							ctx.sub_action_index++;
							ctx.action_time = HAL_GetTick();
							ctx.current_pid_type = PID_TYPE_GYRO;
							encoder_reset();
						}
					}
					break;
					case UTURN_RIGHT_WALL:
					case UTURN_LEFT_WALL:
					case UTURN_FRONT_RIGHT_WALLS:
					case UTURN_FRONT_LEFT_WALLS:
					{
						// curve speed
						speed_control(0.0F,0.0F);
						// transition / condition
						if(HAL_GetTick() > ctx.action_time + W_U_T2_90)
						{
							ctx.sub_action_index++;
							ctx.action_time = HAL_GetTick();
							ctx.current_pid_type = PID_TYPE_GYRO;
							encoder_reset();
						}
					}
					break;
				}
			}
			break;
			// STEP5 : FRONT WALL CALIBRATION or just STILL for a little time
			case 5 :
			{
				switch(ctx.current_uturn_type)
				{
					case UTURN_NO_WALL:
					case UTURN_FRONT_WALL:
					{
						// keep position
						speed_control(0.0F,0.0F);
						// transition / condition
						if(HAL_GetTick() > ctx.action_time + 50)
						{
							ctx.sub_action_index++;
							ctx.action_time = HAL_GetTick();
							ctx.current_pid_type = PID_TYPE_GYRO;
							encoder_reset();
						}
					}
					break;
					case UTURN_FRONT_RIGHT_WALLS:
					case UTURN_FRONT_LEFT_WALLS:
					case UTURN_RIGHT_WALL:
					case UTURN_LEFT_WALL:
					{
						// front wall calibration
						speed_control_with_front_wall_calibration();
						// transition / condition
						if(HAL_GetTick() > ctx.action_time + 1000)
						{
							ctx.sub_action_index++;
							ctx.action_time = HAL_GetTick();
							ctx.current_pid_type = PID_TYPE_GYRO;
							encoder_reset();
						}
					}
					break;
				}

			}
			break;
			// STEP6 : TURN (accelerate+cruise) or just STILL for a little time
			case 6 :
			{
				switch(ctx.current_uturn_type)
				{
					case UTURN_NO_WALL:
					case UTURN_FRONT_WALL:
					{
						// keep position
						speed_control(0.0F,0.0F);
						// transition / condition
						if(HAL_GetTick() > ctx.action_time + 50)
						{
							ctx.sub_action_index++;
							ctx.action_time = HAL_GetTick();
							ctx.current_pid_type = PID_TYPE_GYRO;

							encoder_reset();
						}
					}
					break;
					case UTURN_FRONT_RIGHT_WALLS:
					case UTURN_FRONT_LEFT_WALLS:
					case UTURN_RIGHT_WALL:
					case UTURN_LEFT_WALL:
					{
						// curve speed
						if( (ctx.current_uturn_type == UTURN_RIGHT_WALL) ||
							(ctx.current_uturn_type == UTURN_FRONT_RIGHT_WALLS) )
						{
							speed_control(0.0F,-W_SPEED);
						}
						else
						{
							speed_control(0.0F,W_SPEED);
						}
						// transition / condition
						if(HAL_GetTick() > ctx.action_time + W_U_T1_90)
						{
							ctx.sub_action_index++;
						}
					}
					break;
				}
			}
			break;
			// STEP7 : TURN (decelerate) or just STILL for a little time
			case 7 :
			{
				switch(ctx.current_uturn_type)
				{
					case UTURN_NO_WALL:
					case UTURN_FRONT_WALL:
					{
						// keep position
						speed_control(0.0F,0.0F);
						// transition / condition
						if(HAL_GetTick() > ctx.action_time + 50)
						{
							ctx.sub_action_index++;
							ctx.action_time = HAL_GetTick();
							ctx.current_pid_type = PID_TYPE_GYRO;
							encoder_reset();
						}
					}
					break;
					case UTURN_FRONT_RIGHT_WALLS:
					case UTURN_FRONT_LEFT_WALLS:
					case UTURN_RIGHT_WALL:
					case UTURN_LEFT_WALL:
					{
						// curve speed
						speed_control(0.0F,0.0F);
						// transition / condition
						if(HAL_GetTick() > ctx.action_time + W_U_T2_90)
						{
							ctx.sub_action_index++;
							ctx.action_time = HAL_GetTick();
							ctx.current_pid_type = PID_TYPE_GYRO;
							encoder_reset();
						}
					}
					break;
				}
			}
			break;
			// STEP8 : STILL for a little time
			case 8 :
			{
				// keep position
				speed_control(0.0F,0.0F);
				// transition / condition
				if(HAL_GetTick() > ctx.action_time + 500)
				{
					ctx.sub_action_index++;
					ctx.action_time = HAL_GetTick();
					ctx.current_pid_type = PID_TYPE_GYRO;
					encoder_reset();
				}
			}
			break;
			//START
			case 9 :
			{
				// move forward (acc+cruise)
				speed_control(X_SPEED,0.0F);

				// transition / condition
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
				// move forward and use side walls
				if( (encoder_get_absolute() <= DIST_RUN_1/2.0) && (wall_sensor_is_left_wall_detected() || wall_sensor_is_right_wall_detected()) )
				{
					ctx.current_pid_type = PID_TYPE_WALL;
					speed_control_with_wall_following(X_SPEED);
				}
				else
				{
					ctx.current_pid_type = PID_TYPE_GYRO;
					speed_control(X_SPEED,0.0F);
				}

				// transition / condition
				if(have_to_break(0, ctx.x_speed_setpoint, DIST_STOP-encoder_get_absolute(), X_MAX_DECELERATION))
				{
					ctx.sub_action_index++;
					ctx.action_time = HAL_GetTick();
					ctx.current_pid_type = PID_TYPE_GYRO;
				}
			}
			break;
			// BRAKE
			case 1 :
			{
				// brake and use side walls
				if( (encoder_get_absolute() <= DIST_RUN_1/2.0) && (wall_sensor_is_left_wall_detected() || wall_sensor_is_right_wall_detected()) )
				{
					ctx.current_pid_type = PID_TYPE_WALL;
					speed_control_with_wall_following(0.05F);
				}
				else
				{
					ctx.current_pid_type = PID_TYPE_GYRO;
					speed_control(0.05F,0.0F);
				}

				// transition / condition
				if(encoder_get_absolute() >= DIST_STOP)
				{
					ctx.sub_action_index++;
					ctx.action_time = HAL_GetTick();
					ctx.current_pid_type = PID_TYPE_GYRO;
				}
			}
			break;

			// STILL
			case 2 :
			{
				// keep position
				speed_control(0.0F,0.0F);

				// transition / condition
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
					reset_speed_control();

					led_toggle();

#ifdef FIXED_MOVES

#else
					if(ctx.maze.mode == SOLVE)
					{
						// For debug purpose
						//display_maze_ctx(&ctx.maze);
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
				// keep position
				speed_control(0.0F,0.0F);
				// transition
				++ctx.sub_action_index;
				ctx.action_time = HAL_GetTick();

			}
			break;

			case 1 :
			{
				// keep position
				speed_control(0.0F,0.0F);
				// transition / condition
				if(HAL_GetTick() > ctx.action_time + 1000)
				{
#ifdef FIXED_MOVES
					ctx.current_state = get_next_move();
#else
					ctx.current_state = ACTION_IDLE;
#endif
					ctx.sub_action_index = 0;
					ctx.action_time = HAL_GetTick();
					ctx.current_pid_type = PID_TYPE_GYRO;

					encoder_reset();
					calibration_reset();
					reset_speed_control();

					led_toggle();
				}
			}
			break;
		}
	}
	break;
	}
}



