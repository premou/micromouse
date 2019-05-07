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

////////////
// settings (To Be Deleted)
////////////

// speed
#define X_SPEED_FAST_RUN 0.7 // m/s
#define X_SPEED_FAST_RUN_IMPROVED 1.0 // m/s

// U turn
//#define W_U_T1 890 					//in ms
#define W_U_T1 280 					//in ms
//#define W_U_T2 930					//in ms
//#define W_U_T2 465					//in ms

// front wall distance position  PID
#define X_WALL_FRONT_KP 1.0
#define X_WALL_FRONT_KI 0.0001
#define X_WALL_FRONT_KD 0.0

// front wall angle position  PID
#define W_WALL_FRONT_KP 1.0
#define W_WALL_FRONT_KI 0.0001
#define W_WALL_FRONT_KD 0.0

////////////
// end of settings (To Be Deleted)
////////////

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
// HELPER FUNCTIONS
///////////////////

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

	// reset all PID and setpoint
	reset_speed_control();

	// reset longitudinal calibration (wall-to-no-wall, post-to-no-post)
	calibration_reset();

	// reset encoder
	encoder_reset();

	// AI
	maze_ctx_start(&ctx.maze) ;

	// action
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
	ctx.action_time = HAL_GetTick();
	ctx.sub_action_index = 0;
	ctx.current_pid_type = PID_TYPE_GYRO;

	// reset all PID and setpoint
	reset_speed_control();

	// reset longitudinal calibration (wall-to-no-wall, post-to-no-post)
	calibration_reset();

	// reset encoder
	encoder_reset();

	// AI
	// nop

	// action
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

	//action new u_turn is concatenation of action stop and action break and action old u_turn and action break and action start
	case ACTION_U_TURN_RIGHT :
	{
		switch (ctx.sub_action_index) {
				// CRUISE
				case 0 :
				{
					// forward speed
					ctx.x_speed_target = X_SPEED;
					ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, CONTROLLER_PERDIO_F, ctx.x_speed_setpoint);
					ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) * CONTROLLER_FREQ_F;
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
					ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, CONTROLLER_PERDIO_F, ctx.x_speed_setpoint);
					ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) * CONTROLLER_FREQ_F;
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
					ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, CONTROLLER_PERDIO_F, ctx.x_speed_setpoint);
					ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) * CONTROLLER_FREQ_F;
					ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
					ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

					// rotation speed
					ctx.w_speed_target = -W_SPEED;
					ctx.w_speed_setpoint = next_speed(ctx.w_speed_target, W_MAX_ACCELERATION, W_MAX_DECELERATION, CONTROLLER_PERDIO_F, ctx.w_speed_setpoint);
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
						ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, CONTROLLER_PERDIO_F, ctx.x_speed_setpoint);
						ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) * CONTROLLER_FREQ_F;
						ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
						ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);


						// rotation speed
						ctx.w_speed_target = 0.0;
						ctx.w_speed_setpoint = next_speed(ctx.w_speed_target, W_MAX_ACCELERATION, W_MAX_DECELERATION, CONTROLLER_PERDIO_F, ctx.w_speed_setpoint);
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
					ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, CONTROLLER_PERDIO_F, ctx.x_speed_setpoint);
					ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) * CONTROLLER_FREQ_F;
					ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
					ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

					// rotation speed
					ctx.w_speed_target = -W_SPEED;
					ctx.w_speed_setpoint = next_speed(ctx.w_speed_target, W_MAX_ACCELERATION, W_MAX_DECELERATION, CONTROLLER_PERDIO_F, ctx.w_speed_setpoint);
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
						ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, CONTROLLER_PERDIO_F, ctx.x_speed_setpoint);
						ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) * CONTROLLER_FREQ_F;
						ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
						ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

						// rotation speed
						ctx.w_speed_target = 0.0;
						ctx.w_speed_setpoint = next_speed(ctx.w_speed_target, W_MAX_ACCELERATION, W_MAX_DECELERATION, CONTROLLER_PERDIO_F, ctx.w_speed_setpoint);
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
						ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, CONTROLLER_PERDIO_F, ctx.x_speed_setpoint);
						ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) * CONTROLLER_FREQ_F;
						ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
						ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);

						// rotation speed
						ctx.w_speed_target = 0.0;
						ctx.w_speed_setpoint = next_speed(ctx.w_speed_target, W_MAX_ACCELERATION, W_MAX_DECELERATION, CONTROLLER_PERDIO_F, ctx.w_speed_setpoint);
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
						ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, CONTROLLER_PERDIO_F, ctx.x_speed_setpoint);
						ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) * CONTROLLER_FREQ_F;
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
				// move forward and use side walls
				if( (encoder_get_absolute() <= DIST_RUN_1/2.0) && (wall_sensor_is_left_wall_detected() || wall_sensor_is_right_wall_detected()) )
				{
					ctx.current_pid_type = PID_TYPE_WALL;
					speed_control_with_wall_following(0.0F);
				}
				else
				{
					ctx.current_pid_type = PID_TYPE_GYRO;
					speed_control(0.0F,0.0F);
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

	case ACTION_CTR :
	{
		// nop
	}
	break;

	}
}



