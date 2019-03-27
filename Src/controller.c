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

// globals
extern HAL_Serial_Handler com;

// constants
#define CONTROLLER_PERIOD 1200U // us microseconds (= 833Hz ODR GYRO)
#define X_MAX_ACCELERATION 5.0 		// m/s-2
#define X_MAX_DECELERATION 3.0		// m/s-2
#define W_MAX_ACCELERATION 5000		// °/s-2
#define W_MAX_DECELERATION 5000		// °/s-2
#define X_SPEED_LEARNING_RUN 0.34 	// m/s
#define W_SPEED_LEARNING_RUN 205 	// °/s
#define X_SPEED_CALIBRATION -0.08 	// m/s
#define DIST_START 0.09 			// m
#define DIST_RUN_1 0.18 			// m
#define DIST_STOP 0.09 				// m
#define DIST_CALIBRATION -0.20 			// m

#define W_T1 439 					//in ms
#define W_T2 480					//in ms

#define W_U_T1 890 					//in ms
#define W_U_T2 930					//in ms

// speed
#define X_SPEED_KP 600.0
#define X_SPEED_KI 10.0
#define X_SPEED_KD 0.0

// rotation
#define W_SPEED_KP 0.1
#define W_SPEED_KI 0.004
#define W_SPEED_KD 0.0

// ENUM

typedef enum {
	ACTION_IDLE,
	ACTION_START,
	ACTION_RUN_1,
	ACTION_TURN_RIGHT,
	ACTION_TURN_LEFT,
	ACTION_U_TURN_RIGHT,
	ACTION_STOP,
	ACTION_CALIBRATION,
	ACTION_CTR
} action_t;

// STRUCTURES DEFINITIONS

typedef struct  {
	// controller fsm
	uint16_t time_us;
	uint32_t actions_index; // index of current action in the scenario array
	uint32_t action_time;
	uint32_t sub_action_index;
	uint32_t gyro_state;

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

} controller_t;

// GLOBAL VARIABLES
/*
static action_t actions_scenario[] = {
	ACTION_START,
	ACTION_TURN_RIGHT,
	ACTION_RUN_1,
	ACTION_TURN_RIGHT,
	ACTION_STOP,
	ACTION_U_TURN_RIGHT,
	ACTION_START,
	ACTION_TURN_LEFT,
	ACTION_TURN_RIGHT,
	ACTION_TURN_LEFT,
	ACTION_TURN_RIGHT,
	ACTION_STOP,
	ACTION_IDLE
  };
*/

static action_t actions_scenario[] = {
	ACTION_CALIBRATION,
	ACTION_IDLE
  };

static controller_t ctx;


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

static void led_toggle(){
	HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
	HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
}

// PUBLIC FUNCTIONS

uint32_t controller_init () // return GYRO ERROR (ZERO is GYRO OK)
{
	// reset controller fsm
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

	motor_init();
	encoder_init();
	ctx.gyro_state = gyro_init();
	wall_sensor_init();

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

	return ctx.gyro_state;
}

void controller_start()
{
	// reset controller fsm
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


	encoder_reset();

	HAL_DataLogger_Clear();
}

void controller_stop()
{
	// reset controller fsm
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
				(int32_t)(ctx.x_speed_target * 1000.0),	 // integer value of each field
				(int32_t)(ctx.x_speed_setpoint * 1000.0),// integer value of each field
				(int32_t)(ctx.x_speed_current * 1000.0),	 // integer value of each field
				(int32_t)(ctx.x_speed_pwm),				 // integer value of each field
				(int32_t)(ctx.x_speed_error * 10.0),	 // integer value of each field
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

		static uint32_t counter=0;
		if(counter++%100==0)
		{
			HAL_Serial_Print(&com,"WALL %d %d %d %d\r\n",
					wall_sensor_get(WALL_SENSOR_LEFT_STRAIGHT),
					wall_sensor_get(WALL_SENSOR_RIGHT_STRAIGHT),
					wall_sensor_get(WALL_SENSOR_LEFT_DIAG),
					wall_sensor_get(WALL_SENSOR_RIGHT_DIAG)
					);
		}

	}
}

bool controller_is_end(){
	return actions_scenario[ctx.actions_index] == ACTION_IDLE;
}

// PRIVATE FUNCTIONS

void controller_fsm()
{
	switch(actions_scenario[ctx.actions_index])
	{
	case ACTION_IDLE :
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

			++ctx.actions_index;
			ctx.sub_action_index = 0;
			ctx.action_time = HAL_GetTick();

			led_toggle();
			HAL_Serial_Print(&com,".");
		}

	}
	break;

	case ACTION_RUN_1 :
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
		if(dist >= DIST_RUN_1)
		{
			encoder_set_absolute(dist - DIST_RUN_1);

			++ctx.actions_index;
			ctx.sub_action_index = 0;
			ctx.action_time = HAL_GetTick();

			led_toggle();
			HAL_Serial_Print(&com,".");
		}
	}
	break;

	case ACTION_TURN_RIGHT :
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
				++ctx.actions_index;
				ctx.sub_action_index = 0;
				ctx.action_time = HAL_GetTick();

				encoder_reset();

				led_toggle();
				HAL_Serial_Print(&com,".");
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
				++ctx.actions_index;
				ctx.sub_action_index = 0;
				ctx.action_time = HAL_GetTick();

				encoder_reset();

				led_toggle();
				HAL_Serial_Print(&com,".");
			}
			break;
		default:
			break;
		}
	}
	break;

	case ACTION_U_TURN_RIGHT :
	{
		switch (ctx.sub_action_index) {
				//ACCELARATION
				case 0 :
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
					break;
				//DECELERATION
				case 1 :
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
						++ctx.actions_index;
						ctx.sub_action_index = 0;
						ctx.action_time = HAL_GetTick();

						encoder_reset();

						led_toggle();
						HAL_Serial_Print(&com,".");
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

				++ctx.actions_index;
				ctx.sub_action_index = 0;
				ctx.action_time = HAL_GetTick();

				encoder_reset();

				led_toggle();
				HAL_Serial_Print(&com,".");
			}
			break;
		}

	}
	break;

	case ACTION_CALIBRATION :
	{
		// forward speed
		ctx.x_speed_target = X_SPEED_CALIBRATION;
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

		static int32_t calibration_led_left_straight[200];
		static int32_t calibration_led_left_diag[200];
		static int32_t calibration_led_right_straight[200];
		static int32_t calibration_led_right_diag[200];

		if((dist < 0) && (dist > (-200)))
		{
			calibration_led_left_straight[(int32_t)(-dist*1000)] = wall_sensor_get(WALL_SENSOR_LEFT_STRAIGHT);
			calibration_led_left_diag[(int32_t)(-dist*1000)] = wall_sensor_get(WALL_SENSOR_LEFT_DIAG);
			calibration_led_right_straight[(int32_t)(-dist*1000)] = wall_sensor_get(WALL_SENSOR_RIGHT_STRAIGHT);
			calibration_led_right_diag[(int32_t)(-dist*1000)] = wall_sensor_get(WALL_SENSOR_RIGHT_DIAG);
		}

		if(dist <= DIST_CALIBRATION)
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

			++ctx.actions_index;
			ctx.sub_action_index = 0;
			ctx.action_time = HAL_GetTick();

			led_toggle();
			HAL_Serial_Print(&com,".");
			for(uint32_t i=0;i<200;i++)
			{
				HAL_Delay(1);
				HAL_Serial_Print(&com,"%d, %d, %d, %d, %d, %d\n", i, calibration_led_left_straight[i],
															 calibration_led_right_straight[i],
															 calibration_led_left_diag[i],
															 calibration_led_right_diag[i], (int32_t)(log(calibration_led_left_straight[i])*1000));
			}
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
