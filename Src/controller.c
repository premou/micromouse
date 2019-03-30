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

// led
#define MEDIAN_SIZE 3
#define CALIBRATION_SIZE 200 // ((size_t)(-DIST_CALIBRATION*1000))

// ENUM

typedef enum {
	ACTION_IDLE,
	ACTION_START,
	ACTION_RUN_1,
	ACTION_TURN_RIGHT,
	ACTION_TURN_LEFT,
	ACTION_U_TURN_RIGHT,
	ACTION_STOP,
//	ACTION_CALIBRATION_LED,
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

	//led IR
	float a_slope;
	float b_slope;

} controller_t;

//// GLOBAL VARIABLES
//static action_t actions_scenario[] = {
//	ACTION_START,
//	ACTION_TURN_RIGHT,
//	ACTION_RUN_1,
//	ACTION_TURN_RIGHT,
//	ACTION_STOP,
//	ACTION_U_TURN_RIGHT,
//	ACTION_START,
//	ACTION_TURN_LEFT,
//	ACTION_TURN_RIGHT,
//	ACTION_TURN_LEFT,
//	ACTION_TURN_RIGHT,
//	ACTION_STOP,
//	ACTION_IDLE
//  };
// GLOBAL VARIABLES

static action_t actions_scenario[] = {
	ACTION_START,
	ACTION_RUN_1,
	ACTION_STOP,
	ACTION_IDLE
  };


//Calibration actions
//static action_t actions_scenario[] = {
//	ACTION_CALIBRATION_LED,
//	ACTION_IDLE
//  };

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

		wall_sensor_update();
		float dist_led = controller_get_distance_led(wall_sensor_get(WALL_SENSOR_LEFT_STRAIGHT));
		if(dist_led < 20){
			HAL_Serial_Print(&com,"sensor:%d ,dist:%d, dist_led:%d\n",(int)wall_sensor_get(WALL_SENSOR_LEFT_STRAIGHT), (int)dist, (int)dist_led);
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

//	case ACTION_CALIBRATION_LED :
//	{
//		// forward speed
//		ctx.x_speed_target = X_SPEED_CALIBRATION;
//		ctx.x_speed_setpoint = next_speed(ctx.x_speed_target, X_MAX_ACCELERATION, X_MAX_DECELERATION, 0.001, ctx.x_speed_setpoint);
//		ctx.x_speed_current = ((encoder_get_delta_left() + encoder_get_delta_right()) / 2.0) / 0.001;
//		ctx.x_speed_error = ctx.x_speed_setpoint - ctx.x_speed_current;
//		ctx.x_speed_pwm = pid_output(&ctx.x_speed_pid, ctx.x_speed_error);
//
//		// rotation speed
//		ctx.w_speed_target = 0;
//		ctx.w_speed_setpoint = 0;
//		ctx.w_speed_current = gyro_get_dps();
//		ctx.w_speed_error = ctx.w_speed_setpoint - ctx.w_speed_current;
//		ctx.w_speed_pwm = pid_output(&ctx.w_speed_pid, ctx.w_speed_error);
//
//		motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
//		motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);
//
//		float dist = encoder_get_absolute();
//
//		// build raw array on-the-move
//		#define CALIBRATION_SIZE 200 // ((size_t)(-DIST_CALIBRATION*1000))
//		static int32_t calibration_raw_left_straight[CALIBRATION_SIZE];
//		static int32_t calibration_raw_left_diag[CALIBRATION_SIZE];
//		static int32_t calibration_raw_right_straight[CALIBRATION_SIZE];
//		static int32_t calibration_raw_right_diag[CALIBRATION_SIZE];
//		size_t position = (size_t)(-dist*1000);
//		if( (position >= 0) && (position < CALIBRATION_SIZE) )
//		{
//			calibration_raw_left_straight[position] = wall_sensor_get(WALL_SENSOR_LEFT_STRAIGHT);
//			calibration_raw_left_diag[position] = wall_sensor_get(WALL_SENSOR_LEFT_DIAG);
//			calibration_raw_right_straight[position] = wall_sensor_get(WALL_SENSOR_RIGHT_STRAIGHT);
//			calibration_raw_right_diag[position] = wall_sensor_get(WALL_SENSOR_RIGHT_DIAG);
//		}
//
//		HAL_Serial_Print(&com,"TOTO %d dist\n", (int)dist);
//
//		if(dist <= DIST_CALIBRATION)
//		{
//			// forward speed
//			ctx.x_speed_target = 0;
//			ctx.x_speed_setpoint = 0;
//			ctx.x_speed_current = 0;
//			ctx.x_speed_error = 0;
//			ctx.x_speed_pwm = 0;
//
//			// rotation speed
//			ctx.w_speed_target = 0;
//			ctx.w_speed_setpoint = 0;
//			ctx.w_speed_current = 0;
//			ctx.w_speed_error = 0;
//			ctx.w_speed_pwm = 0;
//
//			motor_speed_left(ctx.x_speed_pwm - ctx.w_speed_pwm);
//			motor_speed_right(ctx.x_speed_pwm + ctx.w_speed_pwm);
//
//			++ctx.actions_index;
//			ctx.sub_action_index = 0;
//			ctx.action_time = HAL_GetTick();
//
//			led_toggle();
//
//			// display raw arrays
//			HAL_Serial_Print(&com,".");
//			for(uint32_t i=0;i<CALIBRATION_SIZE;i++)
//			{
//				HAL_Delay(1);
//				HAL_Serial_Print(&com,"%d, %d, %d, %d, %d\n",
//						i,
//						calibration_raw_left_straight[i],
//						calibration_raw_right_straight[i],
//						calibration_raw_left_diag[i],
//						calibration_raw_right_diag[i]);
//			}
//
//			int compare (const void * a, const void * b)
//			{
//			  return ( *(int*)a - *(int*)b );
//			}
//
//			// calibrate 0) median filter
//			static int32_t calibration_median_left_straight[CALIBRATION_SIZE];
//			static int32_t calibration_median_left_diag[CALIBRATION_SIZE];
//			static int32_t calibration_median_right_straight[CALIBRATION_SIZE];
//			static int32_t calibration_median_right_diag[CALIBRATION_SIZE];
//			#define MEDIAN_SIZE 3
//			static int32_t median_buf_left_straight[MEDIAN_SIZE];
//			static int32_t median_buf_left_diag[MEDIAN_SIZE];
//			static int32_t median_buf_right_straight[MEDIAN_SIZE];
//			static int32_t median_buf_right_diag[MEDIAN_SIZE];
//			for(uint32_t index=1;index<CALIBRATION_SIZE-1;index++)
//			{
//				for(int32_t index_sort=0;index_sort<MEDIAN_SIZE;index_sort++)
//				{
//					median_buf_left_straight[index_sort]=calibration_raw_left_straight[index+index_sort-(size_t)floor((float)MEDIAN_SIZE/2.0)];
//					median_buf_left_diag[index_sort]=calibration_raw_left_diag[index+index_sort-(size_t)floor((float)MEDIAN_SIZE/2.0)];
//					median_buf_right_straight[index_sort]=calibration_raw_right_straight[index+index_sort-(size_t)floor((float)MEDIAN_SIZE/2.0)];
//					median_buf_right_diag[index_sort]=calibration_raw_right_diag[index+index_sort-(size_t)floor((float)MEDIAN_SIZE/2.0)];
//				}
//				qsort(median_buf_left_straight,MEDIAN_SIZE,sizeof(int32_t),compare);
//				qsort(median_buf_left_diag,MEDIAN_SIZE,sizeof(int32_t),compare);
//				qsort(median_buf_right_straight,MEDIAN_SIZE,sizeof(int32_t),compare);
//				qsort(median_buf_right_diag,MEDIAN_SIZE,sizeof(int32_t),compare);
//
//				calibration_median_left_straight[index]=median_buf_left_straight[(size_t)ceil((float)MEDIAN_SIZE/2.0)];
//				calibration_median_left_diag[index]=median_buf_left_diag[(size_t)ceil((float)MEDIAN_SIZE/2.0)];
//				calibration_median_right_straight[index]=median_buf_right_straight[(size_t)ceil((float)MEDIAN_SIZE/2.0)];
//				calibration_median_right_diag[index]=median_buf_right_diag[(size_t)ceil((float)MEDIAN_SIZE/2.0)];
//			}
//
//			float calibration_a[CALIBRATION_SIZE];
//			float calibration_b[CALIBRATION_SIZE];
//			float a_moy=0;
//			uint32_t a_count=0;
//			float a_sum=0;
//
//			//ADC=a/ln(dist)
//			for(uint32_t index=0;index<CALIBRATION_SIZE-5;index++){
//
//				float y1 = (float)(index);
//				float y2 = (float)(index+5);
//
//				float x1 = 1.0/log((float)calibration_median_left_straight[index]);
//				float x2 = 1.0/log((float)calibration_median_left_straight[index+5]);
//
//				if((x2-x1) != 0)
//				{
//					//significatives values between 30 and 100
//					calibration_a[index] = (y2 - y1)/(x2 - x1);
//					if((index>=30) && (index<=100))
//					{
//						a_sum += calibration_a[index];
//						++a_count;
//					}
//				}
//				else
//				{
//					calibration_a[index]=0.0;
//				}
//
//			}
//			a_moy = a_sum/(float)a_count;
//			HAL_Serial_Print(&com,"a_moy is :%d, a_count is %d\n", (int)a_moy, a_count);
//
//			float b_moy=0;
//			uint32_t b_count=0;
//			float b_sum=0;
//
//			for(uint32_t index=0;index<CALIBRATION_SIZE-1;index++){
//
//				float y1 = (float)(index);
//				float x1 = 1.0/log((float)calibration_median_left_straight[index]);
//
//					calibration_b[index] = a_moy*x1 - y1;
//					if((index>=30) && (index<=100))
//					{
//						b_sum += calibration_b[index];
//						++b_count;
//					}
//
//			}
//			b_moy = b_sum/(float)b_count;
//			HAL_Serial_Print(&com,"b_moy is :%d, b_count is %d\n", (int)b_moy, b_count);
//
//			float distance_error[CALIBRATION_SIZE];
//			for(uint32_t i=0;i<CALIBRATION_SIZE;i++)
//			{
//				distance_error[i]=(float)i - a_moy/log(calibration_raw_left_straight[i]) + b_moy;
//			}
//			HAL_Serial_Print(&com,".");
//			for(uint32_t i=0;i<CALIBRATION_SIZE;i++)
//			{
//				HAL_Delay(1);
//				HAL_Serial_Print(&com,"%d,%d\n",
//						i,
//						(int)distance_error[i]);
//			}
//
//			// display raw arrays
////			HAL_Serial_Print(&com,".");
////			for(uint32_t i=0;i<CALIBRATION_SIZE;i++)
////			{
////				HAL_Delay(1);
////				HAL_Serial_Print(&com,"%d, %d, %d, %d, %d\n",
////						i,
////						calibration_median_left_straight[i],
////						calibration_median_right_straight[i],
////						calibration_median_left_diag[i],
////						calibration_median_right_diag[i]);
////			}
//
////			HAL_Serial_Print(&com,".");
////			for(uint32_t i=0;i<CALIBRATION_SIZE;i++)
////			{
////				HAL_Delay(1);
////				HAL_Serial_Print(&com,"%d,%d\n",
////						i,
////						(int)calibration_a[i]);
////			}
//
//		}
//
//	}
//	break;

	case ACTION_CTR :
	{
		// nop
	}
	break;

	}
}

void controller_led_calibrate(){
	float dist = DIST_CALIBRATION + 1.0;

	// build raw array on-the-move
	static int32_t calibration_raw_left_straight[CALIBRATION_SIZE];
	static int32_t calibration_raw_left_diag[CALIBRATION_SIZE];
	static int32_t calibration_raw_right_straight[CALIBRATION_SIZE];
	static int32_t calibration_raw_right_diag[CALIBRATION_SIZE];

	encoder_reset();

	while (dist > DIST_CALIBRATION)
	{
		// FIX : Period 1200us

		// sensor update
		encoder_update();
		gyro_update();
		wall_sensor_update();

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

		dist = encoder_get_absolute();

		size_t position = (size_t)(-dist*1000);
		if( (position >= 0) && (position < CALIBRATION_SIZE) )
		{
			calibration_raw_left_straight[position] = wall_sensor_get(WALL_SENSOR_LEFT_STRAIGHT);
			calibration_raw_left_diag[position] = wall_sensor_get(WALL_SENSOR_LEFT_DIAG);
			calibration_raw_right_straight[position] = wall_sensor_get(WALL_SENSOR_RIGHT_STRAIGHT);
			calibration_raw_right_diag[position] = wall_sensor_get(WALL_SENSOR_RIGHT_DIAG);
		}
	}
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

	// display raw arrays
	HAL_Serial_Print(&com,".");
	for(uint32_t i=0;i<CALIBRATION_SIZE;i++)
	{
		HAL_Delay(1);
		HAL_Serial_Print(&com,"%d, %d, %d, %d, %d\n",
				i,
				calibration_raw_left_straight[i],
				calibration_raw_right_straight[i],
				calibration_raw_left_diag[i],
				calibration_raw_right_diag[i]);
	}

	int compare (const void * a, const void * b)
	{
		return ( *(int*)a - *(int*)b );
	}

	// calibrate 0) median filter
	static int32_t calibration_median_left_straight[CALIBRATION_SIZE];
	static int32_t calibration_median_left_diag[CALIBRATION_SIZE];
	static int32_t calibration_median_right_straight[CALIBRATION_SIZE];
	static int32_t calibration_median_right_diag[CALIBRATION_SIZE];

	static int32_t median_buf_left_straight[MEDIAN_SIZE];
	static int32_t median_buf_left_diag[MEDIAN_SIZE];
	static int32_t median_buf_right_straight[MEDIAN_SIZE];
	static int32_t median_buf_right_diag[MEDIAN_SIZE];
	for(uint32_t index=1;index<CALIBRATION_SIZE-1;index++)
	{
		for(int32_t index_sort=0;index_sort<MEDIAN_SIZE;index_sort++)
		{
			median_buf_left_straight[index_sort]=calibration_raw_left_straight[index+index_sort-(size_t)floor((float)MEDIAN_SIZE/2.0)];
			median_buf_left_diag[index_sort]=calibration_raw_left_diag[index+index_sort-(size_t)floor((float)MEDIAN_SIZE/2.0)];
			median_buf_right_straight[index_sort]=calibration_raw_right_straight[index+index_sort-(size_t)floor((float)MEDIAN_SIZE/2.0)];
			median_buf_right_diag[index_sort]=calibration_raw_right_diag[index+index_sort-(size_t)floor((float)MEDIAN_SIZE/2.0)];
		}
		qsort(median_buf_left_straight,MEDIAN_SIZE,sizeof(int32_t),compare);
		qsort(median_buf_left_diag,MEDIAN_SIZE,sizeof(int32_t),compare);
		qsort(median_buf_right_straight,MEDIAN_SIZE,sizeof(int32_t),compare);
		qsort(median_buf_right_diag,MEDIAN_SIZE,sizeof(int32_t),compare);

		calibration_median_left_straight[index]=median_buf_left_straight[(size_t)ceil((float)MEDIAN_SIZE/2.0)];
		calibration_median_left_diag[index]=median_buf_left_diag[(size_t)ceil((float)MEDIAN_SIZE/2.0)];
		calibration_median_right_straight[index]=median_buf_right_straight[(size_t)ceil((float)MEDIAN_SIZE/2.0)];
		calibration_median_right_diag[index]=median_buf_right_diag[(size_t)ceil((float)MEDIAN_SIZE/2.0)];
	}

	float calibration_a[CALIBRATION_SIZE];
	float calibration_b[CALIBRATION_SIZE];

	uint32_t a_count=0;
	float a_sum=0;

	//ADC=a/ln(dist)
	for(uint32_t index=0;index<CALIBRATION_SIZE-5;index++){

		float y1 = (float)(index);
		float y2 = (float)(index+5);

		float x1 = 1.0/log((float)calibration_median_left_straight[index]);
		float x2 = 1.0/log((float)calibration_median_left_straight[index+5]);

		if((x2-x1) != 0)
		{
			//significatives values between 30 and 100
			calibration_a[index] = (y2 - y1)/(x2 - x1);
			if((index>=30) && (index<=100))
			{
				a_sum += calibration_a[index];
				++a_count;
			}
		}
		else
		{
			calibration_a[index]=0.0;
		}

	}
	ctx.a_slope = a_sum/(float)a_count;
	HAL_Serial_Print(&com,"a_moy is :%d, a_count is %d\n", (int)ctx.a_slope, a_count);

	uint32_t b_count=0;
	float b_sum=0;

	for(uint32_t index=0;index<CALIBRATION_SIZE-1;index++){

		float y1 = (float)(index);
		float x1 = 1.0/log((float)calibration_median_left_straight[index]);

		calibration_b[index] = ctx.a_slope*x1 - y1;
		if((index>=30) && (index<=100))
		{
			b_sum += calibration_b[index];
			++b_count;
		}

	}
	ctx.b_slope = b_sum/(float)b_count;
	HAL_Serial_Print(&com,"b_moy is :%d, b_count is %d\n", (int)ctx.b_slope, b_count);

	float distance_error[CALIBRATION_SIZE];
	for(uint32_t i=0;i<CALIBRATION_SIZE;i++)
	{
		distance_error[i]=(float)i - ctx.a_slope/log(calibration_raw_left_straight[i]) + ctx.b_slope;
	}
	HAL_Serial_Print(&com,".");
	for(uint32_t i=0;i<CALIBRATION_SIZE;i++)
	{
		HAL_Delay(1);
		HAL_Serial_Print(&com,"%d,%d\n",
				i,
				(int)distance_error[i]);
	}

}

float controller_get_distance_led(int32_t adc){
	return ctx.a_slope/adc + ctx.b_slope;
}
