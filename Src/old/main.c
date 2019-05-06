
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */
#include "serial.h"
#include "mmi.h"
#include "encoder.h"
#include "motor.h"
#include "imu.h"
#include "ewma.h"
#include "speed.h"
#include "pid.h"
#include "battery.h"
#include "buzzer.h"
#include "WallSensor.h"
#include "datalogger.h"
#include "mymath.h"
#include "Maze.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
HAL_Serial_Handler serial;
HAL_Encoder_HandleTypeDef front_left_encoder;
HAL_Encoder_HandleTypeDef front_right_encoder;
HAL_Encoder_HandleTypeDef back_left_encoder;
HAL_Encoder_HandleTypeDef back_right_encoder;
IMU_HandleTypeDef imu;

// Controller
enum
{
	IDLE=0,
	WARMUP,
	RUNNING,
	STOP
};
unsigned int state = IDLE;
uint32_t state_time = 0;
float target_x_speed = 0.0f;
float target_w_speed = 0.0f;
float current_x_speed = 0.0f;
float current_w_speed = 0.0f;
float actual_x_speed = 0.0f;
float actual_w_speed = 0.0f;
double target_heading = 0.0f;
double actual_heading = 0.0f;
float error_x_speed = 0.0f;
float error_w_speed = 0.0f;
float error_x_position = 0.0f;
float error_w_position = 0.0f;
float error_x_position_integral = 0.0f;
float error_w_position_integral = 0.0f;
float error_w_IR = 0.0f;
float error_w_IR_integral = 0.0f;
float pwm_x_speed = 0.0f;
float pwm_w_speed = 0.0f;
int32_t pwm_left = 0.0f;
int32_t pwm_right = 0.0f;
float ewma_x_speed_alpha = 0.2f; //0.2
ewma_handler ewma_x_speed = {&ewma_x_speed_alpha, 0.0};
float ewma_w_speed_alpha = 1.0f; //0.9
ewma_handler ewma_w_speed = {&ewma_w_speed_alpha, 0.0};
float const x_acceleration = 3.0f;
float const x_deceleration = 1.0f;
float pid_x_kp = 40.0f;
float pid_x_ki = 0.0f;
float pid_x_kd = 1500.0f;
float pid_x_kff = 1000.0f; // feed forward xKm
pid_mm_handler pid_x_speed = {
		&pid_x_kp,
		&pid_x_ki,
		&pid_x_kd,
		-999.0f,
		999.0f,
	    0.2
};
float gyro_sensitivity_correction = 0.9812f;
float const w_acceleration = 1500.0f;
float const w_deceleration = 1500.0f;
float w_curve_speed = 250.0f;
float w_curve_d1 = 0.014f;
float w_curve_d2 = 0.014f;
float w_curve_t1 = 0.170f;
float w_curve_t2 = 0.187f; //190
float w_uturn90_t1 = 0.170f;
float w_uturn90_t2 = 0.180f;
float w_uturn180_t2 = 0.530f;
float w_d45_t1 = 0.167f;
float w_d45_t2 = 0.013f;
float w_d90_t1 = 0.167f;
float w_d90_t2 = 0.193f;
float w_d135_t1 = 0.167f;
float w_d135_t2 = 0.373f;
float pid_w_kp = 0.008f;
float pid_w_ki = 0.0f;
float pid_w_kd = 0.300f;
float pid_w_kff = 0.700f; // feed forward wKm
float pid_w_kir = 1200.0f; // IR wall following
pid_mm_handler pid_w_speed = {
		&pid_w_kp,
		&pid_w_ki,
		&pid_w_kd,
		-999.0f,
		999.0f,
	    0.75
};
float error_x_IR = 0.0f;
float pid_xir_kp = 15.0f;
float pid_xir_ki = 0.000f;
float pid_xir_kd = 0.000f;
pid_handler pid_x_ir = {
		&pid_xir_kp,
		&pid_xir_ki,
		&pid_xir_kd,
		-300.0f,
		300.0f,
		0.9
};
float pid_wir_kp = 10.000f;
float pid_wir_ki = 0.000f;
float pid_wir_kd = 0.000f;
pid_handler pid_w_ir = {
		&pid_wir_kp,
		&pid_wir_ki,
		&pid_wir_kd,
		-300.0f,
		300.0f,
		0.9
};



enum {
	MOVE_IDLE = 0,

	MOVE_START,
	MOVE_FINISH,
	MOVE_U_TURN,

	MOVE_SS_FORWARD,

	MOVE_SS_LEFT_90,
	MOVE_SS_RIGHT_90,

	MOVE_SS_LEFT_90_FAST,
	MOVE_SS_RIGHT_90_FAST,

	MOVE_SS_LEFT_180_FAST,
	MOVE_SS_RIGHT_180_FAST,

	MOVE_SD_LEFT_45,
	MOVE_SD_RIGHT_45,

	MOVE_SD_LEFT_135,
	MOVE_SD_RIGHT_135,

	MOVE_DD_FORWARD,

	MOVE_DD_LEFT_90,
	MOVE_DD_RIGHT_90,

	MOVE_DS_LEFT_45,
	MOVE_DS_RIGHT_45,

	MOVE_DS_LEFT_135,
	MOVE_DS_RIGHT_135,

	MOVE_STEP_RESPONSE_X,
	MOVE_STEP_RESPONSE_W,

	MOVE_FOLLOW_FRONT_WALL

};
unsigned int move_state = MOVE_IDLE;
enum {
	SUBMOVE_0 = 0,
	SUBMOVE_1,
	SUBMOVE_2,
	SUBMOVE_3,
	SUBMOVE_4,
	SUBMOVE_5,
	SUBMOVE_6,
	SUBMOVE_7,
	SUBMOVE_8,
	SUBMOVE_9,
	SUBMOVE_10,
	SUBMOVE_11,
	SUBMOVE_12,
	SUBMOVE_13,
	SUBMOVE_14,
	SUBMOVE_15,
	SUBMOVE_16,
	SUBMOVE_17,
	SUBMOVE_18,
	SUBMOVE_19,
	SUBMOVE_20,
	SUBMOVE_21,
	SUBMOVE_22,
	SUBMOVE_23,
	SUBMOVE_24,
	SUBMOVE_25,
	SUBMOVE_26,
	SUBMOVE_27,
	SUBMOVE_28,
	SUBMOVE_29,
	SUBMOVE_30
};
unsigned int submove_state = SUBMOVE_0;
float remaining_distance = 0.0f;
float remaining_time = 0.0f;
unsigned int move_plan[] = {

//		MOVE_STEP_RESPONSE_X,
//		MOVE_FOLLOW_FRONT_WALL,
//


//		MOVE_START,
//		MOVE_SS_FORWARD,
//		MOVE_U_TURN,
//		MOVE_SS_FORWARD,
//		MOVE_FINISH

//		MOVE_START,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_FINISH

//		MOVE_START,
//		MOVE_U_TURN,
//		MOVE_FINISH

//		MOVE_SS_LEFT_90,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_LEFT_90,
//		MOVE_FINISH

		//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_FINISH,

//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//
//
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//
//
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_FORWARD,
//		MOVE_SS_LEFT_90,


//		MOVE_U_TURN,
//		MOVE_SS_RIGHT_90,

//		MOVE_FINISH,

//		MOVE_IDLE
};
unsigned int move_plan_step = 0;
bool cell_has_left_wall = false;
bool cell_has_front_wall = false;
bool cell_has_right_wall = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// IHM helper
void start()
{
	state = WARMUP;
}

// IHM Helper
void send_log()
{
	HAL_DataLogger_Send();
}

// IHM Helper
void batt()
{
	HAL_Serial_Print(&serial,"battery=%dmV\n", (int32_t)(HAL_Battery_Get(1)*1000.0f));
}

// Speed control helper
void reset_speed_control()
{
	reset_ewma(&ewma_x_speed);
	reset_ewma(&ewma_w_speed);

	target_x_speed = 0.0f;
	target_w_speed = 0.0f;
	current_x_speed = 0.0f;
	current_w_speed = 0.0f;
	error_x_speed = 0.0f;
	error_w_speed = 0.0f;
	error_x_position = 0.0f;
	error_w_position = 0.0f;
	error_x_position_integral = 0.0f;
	error_w_position_integral = 0.0f;

	error_x_IR = 0.0f;
	error_w_IR = 0.0f;
	error_w_IR_integral = 0.0f;

	reset_pid_mm(&pid_x_speed);
	reset_pid_mm(&pid_w_speed);
	reset_pid(&pid_x_ir);
	reset_pid(&pid_w_ir);

	pwm_x_speed = 0;
	pwm_w_speed =  0;

	pwm_left = pwm_x_speed - pwm_w_speed;
	pwm_right = pwm_x_speed + pwm_w_speed;
	HAL_Motor_Set(HAL_MOTOR_LEFT,HAL_MOTOR_AUTO,pwm_left);
	HAL_Motor_Set(HAL_MOTOR_RIGHT,HAL_MOTOR_AUTO,pwm_right);
}

// Speed control helper
void process_speed_control(
		float xspd, //! set target speed
		float wspd, //! set target speed,
		uint16_t delta_time_us // period in us
)
{
	target_x_speed = xspd;
	target_w_speed = wspd;
	next_speed(&current_x_speed,target_x_speed,x_acceleration,x_deceleration,delta_time_us);
	next_speed(&current_w_speed,target_w_speed,w_acceleration,w_deceleration,delta_time_us);
	error_x_speed = current_x_speed-actual_x_speed;
	error_w_speed = current_w_speed-actual_w_speed;
	error_x_position += error_x_speed;
	error_w_position += error_w_speed;
	error_x_position_integral = 0.0f;
	error_w_position_integral = 0.0f;
	error_w_IR = 0.0f;
	error_w_IR_integral = 0.0f;
	pwm_x_speed = process_pid_mm(&pid_x_speed,error_x_position,error_x_position_integral,error_x_speed) + pid_x_kff*current_x_speed;
	pwm_w_speed =  process_pid_mm(&pid_w_speed,error_w_position,error_w_position_integral,error_w_speed) + pid_w_kff*current_w_speed;
	pwm_left = pwm_x_speed - pwm_w_speed;
	pwm_right = pwm_x_speed + pwm_w_speed;
	HAL_Motor_Set(HAL_MOTOR_LEFT,HAL_MOTOR_AUTO,pwm_left);
	HAL_Motor_Set(HAL_MOTOR_RIGHT,HAL_MOTOR_AUTO,pwm_right);
}

// Speed control helper
void process_speed_control_with_wall_following(
		float xspd, //! set target speed
		float wspd, //! set target speed,
		uint16_t delta_time_us // period in us
)
{
	target_x_speed = xspd;
	target_w_speed = wspd;
	next_speed(&current_x_speed,target_x_speed,x_acceleration,x_deceleration,delta_time_us);
	next_speed(&current_w_speed,target_w_speed,w_acceleration,w_deceleration,delta_time_us);
	error_x_speed = current_x_speed-actual_x_speed;
	error_w_speed = current_w_speed-actual_w_speed;
	error_x_position += error_x_speed;
	error_w_position += error_w_speed;
	error_x_position_integral = 0.0f;
	error_w_position_integral = 0.0f;

	bool wall_on_the_left = HAL_WallSensor_Get(WALL_SENSOR_LEFT_DIAG) < 80;
	bool wall_on_the_right = HAL_WallSensor_Get(WALL_SENSOR_RIGHT_DIAG) < 80;
	if(wall_on_the_left && wall_on_the_right)
	{
		error_w_IR = (HAL_WallSensor_Get(WALL_SENSOR_LEFT_DIAG) - HAL_WallSensor_Get(WALL_SENSOR_RIGHT_DIAG))*pid_w_kir;
		error_w_IR_integral += error_w_IR;
		error_w_speed = 0.0f;
		error_w_position = 0.0f;
	}
	else if(wall_on_the_left)
	{
		error_w_IR = (HAL_WallSensor_Get(WALL_SENSOR_LEFT_DIAG) - 80)*pid_w_kir;
		error_w_IR_integral += error_w_IR;
		error_w_speed = 0.0f;
		error_w_position = 0.0f;
	}
	else if(wall_on_the_right)
	{
		error_w_IR = (80 - HAL_WallSensor_Get(WALL_SENSOR_RIGHT_DIAG))*pid_w_kir;
		error_w_IR_integral += error_w_IR;
		error_w_speed = 0.0f;
		error_w_position = 0.0f;
	}
	else
	{
		error_w_IR = 0.0f;
		error_w_IR_integral = 0.0f;
	}

	pwm_x_speed = process_pid_mm(&pid_x_speed,error_x_position,error_x_position_integral,error_x_speed) + pid_x_kff*current_x_speed;
	pwm_w_speed =  process_pid_mm(&pid_w_speed,error_w_position+error_w_IR,error_w_position_integral+error_w_IR_integral,error_w_speed) + pid_w_kff*current_w_speed;
	pwm_left = pwm_x_speed - pwm_w_speed;
	pwm_right = pwm_x_speed + pwm_w_speed;
	HAL_Motor_Set(HAL_MOTOR_LEFT,HAL_MOTOR_AUTO,pwm_left);
	HAL_Motor_Set(HAL_MOTOR_RIGHT,HAL_MOTOR_AUTO,pwm_right);
}

// Speed control helper
void process_speed_control_with_front_wall_following()
{
	bool wall_on_the_left = HAL_WallSensor_Get(WALL_SENSOR_LEFT_STRAIGHT) < 70;
	bool wall_on_the_right = HAL_WallSensor_Get(WALL_SENSOR_RIGHT_STRAIGHT) < 70;


	if(wall_on_the_left && wall_on_the_right)
	{
		error_x_IR = (HAL_WallSensor_Get(WALL_SENSOR_LEFT_STRAIGHT) + HAL_WallSensor_Get(WALL_SENSOR_LEFT_STRAIGHT)) - 62;
		error_w_IR = -(HAL_WallSensor_Get(WALL_SENSOR_LEFT_STRAIGHT) - HAL_WallSensor_Get(WALL_SENSOR_RIGHT_STRAIGHT) - 9);
	}
	else
	{
		error_x_IR = 0.0f;
		error_w_IR = 0.0f;

	}
	pwm_x_speed = process_pid(&pid_x_ir,error_x_IR);
	pwm_w_speed =  process_pid(&pid_w_ir,error_w_IR);
	pwm_left = pwm_x_speed - pwm_w_speed;
	pwm_right = pwm_x_speed + pwm_w_speed;
	HAL_Motor_Set(HAL_MOTOR_LEFT,HAL_MOTOR_AUTO,pwm_left);
	HAL_Motor_Set(HAL_MOTOR_RIGHT,HAL_MOTOR_AUTO,pwm_right);
}

/// wall calibration helper
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
};
uint32_t calibration_state = CALIBRATION_IDLE;
float ewma_alpha_calibration_wall_distance = 0.25f;
ewma_handler ewma_calibration_wall_distance_left = {&ewma_alpha_calibration_wall_distance, 0.0};
ewma_handler ewma_calibration_wall_distance_right = {&ewma_alpha_calibration_wall_distance, 0.0};
float calibration_wall_distance_left = 0.0f;
float calibration_wall_distance_right = 0.0f;

void calibration_reset()
{
	calibration_state = CALIBRATION_IDLE;
	reset_ewma(&ewma_calibration_wall_distance_left);
	reset_ewma(&ewma_calibration_wall_distance_right);
	calibration_wall_distance_left = 0.0f;
	calibration_wall_distance_right = 0.0f;
}

bool calibration_process()
{
	calibration_wall_distance_left = process_ewma(&ewma_calibration_wall_distance_left,HAL_WallSensor_Get(WALL_SENSOR_LEFT_DIAG));
	calibration_wall_distance_right = process_ewma(&ewma_calibration_wall_distance_right,HAL_WallSensor_Get(WALL_SENSOR_RIGHT_DIAG));
	switch(calibration_state)
	{
	case CALIBRATION_IDLE:
		{
			if(remaining_distance<0.15f)
			{
				if(calibration_wall_distance_left<120 && calibration_wall_distance_right<120)
				{
					calibration_state = CALIBRATION_WALL_BOTH;
				}
				else if(calibration_wall_distance_left<120)
				{
					calibration_state = CALIBRATION_WALL_LEFT;
				}
				else if(calibration_wall_distance_right<120)
				{
					calibration_state = CALIBRATION_WALL_RIGHT;
				}
				else
				{
					calibration_state = CALIBRATION_NO_WALL;
				}
			}
		}
		break;
	case CALIBRATION_NO_WALL:
		{
			if(calibration_wall_distance_left<140 && calibration_wall_distance_right<140)
			{
				calibration_state = CALIBRATION_POST_BOTH;
			}
			else if(calibration_wall_distance_left<140)
			{
				calibration_state = CALIBRATION_POST_LEFT;
			}
			else if(calibration_wall_distance_right<140)
			{
				calibration_state = CALIBRATION_POST_RIGHT;
			}
		}
		break;
	case CALIBRATION_POST_LEFT:
		{
			if(calibration_wall_distance_left>140)
			{
				remaining_distance = 0.070;
				calibration_state = CALIBRATION_END;
				return true;
			}
			if(calibration_wall_distance_right<140)
			{
				calibration_state = CALIBRATION_POST_BOTH;
			}
		}
		break;
	case CALIBRATION_POST_RIGHT:
		{
			if(calibration_wall_distance_right>140)
			{
				remaining_distance = 0.070;
				calibration_state = CALIBRATION_END;
				return true;
			}
			if(calibration_wall_distance_left<140)
			{
				calibration_state = CALIBRATION_POST_BOTH;
			}
		}
		break;
	case CALIBRATION_POST_BOTH:
		{
			if(calibration_wall_distance_right>140 || calibration_wall_distance_left>140)
			{
				remaining_distance = 0.070;
				calibration_state = CALIBRATION_END;
				return true;
			}
		}
		break;
	case CALIBRATION_WALL_LEFT:
		{
			if(calibration_wall_distance_left>140)
			{
				remaining_distance = 0.065;
				calibration_state = CALIBRATION_END;
				return true;
			}
			if(calibration_wall_distance_right<140)
			{
				calibration_state = CALIBRATION_WALL_LEFT_with_RIGHT_POST;
			}
		}
		break;
	case CALIBRATION_WALL_RIGHT:
		{
			if(calibration_wall_distance_right>140)
			{
				remaining_distance = 0.065;
				calibration_state = CALIBRATION_END;
				return true;
			}
			if(calibration_wall_distance_left<140)
			{
				calibration_state = CALIBRATION_WALL_RIGHT_with_LEFT_POST;
			}
		}
		break;
	case CALIBRATION_WALL_BOTH:
		{
			if(calibration_wall_distance_right>140 || calibration_wall_distance_left>140)
			{
				remaining_distance = 0.065;
				calibration_state = CALIBRATION_END;
				return true;
			}
		}
		break;
	case CALIBRATION_WALL_LEFT_with_RIGHT_POST:
		{
			if(calibration_wall_distance_left>140)
			{
				remaining_distance = 0.065;
				calibration_state = CALIBRATION_END;
				return true;
			}
			if(calibration_wall_distance_right>140)
			{
				remaining_distance = 0.070;
				calibration_state = CALIBRATION_END;
				return true;
			}

		}
		break;
	case CALIBRATION_WALL_RIGHT_with_LEFT_POST:
		{
			if(calibration_wall_distance_right>140)
			{
				remaining_distance = 0.065;
				calibration_state = CALIBRATION_END;
				return true;
			}
			if(calibration_wall_distance_left>140)
			{
				remaining_distance = 0.070;
				calibration_state = CALIBRATION_END;
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

// AI helper
void first_move_state()
{
	if(sizeof(move_plan)>0)
	{
		move_plan_step = 0;
		move_state = move_plan[move_plan_step];
	}
	else
	{
		// check walls
		move_state = MOVE_START;
	}
	submove_state = SUBMOVE_0;
}

void next_move_state()
{
	if(sizeof(move_plan)>0)
	{
		++move_plan_step;
		move_state = move_plan[move_plan_step];

	}
	else
	{

		// check walls
		bool LEFT_wall = HAL_WallSensor_Get(WALL_SENSOR_LEFT_DIAG) <= 120;
		bool FL_wall = HAL_WallSensor_Get(WALL_SENSOR_LEFT_STRAIGHT) <= 95;
		bool FR_wall = HAL_WallSensor_Get(WALL_SENSOR_RIGHT_STRAIGHT) <= 95;
		bool RIGHT_wall = HAL_WallSensor_Get(WALL_SENSOR_RIGHT_DIAG) <= 120;

		AI_Maze_Wall_Detected(LEFT_wall, FL_wall || FR_wall, RIGHT_wall);
		AI_MAZE_ACTION_DIRECTION move = AI_Maze_Get_Next_Move();
		switch(move)
		{
		case AI_MAZE_RELATIVE_DIRECTION_TURN_LEFT :
			move_state = MOVE_SS_LEFT_90;
			break;
		case AI_MAZE_RELATIVE_DIRECTION_GO_FORWARD :
			move_state = MOVE_SS_FORWARD;
			break;
		case AI_MAZE_RELATIVE_DIRECTION_TURN_RIGHT :
			move_state = MOVE_SS_RIGHT_90;
			break;
		case AI_MAZE_RELATIVE_DIRECTION_TURN_BACK_WITH_FRONT_AND_RIGHT_OR_LEFT_WALL_CALIBRATION:
		case AI_MAZE_RELATIVE_DIRECTION_TURN_BACK_WITH_FRONT_AND_RIGHT_WALL_CALIBRATION:
		case AI_MAZE_RELATIVE_DIRECTION_TURN_BACK_WITH_FRONT_AND_LEFT_WALL_CALIBRATION:
		case AI_MAZE_RELATIVE_DIRECTION_TURN_BACK_WITH_RIGHT_OR_LEFT_WALL_CALIBRATION:
		case AI_MAZE_RELATIVE_DIRECTION_TURN_BACK_WITH_RIGHT_WALL_CALIBRATION:
		case AI_MAZE_RELATIVE_DIRECTION_TURN_BACK_WITH_LEFT_WALL_CALIBRATION:
		case AI_MAZE_RELATIVE_DIRECTION_TURN_BACK_WITH_FRONT_WALL_CALIBRATION:
		case AI_MAZE_RELATIVE_DIRECTION_TURN_BACK_WITHOUT_WALL_CALIBRATION:
			move_state = MOVE_U_TURN;
			break;
		case AI_MAZE_RELATIVE_DIRECTION_TURN_BACK_AND_STOP:
		case AI_MAZE_RELATIVE_DIRECTION_STOP_IN_MIDDLE_OF_THE_CELL:
		case AI_MAZE_RELATIVE_DIRECTION_STOP:
			move_state = MOVE_FINISH;
			break;
		}
	}
	submove_state = SUBMOVE_0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  HAL_Motor_Init();
  HAL_Motor_Set(HAL_MOTOR_ALL,HAL_MOTOR_AUTO,0);
  HAL_Battery_Init();
  HAL_Encoder_Init(&front_left_encoder,&htim5,TIMER_32,CW);
  HAL_Encoder_Init(&front_right_encoder,&htim4,TIMER_16,CW);
  HAL_Encoder_Init(&back_left_encoder,&htim2,TIMER_32,CW);
  HAL_Encoder_Init(&back_right_encoder,&htim3,TIMER_16,CCW);
  HAL_IMU_Init();
  HAL_IMU_Add(
		&imu,
		&hi2c3,
		&gyro_sensitivity_correction
	);
  HAL_Serial_Init(&huart1,&serial);
  HAL_Serial_Print(&serial,"Thesee V Wall Maze Breaker [Reset]\r\n");
  HAL_MMI_Init(&serial);
  HAL_MMI_Entry mmi[] = {

		  {"gsc", MMI_ENTRY_DATA,          {.dptr=&gyro_sensitivity_correction},           1000	},
		  {"xkp", MMI_ENTRY_DATA,          {.dptr=&pid_x_kp},           1	},
		  {"xki", MMI_ENTRY_DATA,          {.dptr=&pid_x_ki},           1 	},
		  {"xkd", MMI_ENTRY_DATA,          {.dptr=&pid_x_kd},           1	},
		  {"xkf", MMI_ENTRY_DATA,          {.dptr=&pid_x_kff},           1	},
		  {"wkp", MMI_ENTRY_DATA,          {.dptr=&pid_w_kp},           1000},
		  {"wki", MMI_ENTRY_DATA,          {.dptr=&pid_w_ki},           1000},
		  {"wkd", MMI_ENTRY_DATA,          {.dptr=&pid_w_kd},           1000},
		  {"wkf", MMI_ENTRY_DATA,          {.dptr=&pid_w_kff},          1000},
		  {"wcs",  	MMI_ENTRY_DATA,		   {.dptr=&w_curve_speed},		1	},
		  {"wcd1",  MMI_ENTRY_DATA,		   {.dptr=&w_curve_d1},		1000	},
		  {"wcd2",  MMI_ENTRY_DATA,		   {.dptr=&w_curve_d2},		1000	},
		  {"wct1",  MMI_ENTRY_DATA,		   {.dptr=&w_curve_t1},		1000	},
		  {"wct2",  MMI_ENTRY_DATA,		   {.dptr=&w_curve_t2},		1000	},
		  {"wctu",  MMI_ENTRY_DATA,		   {.dptr=&w_uturn180_t2},		1000	},
		  {"wkir", MMI_ENTRY_DATA,          {.dptr=&pid_w_kir},           1	},
		  {"L"  , MMI_ENTRY_FUNCTION,      {.fptr=&send_log},1   },
		  {"R"  , MMI_ENTRY_FUNCTION,      {.fptr=&start},				1   },
		  {"xirkp",  MMI_ENTRY_DATA,		   {.dptr=&pid_xir_kp},		1000	},
		  {"wirkp",  MMI_ENTRY_DATA,		   {.dptr=&pid_wir_kp},		1000	},
		  {0}
  };
  HAL_MMI_Configure(mmi);
  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // OFF LED1
  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // OFF LED2
  HAL_TIM_Base_Start(&htim6); // This timer is used as a time base for main processing (1 cout = 1us)
  uint16_t last_time_us = __HAL_TIM_GET_COUNTER(&htim6);
  HAL_WallSensor_Init();
  HAL_DataLogger_Init(8,4,4,4,4,4,4,4,4);

  // controller
  reset_ewma(&ewma_x_speed);
  reset_ewma(&ewma_w_speed);
  reset_pid_mm(&pid_x_speed);
  reset_pid_mm(&pid_w_speed);
  // buzzer
	HAL_Buzzer_Init();
	{
//		HAL_Melody_Note notes[] = {
//		{NOTE_C4,200,1000},
//		{NOTE_G4,200,1000},
//		{NOTE_F4,200,250},
//		{NOTE_E4,200,250},
//		{NOTE_D4,200,250},
//		{NOTE_C5,200,1000},
//		{NOTE_G4,200,500},
//		{NOTE_F4,200,250},
//		{NOTE_E4,200,250},
//		{NOTE_D4,200,250},
//		{NOTE_C5,200,1000},
//		{NOTE_G4,200,500},
//		{NOTE_F4,200,250},
//		{NOTE_E4,200,250},
//		{NOTE_F4,200,250},
//		{NOTE_D4,200,2000}
//		};
//		HAL_Melody_Add(notes,16);
		HAL_Melody_Note notes[] = {
			{NOTE_C4,200,200},
			{NOTE_F4,200,200},
			{NOTE_C4,200,200}
		};
		HAL_Melody_Add(notes,3);
	}
	HAL_Melody_Play();
	HAL_GPIO_WritePin(IR_LED_FR_GPIO_Port,IR_LED_FR_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IR_LED_FL_GPIO_Port,IR_LED_FL_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IR_LED_DR_GPIO_Port,IR_LED_DR_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IR_LED_DL_GPIO_Port,IR_LED_DL_Pin,GPIO_PIN_RESET);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  HAL_Melody_Process();

	// 833Hz processing
	uint16_t current_time_us = __HAL_TIM_GET_COUNTER(&htim6);
	uint16_t delta_time_us = current_time_us-last_time_us;
	if(delta_time_us>=1200) //! 833Hz (ODR)
	{
		// period
		float period_us = (float)(delta_time_us)/1000000.0f; //s
		last_time_us = current_time_us;


		uint16_t t0 = __HAL_TIM_GET_COUNTER(&htim6);

		// sensors
		HAL_IMU_Read_Sensors(&imu,state==IDLE || state==WARMUP,period_us);

		uint16_t t1 = __HAL_TIM_GET_COUNTER(&htim6);

		HAL_WallSensor_Process();

		uint16_t t2 = __HAL_TIM_GET_COUNTER(&htim6);

		// controller
		float const ratio = PI_FLOAT * 0.026 / ( 600.0 *  period_us );
		float encoder_sum = process_ewma( &ewma_x_speed, (float)(HAL_Encoder_Delta(&front_left_encoder) +
							HAL_Encoder_Delta(&front_right_encoder) +
							HAL_Encoder_Delta(&back_left_encoder) +
							HAL_Encoder_Delta(&back_right_encoder)) );
		actual_x_speed = ( encoder_sum * ratio ) / 4.0f;
		//actual_w_speed = process_ewma( &ewma_w_speed, imu.scaled_sensor_data[GYR_Z] );
		actual_w_speed = imu.rate;
		actual_heading = imu.heading;

		switch(state)
		{
		case IDLE:
			{
				reset_speed_control();

				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // OFF LED1
				HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // OFF LED2

				if(HAL_GPIO_ReadPin(BUTTON3_GPIO_Port, BUTTON3_Pin) == GPIO_PIN_RESET && !HAL_Battery_Is_Low())
				{
					state=WARMUP;
					state_time = HAL_GetTick();


				}
			}
			break;
		case WARMUP:
			{
				reset_speed_control();

				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET); // ON LED1
				HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET); // ON LED2

				if(HAL_GetTick()>(state_time+1500))
				{
					state=RUNNING;
					state_time = HAL_GetTick();

					first_move_state();

					HAL_DataLogger_Clear();
					HAL_IMU_Zero_Heading(&imu);
					target_heading = 0.0f;
				}
			}
			break;
		case RUNNING:
			{
				HAL_DataLogger_Record(8,
					(int32_t)(current_x_speed*1000.0f),
					(int32_t)(actual_x_speed*1000.0f),
					(int32_t)(pwm_x_speed*1000.0f),
					(int32_t)(current_w_speed*1000.0f),
					(int32_t)(actual_w_speed*1000.0f),
					(int32_t)(pwm_w_speed*1000.0f),
					(int32_t)(actual_heading*1000.0f),
					0
				);
//				HAL_DataLogger_Record(8,
//					(int32_t)(move_state),
//					(int32_t)(remaining_distance),
//					(int32_t)(calibration_wall_distance_left*1000.0f),
//					(int32_t)(calibration_wall_distance_right*1000.0f),
//					(int32_t)(calibration_state),
//					(int32_t)(0*1000.0f),
//					(int32_t)(0*1000.0f),
//					(int32_t)(actual_heading*1000.0f)
//				);



				switch(move_state)
				{
				case MOVE_IDLE:
					{
						reset_speed_control();

						HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET); // ON LED1
						HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET); // ON LED2

						state=STOP;
					}
					break;

				case MOVE_START:
					{
						switch(submove_state)
						{
						case SUBMOVE_0:
							{
								process_speed_control_with_wall_following(0.25f, 0.0f, delta_time_us);

								remaining_distance = 0.12f;
								submove_state = SUBMOVE_1;

								calibration_reset();
							}
							break;
						case SUBMOVE_1:
							{
								process_speed_control_with_wall_following(0.25f, 0.0f, delta_time_us);

								remaining_distance -= actual_x_speed*period_us;

								//calibration_process(); // not enough distance

								if(remaining_distance<0.11)
								{
									HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // OFF LED1
									HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // OFF LED2
								}

								if(remaining_distance<=0.0f)
								{

									if(sizeof(move_plan)==0)
										AI_Maze_Start();
									next_move_state();

									HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET); // ON LED1
									HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET); // ON LED2
								}

							}
							break;
						}
					}
					break;
				case MOVE_FINISH:
					{
						switch(submove_state)
						{
						case SUBMOVE_0:
							{
								process_speed_control_with_wall_following(0.25f, 0.0f, delta_time_us);

								remaining_distance += 0.08f;
								submove_state = SUBMOVE_1;
							}
							break;
						case SUBMOVE_1:
							{
								process_speed_control_with_wall_following(0.25f, 0.0f, delta_time_us);

								if(remaining_distance<0.07)
								{
									HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // OFF LED1
									HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // OFF LED2
								}

								remaining_distance -= actual_x_speed*period_us;

								if(-x_deceleration>=accelaration_until_distance_left(current_x_speed,0.0f,remaining_distance))
								{
									submove_state = SUBMOVE_2;
								}

							}
							break;
						case SUBMOVE_2:
							{
								process_speed_control(0.01f, 0.0f, delta_time_us);

								remaining_distance -= actual_x_speed*period_us;

								if(remaining_distance<=0.0f)
								{
									//next_move_state();

									reset_speed_control();

									HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET); // ON LED1
									HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET); // ON LED2

									state=STOP;
								}

							}
							break;
						}
					}
					break;
				case MOVE_SS_FORWARD:
					{
						switch(submove_state)
						{
						case SUBMOVE_0:
							{
								process_speed_control_with_wall_following(0.25f, 0.0f, delta_time_us);

								remaining_distance += 0.18f;
								submove_state = SUBMOVE_1;

								calibration_reset();

							}
							break;
						case SUBMOVE_1:
							{
								process_speed_control_with_wall_following(0.25f, 0.0f, delta_time_us);

								if(remaining_distance<0.17)
								{
									HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // OFF LED1
									HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // OFF LED2
								}


								remaining_distance -= actual_x_speed*period_us;

								calibration_process();

								if(remaining_distance<=0.0f)
								{
									if(sizeof(move_plan)==0)
										AI_Maze_Move_Forward();
									next_move_state();

									HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET); // ON LED1
									HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET); // ON LED2

								}

							}
							break;
						}
					}
					break;
				case MOVE_SS_LEFT_90:
				case MOVE_SS_RIGHT_90 :
					{
						switch(submove_state)
						{
						case SUBMOVE_0:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								//! front wall assist
								//float front_wall_distance = ((float)HAL_WallSensor_Get(WALL_SENSOR_LEFT_STRAIGHT)+(float)HAL_WallSensor_Get(WALL_SENSOR_RIGHT_STRAIGHT))/2.0f;
								//remaining_distance = (front_wall_distance - 90.0f)/1000.0f; //0.01f;
								//if(remaining_distance>0.01f)
								remaining_distance = w_curve_d1;
								target_heading = actual_heading + (move_state==MOVE_SS_LEFT_90?1.0f:-1.0f)*90.0f;
								submove_state = SUBMOVE_1;
							}
							break;
						case SUBMOVE_1:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								remaining_distance -= actual_x_speed*period_us;
								if(remaining_distance<=0.0f)
								{
									HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // OFF LED1
									HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // OFF LED2
									submove_state = SUBMOVE_2;
								}

							}
							break;
						case SUBMOVE_2:
							{
								process_speed_control(0.25f, (move_state==MOVE_SS_LEFT_90?1.0f:-1.0f)*w_curve_speed, delta_time_us);

								remaining_time = w_curve_t1+w_curve_t2; //T1+T2 pour rayon de courbure de 80 mm à vitesses 0.25/190
								submove_state = SUBMOVE_3;

							}
							break;
						case SUBMOVE_3:
							{
								process_speed_control(0.25f, (move_state==MOVE_SS_LEFT_90?1.0f:-1.0f)*w_curve_speed, delta_time_us);

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									remaining_time += w_curve_t1; // T1=T3
									submove_state = SUBMOVE_4;
								}
							}
							break;
						case SUBMOVE_4:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									remaining_distance = w_curve_d2;
									submove_state = SUBMOVE_5;
								}

							}
							break;
						case SUBMOVE_5:
							{
								process_speed_control(0.25f, (target_heading-actual_heading)*5.0f, delta_time_us);
								//process_speed_control(0.25f, 0.0f, delta_time_us);

								remaining_distance -= actual_x_speed*period_us;
								if(remaining_distance<=0.0f)
								{
									if(sizeof(move_plan)==0)
									{
										if(move_state == MOVE_SS_LEFT_90)

											AI_Maze_Turn_Left();
										else
											AI_Maze_Turn_Right();
									}
									next_move_state();

									HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET); // ON LED1
									HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET); // ON LED2
								}

							}
							break;
						}
					}
					break;
				case MOVE_U_TURN:
					{
						switch(submove_state)
						{

						//! Run to the middle of the cell
						case SUBMOVE_0:
							{
								cell_has_left_wall = HAL_WallSensor_Get(WALL_SENSOR_LEFT_DIAG) < 110;
								cell_has_front_wall = (HAL_WallSensor_Get(WALL_SENSOR_LEFT_STRAIGHT) < 130) && (HAL_WallSensor_Get(WALL_SENSOR_RIGHT_STRAIGHT) < 130);
								cell_has_right_wall = HAL_WallSensor_Get(WALL_SENSOR_RIGHT_DIAG) < 110;

								process_speed_control_with_wall_following(0.25f, 0.0f, delta_time_us);

								remaining_distance = 0.09f;
								submove_state = SUBMOVE_1;
							}
							break;
						case SUBMOVE_1:
							{
								process_speed_control_with_wall_following(0.25f, 0.0f, delta_time_us);

								if(remaining_distance<0.08)
								{
									HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // OFF LED1
									HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // OFF LED2
								}

								remaining_distance -= actual_x_speed*period_us;

								if(-x_deceleration>=accelaration_until_distance_left(current_x_speed,0.0f,remaining_distance))
								{
									submove_state = SUBMOVE_2;
								}

							}
							break;
						case SUBMOVE_2:
							{
								process_speed_control(0.02f, 0.0f, delta_time_us);

								remaining_distance -= actual_x_speed*period_us;

								if(remaining_distance<=0.0f)
								{
									submove_state = SUBMOVE_3;
								}
							}
							break;

							//! Stop / Pause in the middle of the cell
						case SUBMOVE_3:
							{
								process_speed_control(0.0f, 0.0f, delta_time_us);

								remaining_time = 0.1;
								submove_state = SUBMOVE_4;
							}
							break;
						case SUBMOVE_4:
							{
								process_speed_control(0.0f, 0.0f, delta_time_us);

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									if(cell_has_front_wall)
									{
										reset_speed_control();
										submove_state = SUBMOVE_5;
									}
									else if(cell_has_right_wall)
									{
										remaining_time = w_uturn90_t1+w_uturn90_t2;
										submove_state = SUBMOVE_10;
									}
									else if(cell_has_left_wall)
									{
										remaining_time = w_uturn90_t1+w_uturn90_t2;
										submove_state = SUBMOVE_20;
									}
									else
									{
										remaining_time = w_uturn90_t1+w_uturn180_t2;
										submove_state = SUBMOVE_7;
									}
								}

							}
							break;

							//! Front Wall Calibration
						case SUBMOVE_5:
							{
								process_speed_control_with_front_wall_following();

								remaining_time = 0.5f;
								submove_state = SUBMOVE_6;


							}
							break;
						case SUBMOVE_6:
							{
								process_speed_control_with_front_wall_following();

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									if(cell_has_right_wall)
									{
										remaining_time = w_uturn90_t1+w_uturn90_t2;
										submove_state = SUBMOVE_10;
									}
									else if(cell_has_left_wall)
									{
										remaining_time = w_uturn90_t1+w_uturn90_t2;
										submove_state = SUBMOVE_20;
									}
									else
									{
										remaining_time = w_uturn90_t1+w_uturn180_t2;
										submove_state = SUBMOVE_7;
									}
								}

							}
							break;


						//! 180 Turn without wall
						case SUBMOVE_7:
							{
								process_speed_control(0.0f, -w_curve_speed, delta_time_us);

								remaining_time -= period_us;

								if(remaining_time<=0.0f)
								{
									remaining_time = w_uturn90_t1;
									submove_state = SUBMOVE_8;
								}
							}
							break;
						case SUBMOVE_8:
							{
								process_speed_control(0.0f, 0.0f, delta_time_us);

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									remaining_time = 0.1f;
									submove_state = SUBMOVE_9;
								}

							}
							break;
						case SUBMOVE_9:
							{
								process_speed_control(0.0f, 0.0f, delta_time_us);

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									submove_state = SUBMOVE_29;
								}
							}
							break;

						//! 180 Turn with right wall calibration
						case SUBMOVE_10:
							{
								process_speed_control(0.0f, -w_curve_speed, delta_time_us);

								remaining_time -= period_us;

								if(remaining_time<=0.0f)
								{
									remaining_time = w_uturn90_t1;
									submove_state = SUBMOVE_11;
								}
							}
							break;
						case SUBMOVE_11:
							{
								process_speed_control(0.0f, 0.0f, delta_time_us);

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									remaining_time = 0.1f;
									submove_state = SUBMOVE_12;
								}

							}
							break;
						case SUBMOVE_12:
							{
								process_speed_control(0.0f, 0.0f, delta_time_us);

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									submove_state = SUBMOVE_13;
								}

							}
							break;
						case SUBMOVE_13:
							{
								process_speed_control_with_front_wall_following();

								remaining_time = 0.5f;
								submove_state = SUBMOVE_14;

							}
							break;
						case SUBMOVE_14:
							{
								process_speed_control_with_front_wall_following();

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									remaining_time = w_uturn90_t1+w_uturn90_t2;
									submove_state = SUBMOVE_15;
								}

							}
							break;
						case SUBMOVE_15:
							{
								process_speed_control(0.0f, -w_curve_speed, delta_time_us);

								remaining_time -= period_us;

								if(remaining_time<=0.0f)
								{
									remaining_time = w_uturn90_t1;
									submove_state = SUBMOVE_16;
								}
							}
							break;
						case SUBMOVE_16:
							{
								process_speed_control(0.0f, 0.0f, delta_time_us);

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									remaining_time = 0.1f;
									submove_state = SUBMOVE_17;
								}

							}
							break;
						case SUBMOVE_17:
							{
								process_speed_control(0.0f, 0.0f, delta_time_us);

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									submove_state = SUBMOVE_29;
								}

							}
							break;


						//! 180 Turn with left wall callibration
						case SUBMOVE_20:
							{
								process_speed_control(0.0f, w_curve_speed, delta_time_us);

								remaining_time -= period_us;

								if(remaining_time<=0.0f)
								{
									remaining_time = w_uturn90_t1;
									submove_state = SUBMOVE_21;
								}
							}
							break;
						case SUBMOVE_21:
							{
								process_speed_control(0.0f, 0.0f, delta_time_us);

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									remaining_time = 0.1f;
									submove_state = SUBMOVE_22;
								}

							}
							break;
						case SUBMOVE_22:
							{
								process_speed_control(0.0f, 0.0f, delta_time_us);

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									submove_state = SUBMOVE_23;
								}

							}
							break;
						case SUBMOVE_23:
							{
								process_speed_control_with_front_wall_following();

								remaining_time = 0.5f;
								submove_state = SUBMOVE_24;

								calibration_reset();
							}
							break;
						case SUBMOVE_24:
							{
								process_speed_control_with_front_wall_following();

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									remaining_time = w_uturn90_t1+w_uturn90_t2;
									submove_state = SUBMOVE_25;
								}

							}
							break;
						case SUBMOVE_25:
							{
								process_speed_control(0.0f, w_curve_speed, delta_time_us);

								remaining_time -= period_us;

								if(remaining_time<=0.0f)
								{
									remaining_time = w_uturn90_t1;
									submove_state = SUBMOVE_26;
								}
							}
							break;
						case SUBMOVE_26:
							{
								process_speed_control(0.0f, 0.0f, delta_time_us);

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									remaining_time = 0.1f;
									submove_state = SUBMOVE_27;
								}

							}
							break;
						case SUBMOVE_27:
							{
								process_speed_control(0.0f, 0.0f, delta_time_us);

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									submove_state = SUBMOVE_29;
								}

							}
							break;

						//! Get out of the cell
						case SUBMOVE_29:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								remaining_distance = 0.09f;

								submove_state = SUBMOVE_30;
							}
							break;
						case SUBMOVE_30:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								remaining_distance -= actual_x_speed*period_us;
								if(remaining_distance<=0.0f)
								{
									if(sizeof(move_plan)==0)
										AI_Maze_UTurn();
									next_move_state();

									HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET); // ON LED1
									HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET); // ON LED2

								}
							}
							break;

						}
					}
					break;










				case MOVE_DD_FORWARD:
					{
						switch(submove_state)
						{
						case SUBMOVE_0:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET); // ON LED2

								remaining_distance += 0.13f;
								submove_state = SUBMOVE_1;
							}
							break;
						case SUBMOVE_1:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET); // ON LED2

								remaining_distance -= actual_x_speed*period_us;
								if(remaining_distance<=0.0f)
								{
									++move_plan_step;
									submove_state = SUBMOVE_0;
								}

							}
							break;
						}
					}
					break;

				case MOVE_SD_RIGHT_45:
					{
						switch(submove_state)
						{
						case SUBMOVE_0:
							{
								process_speed_control(0.25f, -w_curve_speed, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_time = w_d45_t1+w_d45_t2; //T1+T2
								submove_state = SUBMOVE_1;

							}
							break;
						case SUBMOVE_1:
							{
								process_speed_control(0.25f, -w_curve_speed, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									remaining_time = w_d45_t1; // T1=T3
									submove_state = SUBMOVE_2;
								}
							}
							break;
						case SUBMOVE_2:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									remaining_distance = 0.075f;
									submove_state = SUBMOVE_3;
								}
							}
							break;
						case SUBMOVE_3:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_distance -= actual_x_speed*period_us;
								if(remaining_distance<=0.0f)
								{
									++move_plan_step;
									submove_state = SUBMOVE_0;
								}
							}
							break;
						}

					}
					break;
				case MOVE_SD_LEFT_135:
					{
						switch(submove_state)
						{
						case SUBMOVE_0:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_distance += 0.04f;
								submove_state = SUBMOVE_1;
							}
							break;
						case SUBMOVE_1:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_distance -= actual_x_speed*period_us;
								if(remaining_distance<=0.0f)
								{
									submove_state = SUBMOVE_2;
								}

							}
							break;
						case SUBMOVE_2:
							{
								process_speed_control(0.25f, w_curve_speed, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_time = w_d45_t1+w_d135_t2; //T1+T2
								submove_state = SUBMOVE_3;

							}
							break;
						case SUBMOVE_3:
							{
								process_speed_control(0.25f, w_curve_speed, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									remaining_time = w_d135_t1; // T1=T3
									submove_state = SUBMOVE_4;
								}
							}
							break;
						case SUBMOVE_4:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									remaining_distance += 0.09f;
									submove_state = SUBMOVE_5;
								}
							}
							break;
						case SUBMOVE_5:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_distance -= actual_x_speed*period_us;
								if(remaining_distance<=0.0f)
								{
									++move_plan_step;
									submove_state = SUBMOVE_0;
								}
							}
							break;
						}
					}
					break;
				case MOVE_DD_RIGHT_90:
					{
						switch(submove_state)
						{
						case SUBMOVE_0:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_distance += 0.06f;
								submove_state = SUBMOVE_1;
							}
							break;
						case SUBMOVE_1:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_distance -= actual_x_speed*period_us;
								if(remaining_distance<=0.0f)
								{
									submove_state = SUBMOVE_2;
								}

							}
							break;
						case SUBMOVE_2:
							{
								process_speed_control(0.25f, -w_curve_speed, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_time = w_d90_t1+w_d90_t2; //T1+T2
								submove_state = SUBMOVE_3;

							}
							break;
						case SUBMOVE_3:
							{
								process_speed_control(0.25f, -w_curve_speed, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									remaining_time = w_d90_t1; // T1=T3
									submove_state = SUBMOVE_4;
								}
							}
							break;
						case SUBMOVE_4:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									remaining_distance += 0.05f;
									submove_state = SUBMOVE_5;
								}
							}
							break;
						case SUBMOVE_5:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_distance -= actual_x_speed*period_us;
								if(remaining_distance<=0.0f)
								{
									++move_plan_step;
									submove_state = SUBMOVE_0;
								}
							}
							break;
						}




					}
					break;
				case MOVE_DS_RIGHT_45:
					{
						switch(submove_state)
						{
						case SUBMOVE_0:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_distance += 0.075f;
								submove_state = SUBMOVE_1;
							}
							break;
						case SUBMOVE_1:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_distance -= actual_x_speed*period_us;
								if(remaining_distance<=0.0f)
								{
									submove_state = SUBMOVE_2;
								}
							}
							break;
						case SUBMOVE_2:
							{
								process_speed_control(0.25f, -w_curve_speed, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_time = w_d45_t1+w_d45_t2; //T1+T2
								submove_state = SUBMOVE_3;

							}
							break;
						case SUBMOVE_3:
							{
								process_speed_control(0.25f, -w_curve_speed, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									remaining_time = w_d45_t1; // T1=T3
									submove_state = SUBMOVE_4;
								}
							}
							break;
						case SUBMOVE_4:
							{
								process_speed_control(0.25f, 0.0f, delta_time_us);

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // ON LED2

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									++move_plan_step;
									submove_state = SUBMOVE_0;
									remaining_distance -= 0.06f;
								}
							}
							break;
						}
					}
					break;



				case MOVE_FOLLOW_FRONT_WALL:
					{
						switch(submove_state)
						{
						case SUBMOVE_0:
							{
								process_speed_control_with_front_wall_following();

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET); // ON LED2

								remaining_time = 5.0f;
								submove_state = SUBMOVE_1;

							}
							break;
						case SUBMOVE_1:
							{
								process_speed_control_with_front_wall_following();

								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET); // ON LED2

								remaining_time -= period_us;
								if(remaining_time<=0.0f)
								{
									++move_plan_step;
									submove_state = SUBMOVE_0;
								}

							}
							break;

						}

					}
					break;

				case MOVE_STEP_RESPONSE_X:
					{
						switch(submove_state)
						{
						case SUBMOVE_0:
							{
								HAL_Motor_Set(HAL_MOTOR_LEFT,HAL_MOTOR_AUTO,800);
								HAL_Motor_Set(HAL_MOTOR_RIGHT,HAL_MOTOR_AUTO,800);
								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET); // ON LED2

								remaining_distance = 0.36f;
								submove_state = SUBMOVE_1;
							}
							break;
						case SUBMOVE_1:
							{
								HAL_Motor_Set(HAL_MOTOR_LEFT,HAL_MOTOR_AUTO,800);
								HAL_Motor_Set(HAL_MOTOR_RIGHT,HAL_MOTOR_AUTO,800);
								HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET); // ON LED1
								HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET); // ON LED2

								remaining_distance -= actual_x_speed*period_us;
								if(remaining_distance<=0.0f)
								{
									++move_plan_step;
									submove_state = SUBMOVE_0;
								}

							}
							break;
						}
					}
					break;
				}
			}
			break;
		case STOP:
			{
				target_x_speed = 0.0f;
				target_w_speed = 0.0f;
				current_x_speed = 0.0f;
				current_w_speed = 0.0f;
				pwm_x_speed = 0.0f;
				pwm_w_speed = 0.0f;
				pwm_left = pwm_x_speed - pwm_w_speed;
				pwm_right = pwm_x_speed + pwm_w_speed;
				HAL_Motor_Set(HAL_MOTOR_LEFT,HAL_MOTOR_AUTO,pwm_left);
				HAL_Motor_Set(HAL_MOTOR_RIGHT,HAL_MOTOR_AUTO,pwm_right);
				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); // OFF LED1
				HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); // OFF LED2

				state=IDLE;
				HAL_Melody_Play();
			}
			break;
		}

		uint16_t t3 = __HAL_TIM_GET_COUNTER(&htim6);

		// TRACE
		static unsigned int trace_counter = 0;
		if(++trace_counter%10==0) // reduce trace rate
		{
			HAL_MMI_Process();

//			HAL_Serial_Print(&serial,"%d %d (bias:%d) (deviation:%d) dt=%d %d %d %d\r\n",
//					(int32_t)(imu.rate*1000.0f),
//					(int32_t)(imu.heading),
//					(int32_t)(imu.rate_bias),
//					(int32_t)(imu.rate_deviation),
//					delta_time_us,
//					t1-t0,
//					t2-t1,
//					t3-t2
//			);
//
//			HAL_Serial_Print(&serial,"%d  %d  %d \r\n",
//					//(int32_t)(imu.raw_rate),
//					//(int32_t)(imu.rate_mean),
//					(int32_t)(imu.rate_deviation),
//					(int32_t)(imu.rate_bias),
//					//(int32_t)(imu.rate*1000.0f),
//					(int32_t)(imu.heading) //(int32_t)(imu.heading*1000.0f)
//					//delta_time_us
//			);


//			HAL_Serial_Print(&serial,"%d\r\n",
//					//(int32_t)(imu.raw_rate),
//					//(int32_t)(imu.rate_mean),
//					//(int32_t)(imu.rate_deviation),
//					//(int32_t)(imu.rate_bias),
//					//(int32_t)(imu.rate*1000.0f),
//					(int32_t)(imu.heading*1.0f)
//					//delta_time_us
//			);

			//			HAL_Serial_Print(&serial,"%d (%d)\r\n",(int32_t)(actual_heading),(int32_t)(target_heading));



//			HAL_Serial_Print(&serial,"%d %d %d %d %d %d\r\n",
//				delta_time_us,
//				HAL_Encoder_Delta(&front_left_encoder),
//				HAL_Encoder_Delta(&front_right_encoder),
//				HAL_Encoder_Delta(&back_left_encoder),
//				HAL_Encoder_Delta(&back_right_encoder),
//				(int32_t)(imu.scaled_sensor_data[GYR_Z])
//			);

//			HAL_Serial_Print(&serial,"%d %d %d %d %d %d %d %d\r\n",
//					(int32_t)(target_x_speed*1000.0f),
//					(int32_t)(current_x_speed*1000.0f),
//					(int32_t)(actual_x_speed*1000.0f),
//					(int32_t)(pwm_x_speed*1000.0f),
//					(int32_t)(target_w_speed*1000.0f),
//					(int32_t)(current_w_speed*1000.0f),
//					(int32_t)(actual_w_speed*1000.0f),
//					(int32_t)(pwm_w_speed*1000.0f)
//			);

//			HAL_Serial_Print(&serial,"%d %d %d\r\n",
//					move_state,
//					submove_state,
//					(int32_t)(remaining_distance*1000.0f)
//				);

//						HAL_Serial_Print(&serial,"%d %d %d %d\r\n",
//								HAL_WallSensor_Get_Raw(WALL_SENSOR_LEFT_DIAG),
//								HAL_WallSensor_Get_Raw(WALL_SENSOR_LEFT_STRAIGHT),
//								HAL_WallSensor_Get_Raw(WALL_SENSOR_RIGHT_STRAIGHT),
//								HAL_WallSensor_Get_Raw(WALL_SENSOR_RIGHT_DIAG)
//							);
//						HAL_Serial_Print(&serial,"%d %d %d %d\r\n",
//							HAL_WallSensor_Get(WALL_SENSOR_LEFT_DIAG),
//							HAL_WallSensor_Get(WALL_SENSOR_LEFT_STRAIGHT),
//							HAL_WallSensor_Get(WALL_SENSOR_RIGHT_STRAIGHT),
//							HAL_WallSensor_Get(WALL_SENSOR_RIGHT_DIAG)
//						);

//									HAL_Serial_Print(&serial,"%d %d\r\n",
//										HAL_WallSensor_Get(WALL_SENSOR_LEFT_STRAIGHT),
//										HAL_WallSensor_Get(WALL_SENSOR_RIGHT_STRAIGHT)
//									);


		}
	}
	//HAL_Delay(1);
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C3;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC3 init function */
static void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x6000030D;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 6;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xffffffff;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 107;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xffff;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM9 init function */
static void MX_TIM9_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 107;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim9);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED2_Pin|RIGHT_DIR1_Pin|RIGHT_DIR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IR_LED_DR_Pin|IR_LED_DL_Pin|IR_LED_FR_Pin|IR_LED_FL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LEFT_DIR2_Pin|LEFT_DIR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_LED_DR_Pin IR_LED_DL_Pin IR_LED_FR_Pin IR_LED_FL_Pin */
  GPIO_InitStruct.Pin = IR_LED_DR_Pin|IR_LED_DL_Pin|IR_LED_FR_Pin|IR_LED_FL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LEFT_ENCODER_CA2_Pin */
  GPIO_InitStruct.Pin = LEFT_ENCODER_CA2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LEFT_ENCODER_CA2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RIGHT_ENCODER_A_Pin */
  GPIO_InitStruct.Pin = RIGHT_ENCODER_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RIGHT_ENCODER_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT_DIR2_Pin LEFT_DIR1_Pin */
  GPIO_InitStruct.Pin = LEFT_DIR2_Pin|LEFT_DIR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RIGHT_DIR1_Pin RIGHT_DIR2_Pin */
  GPIO_InitStruct.Pin = RIGHT_DIR1_Pin|RIGHT_DIR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON2_Pin BUTTON1_Pin */
  GPIO_InitStruct.Pin = BUTTON2_Pin|BUTTON1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON3_Pin */
  GPIO_InitStruct.Pin = BUTTON3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RIGHT_ENCODER_D_Pin IMU_INT2_Pin IMU_INT1_Pin */
  GPIO_InitStruct.Pin = RIGHT_ENCODER_D_Pin|IMU_INT2_Pin|IMU_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LEFT_ENCODER_AB4_Pin */
  GPIO_InitStruct.Pin = LEFT_ENCODER_AB4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LEFT_ENCODER_AB4_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

