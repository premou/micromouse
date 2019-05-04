/*
 * motor.c
 *
 *  Created on: 29 janv. 2019
 *      Author: Invite
 */

#include "main.h"
#include "personnalisation.h"
#include "robot_math.h"
#include "motor.h"

extern TIM_HandleTypeDef htim1;

void motor_init(){
	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
}

void motor_speed_right(float pwm){

	int32_t speed_right = (int32_t)constraint(pwm, -100, 100);
	//GESTION moteur DROIT
	if(speed_right>0 )
	{
		if(speed_right > 100){
			speed_right = 100;
		}
		//moteur allume DROIT avant
		HAL_GPIO_WritePin(RIGHT_DIR1_GPIO_Port,RIGHT_DIR1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RIGHT_DIR2_GPIO_Port,RIGHT_DIR2_Pin,GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,speed_right);
	}
	else if(speed_right<0)
	{
		if(speed_right < -100){
			speed_right = -100;
		}
		//moteur allume DROIT marche arriere
		HAL_GPIO_WritePin(RIGHT_DIR1_GPIO_Port,RIGHT_DIR1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(RIGHT_DIR2_GPIO_Port,RIGHT_DIR2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(-speed_right));
	}
	else if(speed_right==0)
	{
		//moteur eteint
		HAL_GPIO_WritePin(RIGHT_DIR1_GPIO_Port,RIGHT_DIR1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RIGHT_DIR2_GPIO_Port,RIGHT_DIR2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	}
}

void motor_speed_left(float pwm){

	int32_t speed_left = (int32_t)constraint(pwm, -100.0, 100.0);

	//GESTION moteur GAUCHE
	if(speed_left>0 )
	{
		if(speed_left > 100){
			speed_left = 100;
		}
#ifdef __PATRICK__
		//moteur allume GAUCHE avant
		HAL_GPIO_WritePin(LEFT_DIR1_GPIO_Port,LEFT_DIR1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(LEFT_DIR2_GPIO_Port,LEFT_DIR2_Pin,GPIO_PIN_RESET);
#else
		//moteur allume GAUCHE avant
		HAL_GPIO_WritePin(LEFT_DIR1_GPIO_Port,LEFT_DIR1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LEFT_DIR2_GPIO_Port,LEFT_DIR2_Pin,GPIO_PIN_SET);
#endif
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,speed_left);
	}
	else if(speed_left<0)
	{
		if(speed_left < -100){
			speed_left = -100;
		}
#ifdef __PATRICK__
		//moteur allume GAUCHE marche arriere
		HAL_GPIO_WritePin(LEFT_DIR1_GPIO_Port,LEFT_DIR1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LEFT_DIR2_GPIO_Port,LEFT_DIR2_Pin,GPIO_PIN_SET);
#else
		//moteur allume GAUCHE marche arriere
		HAL_GPIO_WritePin(LEFT_DIR1_GPIO_Port,LEFT_DIR1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(LEFT_DIR2_GPIO_Port,LEFT_DIR2_Pin,GPIO_PIN_RESET);
#endif
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(-speed_left));
	}
	else if(speed_left==0)
	{
		//moteur eteint
		HAL_GPIO_WritePin(LEFT_DIR1_GPIO_Port,LEFT_DIR1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LEFT_DIR2_GPIO_Port,LEFT_DIR2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	}
}
