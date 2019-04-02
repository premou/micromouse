/*
 * tone.c
 *
 *  Created on: 28 janv. 2019
 *      Author: Prem
 */


#include "tone.h"

static const uint32_t NOTE_A6 = 1760;
static const uint32_t NOTE_AS6 = 1865;
static const uint32_t NOTE_B6 = 1976;
static const uint32_t NOTE_E6 = 1319;
static const uint32_t NOTE_G6 = 1568;
static const uint32_t NOTE_C7 = 2093;
static const uint32_t NOTE_D7 = 2349;
static const uint32_t NOTE_E7 = 2637;
static const uint32_t NOTE_F7 = 2794;
static const uint32_t NOTE_G7 = 3136;
static const uint32_t NOTE_A7 = 3520;


void tone(TIM_HandleTypeDef* htim,uint32_t frequence, uint32_t delay, uint32_t channel){

//	  HAL_TIM_PWM_Start(htim, channel);
//	  //Frequence
//	  htim->Instance->PSC = frequence / 7;
//	  //Volume
//	  htim->Instance->CCR1 = 400;
//	  HAL_Delay(delay);
//	  HAL_TIM_PWM_Stop(htim, channel);
}


void play_startup_song(TIM_HandleTypeDef* htim, uint32_t channel){
	  for (int i=400;i>=0;i-=50){
		  tone(htim, i, 100, channel);
	  }
}

void play_finishing_song(TIM_HandleTypeDef* htim, uint32_t channel){
	  for (int i=0;i<=400;i+=50){
		  tone(htim, i, 100, channel);
	  }
}


void play_startup_song2(TIM_HandleTypeDef* htim, uint32_t channel){

	(void) NOTE_A6;
	(void) NOTE_AS6;
	(void) NOTE_B6;
	(void) NOTE_E6;
	(void) NOTE_D7;
	(void) NOTE_F7;
	(void) NOTE_A7;
	(void) NOTE_G6;

	tone(htim, NOTE_E7, 200, channel);
	HAL_Delay(100);

	tone(htim, NOTE_E7, 200, channel);
	HAL_Delay(100);

	tone(htim, NOTE_E7, 200, channel);
	HAL_Delay(100);

	tone(htim, NOTE_G7, 200, channel);
	HAL_Delay(50);

	tone(htim, NOTE_E7, 200, channel);
	HAL_Delay(100);

	tone(htim, NOTE_C7, 200, channel);
	HAL_Delay(200);

}


