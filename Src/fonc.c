/*
 * fonc.c
 *
 *  Created on: 8 nov. 2018
 *      Author: Invite
 */

#include "fonc.h"

extern TIM_HandleTypeDef htim1;


//code d'erreur
//1 : action non definie
//2 : context non defini
//appelée toutes les milisecondes
//plusieurs phases par action
/*
uint32_t exec_actions(action action, context_t* p_ctx)
{
	if(p_actions_scenario==NULL)
	{
		return 1;
	}

	if(p_ctx==NULL)
	{
		return 2;
	}


	switch (key) {
		case value:

			break;
		default:
			break;
	}
}
*/
/*
context_t* init_context(){

	context_t* ctx = (context_t*) malloc(sizeof(context_t));

	ctx->total_dist = 0;
	ctx->total_dist = 0;
	ctx->total_dist = 0;

	ctx->delta_dist = 0;
	ctx->delta_dist_left = 0;
	ctx->delta_dist_right = 0;

	ctx->time = HAL_GetTick();

	return ctx;
}
*/
/*
void free_context(context_t* ctx){
	free(ctx);
}
*/

void run(int32_t speed_right, int32_t speed_left)
{
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
		//HAL_GPIO_WritePin(RIGHT_PWM_GPIO_Port,RIGHT_PWM_Pin,GPIO_PIN_SET); //GPIO

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
		// HAL_GPIO_WritePin(RIGHT_PWM_GPIO_Port,RIGHT_PWM_Pin,GPIO_PIN_SET); //GPIO

	}
	else if(speed_right==0)
	{
		//moteur eteint
		HAL_GPIO_WritePin(RIGHT_DIR1_GPIO_Port,RIGHT_DIR1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RIGHT_DIR2_GPIO_Port,RIGHT_DIR2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
		//HAL_GPIO_WritePin(RIGHT_PWM_GPIO_Port,RIGHT_PWM_Pin,GPIO_PIN_SET); //GPIO

	}

	//GESTION moteur GAUCHE
	if(speed_left>0 )
	{
		if(speed_left > 100){
			speed_left = 100;
		}
		//moteur allume GAUCHE avant
		HAL_GPIO_WritePin(LEFT_DIR1_GPIO_Port,LEFT_DIR1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LEFT_DIR2_GPIO_Port,LEFT_DIR2_Pin,GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,speed_left);
		//HAL_GPIO_WritePin(LEFT_PWM_GPIO_Port,LEFT_PWM_Pin,GPIO_PIN_SET); //GPIO

	}
	else if(speed_left<0)
	{
		if(speed_left < -100){
			speed_left = -100;
		}
		//moteur allume GAUCHE marche arriere
		HAL_GPIO_WritePin(LEFT_DIR1_GPIO_Port,LEFT_DIR1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(LEFT_DIR2_GPIO_Port,LEFT_DIR2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(-speed_left));
		// HAL_GPIO_WritePin(LEFT_PWM_GPIO_Port,LEFT_PWM_Pin,GPIO_PIN_SET); //GPIO

	}
	else if(speed_left==0)
	{
		//moteur eteint
		HAL_GPIO_WritePin(LEFT_DIR1_GPIO_Port,LEFT_DIR1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LEFT_DIR2_GPIO_Port,LEFT_DIR2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		//HAL_GPIO_WritePin(LEFT_PWM_GPIO_Port,LEFT_PWM_Pin,GPIO_PIN_SET); //GPIO

	}
}


/*
 * Return speed in m/s
 */
float get_speed(float dist, uint32_t time){
	return dist*1000/time;
}




 //TODO : void avancer_case(int32_t speed)
/* param
 * in  : int32_t t : temps écoulé en seconde
 * out : float * vitesse : vitesse courante des roues en m/s
*/
enum state {ETAT1_ACC,ETAT2_CST,ETAT3_DEC, FIN};
enum state current_state;
void init_setpoint()
{
	current_state = ETAT1_ACC;
}

//bool
/*
uint32_t get_setpoint(float t, float * vitesse)
{
	uint32_t curr = 0; // si curr == 0, get_setpoint() non terminée, si curr==1 , get_setpoint terminée

	 switch(current_state)
	 {
	  case ETAT1_ACC :
	  {
		  //return vitesse = t*V1/t1 m/s
		  *vitesse = t*V1/t1;
		  if(t>=t1)
		  {
			  current_state = ETAT2_CST;
		  }
	  }
	  break;
	  case ETAT2_CST :
	  {
		  *vitesse = V1;
		  if(t>=t2)
		  {
			  current_state = ETAT3_DEC;
		  }
	  }
	  break;
	  case ETAT3_DEC :
	  {
		  //return vitesse = V1 - V1*(t-t2)/(t3-t2)
		  *vitesse = V1 - V1*(t-t2)/(t3-t2);
		  if(t>=t3)
		  {
			  current_state = FIN;
		  }
	  }
	  break;
	  case FIN :
	  {
		  curr = 1;
	  }
	  break;

	  case default :
	  {
		  curr = 1;
	  }
	  break;

	 }
	 return curr;
}

*/


/*
void marche_avant(uint32_t speed)
{
	   //moteur allume GAUCHE avant
	   HAL_GPIO_WritePin(RIGHT_DIR1_GPIO_Port,RIGHT_DIR1_Pin,GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(RIGHT_DIR2_GPIO_Port,RIGHT_DIR2_Pin,GPIO_PIN_SET);
	   //HAL_GPIO_WritePin(RIGHT_PWM_GPIO_Port,RIGHT_PWM_Pin,GPIO_PIN_SET); //GPIO
	   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,speed);

	   //moteur allume DROITE avant
	   HAL_GPIO_WritePin(LEFT_DIR1_GPIO_Port,LEFT_DIR1_Pin,GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(LEFT_DIR2_GPIO_Port,LEFT_DIR2_Pin,GPIO_PIN_SET);
	   //HAL_GPIO_WritePin(LEFT_PWM_GPIO_Port,LEFT_PWM_Pin,GPIO_PIN_SET);
	   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,speed);

}

void marche_arriere(uint32_t speed)
{
	   //moteur allume GAUCHE marche arriere
	   HAL_GPIO_WritePin(RIGHT_DIR1_GPIO_Port,RIGHT_DIR1_Pin,GPIO_PIN_SET);
	   HAL_GPIO_WritePin(RIGHT_DIR2_GPIO_Port,RIGHT_DIR2_Pin,GPIO_PIN_RESET);
	  // HAL_GPIO_WritePin(RIGHT_PWM_GPIO_Port,RIGHT_PWM_Pin,GPIO_PIN_SET);
	   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,speed);

	   //moteur allume DROITE marche arriere
	   HAL_GPIO_WritePin(LEFT_DIR1_GPIO_Port,LEFT_DIR1_Pin,GPIO_PIN_SET);
	   HAL_GPIO_WritePin(LEFT_DIR2_GPIO_Port,LEFT_DIR2_Pin,GPIO_PIN_RESET);
	  // HAL_GPIO_WritePin(LEFT_PWM_GPIO_Port,LEFT_PWM_Pin,GPIO_PIN_SET);
	   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,speed);
}

void arret()
{
	 //moteur eteint
	  HAL_GPIO_WritePin(RIGHT_DIR1_GPIO_Port,RIGHT_DIR1_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(RIGHT_DIR2_GPIO_Port,RIGHT_DIR2_Pin,GPIO_PIN_RESET);
	  //HAL_GPIO_WritePin(RIGHT_PWM_GPIO_Port,RIGHT_PWM_Pin,GPIO_PIN_SET);
	   __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);

	  HAL_GPIO_WritePin(LEFT_DIR1_GPIO_Port,LEFT_DIR1_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LEFT_DIR2_GPIO_Port,LEFT_DIR2_Pin,GPIO_PIN_RESET);
	  //HAL_GPIO_WritePin(LEFT_PWM_GPIO_Port,LEFT_PWM_Pin,GPIO_PIN_SET);
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
}
*/
