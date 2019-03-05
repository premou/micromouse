/*
 * fonc.h
 *
 *  Created on: 8 nov. 2018
 *      Author: Invite
 */

#ifndef FONC_H_
#define FONC_H_

#include "main.h"
#include "stm32f7xx_hal.h"

/*
void marche_avant(uint32_t speed);
void marche_arriere(uint32_t speed);
void arret();
*/




void run(int32_t speed_right, int32_t speed_left);


void init_setpoint();

/**


/**
 * Return la valeur de la vitesse en m/s
 * dist : la distance parcourue en m
 * time : le temps de parcours en ms
 */
float get_speed(float dist, uint32_t time);

uint32_t get_setpoint(float t, float * vitesse);
#endif /* FONC_H_ */
