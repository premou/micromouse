/*
 * controller.h
 *
 *  Created on: 1 mars 2019
 *      Author: Invite
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "stm32f7xx_hal.h"
#include <stdbool.h>

uint32_t controller_init(); // return ZERO if gyro is OK, else return GYRO ERROR
void controller_start();
void controller_update();
bool controller_is_end();
void controller_stop();

#endif /* CONTROLLER_H_ */
