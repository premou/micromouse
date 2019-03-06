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

void controller_init();
void controller_start();
void controller_update();
bool controller_is_end();

#endif /* CONTROLLER_H_ */
