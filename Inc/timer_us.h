/*
 * controller.h
 *
 *  Created on: 21 mars 2019
 *      Author: Invite
 */

#ifndef TIMER_US_H_
#define TIMER_US_H_

#include "stm32f7xx_hal.h"

// every time we need a more accurate timer than HAL GetTick (millisecond), we use a a TIMER configured in timebase mode, providing microsecond precision

// initialise microsecond timer
void timer_us_init();

// get time in microsecond from boot
uint16_t timer_us_get();

// wait for a few microseconds
void timer_us_delay(uint16_t delay_us);

#endif /* TIMER_US_H_ */
