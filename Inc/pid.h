/*
 * pid.h
 *
 *  Created on: 28 févr. 2019
 *      Author: Invite
 */


#include "stm32f7xx_hal.h"

#ifndef PID_H_
#define PID_H_

/*
 *
 */
typedef struct{
	float Kp;
	float Ki;
	float Kd;
	float alpha;

	float err_filtered;
	float err_previous;
	float err_sum;
}pid_context_t;

void pid_init(pid_context_t *ctx, float Kp, float Ki, float Kd, float alpha);

void pid_reset(pid_context_t *ctx);

float pid_output(pid_context_t *ctx, float error);

#endif /* PID_H_ */
