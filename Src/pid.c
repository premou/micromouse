/*
 * pid.c
 *
 *  Created on: 27 févr. 2019
 *      Author: robot °_°
 */

#include "pid.h"

void pid_init(pid_context_t *ctx, float Kp, float Ki, float Kd)
{
	ctx->Kp = Kp;
	ctx->Ki = Ki;
	ctx->Kd = Kd;

	ctx->err_previous = 0;
	ctx->err_sum = 0;
}

void pid_reset(pid_context_t *ctx)
{
	ctx->err_previous = 0;
	ctx->err_sum = 0;
}

float pid_output(pid_context_t *ctx, float error)
{
	float p_term = error*ctx->Kp;
	ctx->err_sum += error;
	float i_term = ctx->err_sum*ctx->Ki;
	float d_term = (error - ctx->err_previous)*ctx->Kd;
	ctx->err_previous = error;

	return p_term + i_term + d_term;
}

