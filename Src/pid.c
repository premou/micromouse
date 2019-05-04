/*
 * pid.c
 *
 *  Created on: 27 févr. 2019
 *      Author: robot °_°
 */

#include "pid.h"

void pid_init(pid_context_t *ctx, float Kp, float Ki, float Kd, float alpha)
{
	ctx->Kp = Kp;
	ctx->Ki = Ki;
	ctx->Kd = Kd;
	ctx->alpha = alpha;

	ctx->err_filtered = 0;
	ctx->err_previous = 0;
	ctx->err_sum = 0;
}

void pid_reset(pid_context_t *ctx)
{
	ctx->err_filtered = 0;
	ctx->err_previous = 0;
	ctx->err_sum = 0;
}

float pid_output(pid_context_t *ctx, float error)
{
	ctx->err_filtered = (1.0-ctx->alpha)*ctx->err_filtered + ctx->alpha*error;

	float p_term = ctx->err_filtered*ctx->Kp;
	ctx->err_sum += error;
	float i_term = ctx->err_sum*ctx->Ki;
	float d_term = (ctx->err_filtered - ctx->err_previous)*ctx->Kd;
	ctx->err_previous = ctx->err_filtered;

	return p_term + i_term + d_term;
}

