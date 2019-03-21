/*
 * math.c
 *
 *  Created on: 1 mars 2019
 *      Author: Invite
 */
#include "math.h"

float constraint(float value, float min, float max)
{
	if(value>max)
	{
		return max;
	}
	else if(value<min)
	{
		return min;
	}
	else
	{
		return value;
	}
}

float next_speed(float speed_target, float acceleration, float decceleration, float time, float speed_current)
{
	if(speed_current==speed_target)
	{
		return speed_target;
	}
	else if(speed_current<speed_target)
	{
		float speed_increment = time*acceleration;
		float speed_next = speed_current + speed_increment;
		if(speed_next>speed_target)
		{
			return speed_target;
		}
		else
		{
			return speed_next;
		}
	}
	else if(speed_current>speed_target)
	{
		float speed_decrement = time*decceleration;
		float speed_next = speed_current - speed_decrement;
		if(speed_next<speed_target)
		{
			return speed_target;
		}
		else
		{
			return speed_next;
		}
	}
	else
	{
		return speed_target;
	}
}

/*
 * return 1 : have to stop
 * return 0 : keep going
 */
bool have_to_break(float speed_target, float speed_current, float distance_remaining, float decceleration)
{
	/* decceleration with speed_current to reach speed_target in distance_remaining */
	//float decc = (speed_current-speed_target)*(speed_current-speed_target) / (2.0*distance_remaining);
	float decc = (speed_current*speed_current-speed_target*speed_target) / (2.0*distance_remaining);
	return decc>=decceleration;
	// FIXME : acceleration = (V2^2 - V1^2)/2D, then check calculated acceleration is less than -decceleration paramter
}

void filter_init(filter_ctx_t *ctx, float alpha)
{
	ctx->alpha = alpha;
	ctx->mean = 0;
}

void filter_reset(filter_ctx_t *ctx)
{
	ctx->mean = 0;
}

float filter_output(filter_ctx_t *ctx, float input)
{
	/* alpha is the weight of the new input value */
	ctx->mean = input*ctx->alpha + (1-ctx->alpha)*ctx->mean;
	return ctx->mean;
}
