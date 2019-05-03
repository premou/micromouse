/*
 * math.h
 *
 *  Created on: 1 mars 2019
 *      Author: Invite
 */
#include <stdbool.h>

float constraint(float value, float min, float max);

float next_speed(float speed_target, float acceleration, float decceleration, float time, float speed_current);

/*
 * return 1 : have to stop
 * return 0 : keep going
 */
bool have_to_break(float speed_target, float speed_current, float distance_remaining, float decceleration);

typedef struct {
	float mean;
	float alpha;
} filter_ctx_t;

void filter_init(filter_ctx_t *ctx, float alpha);

void filter_reset(filter_ctx_t *ctx);

float filter_output(filter_ctx_t *ctx, float input);
