/*
 * motor.h
 *
 *  Created on: 29 janv. 2019
 *      Author: Invite
 */

#ifndef MOTOR_H_
#define MOTOR_H_

// GLOBAL VARIABLES
/* slopes for speed (in m/s-2) */
#define SLOPE_ACC 2
#define SLOPE_DEC 2
#define SPEED_TARGET 0.5 //in m/s
#define DIST_START 0.09 //in m
#define DIST_RUN_1 0.18 //in m
#define DIST_STOP 0.09 //in m


// ENUM

typedef  enum {
	LEFT,
	RIGHT
} side_t;

typedef enum {
	ACTION_IDLE,
	ACTION_START, //avance de 8 cm puis RUN_1
	ACTION_RUN_1,
	ACTION_STOP,
	ACTION_CTR
} action_t;


// STRUCTURES DEFINITIONS

typedef struct {
	float speed;
	float distance;

} action_target_t;

typedef struct {
	float dist;
	float speed;

	uint32 dist_ref_front;
	uint32 dist_ref_back;

} side_t;

typedef struct  {

	float dist;
	float speed;

	side_t left;
	side_t right;

	uint32_t time;

	action_target_t **p_actions;
	uint32            actions_nb;
	action_target_t  *p_action_curr;

} motors_t;

#endif /* MOTOR_H_ */
