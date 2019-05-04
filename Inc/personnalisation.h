/*
 * personnalisation.h
 *
 *  Created on: 4 mai 2019
 *      Author: Patrick
 */

#ifndef PERSONNALISATION_H_
#define PERSONNALISATION_H_

/*****************************************************************************/
/* 									OWNER 									 */
/*****************************************************************************/

// DECLARE OWNER (only one at once)
//#define __ALICE__		// All has the a HW revision 0.00
//#define __FRANCOIS__
#define __PATRICK__ 	// Patrick has the prototype HW
//#define __PREM__
//#define __REMI__

/*****************************************************************************/
/* 									SETTINGS								 */
/*****************************************************************************/

// THIS FILE CONFIGURE EVERY THING (except wall sensor, see WallSensor.h)

#ifdef __ALICE__



#endif

#ifdef __FRANCOIS__



#endif

#ifdef __PATRICK__

// First we set the parameter for running forward with controlled acceleration, speed and position.

// 000. Define FIXED_MOVES for the following tests and configuration
#define FIXED_MOVES // disable AI

// 001. Define SC1_START_STOP
#define SC1_START_STOP

// 002. Set forward speed of learning run and turns
#define 	X_SPEED 			0.5F 		// unit : m/s

// 003. Set forward acceleration
#define 	X_MAX_ACCELERATION 	5.0F 		// unit : m/s^2
#define 	X_MAX_DECELERATION 	3.0F		// unit : m/s^2

// 004. Set PID parameters (Kp,Ki) for X SPEED PID and W SPEED PID
#define 	X_SPEED_KP 			600.0F		// this is a speed parameter
#define 	X_SPEED_KI 			10.0F		// this is a position parameter
#define 	X_SPEED_KD 			0.0F 		// this is an acceleration, so we do not use this input
#define 	W_SPEED_KP 			0.1F 		// this is a speed parameter
#define 	W_SPEED_KI 			0.004F 		// this is a position parameter
#define 	W_SPEED_KD 			0.0F 		// this is an acceleration, so we do not use this input

// 005. Set wheel diameter and micro gear ratio
#define 	GEAR_BOX_RATIO		1.0F/50.0F	// 1:50
#define 	WHEEL_DIAMETER      0.026000F	// 26mm

// NOW, use visualization to adjust all these parameters in order to :
// >>> distance traveled must be equal to 180 cm

// 006. Undefine SC1_START_STOP (comment) and continue tests and configutation






#endif

#ifdef __PREM__



#endif

#ifdef __REMI__



#endif

/*****************************************************************************/


#endif /* PERSONNALISATION_H_ */
