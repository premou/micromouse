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

#define 	X_SPEED 			0.34F 		// unit : m/s
#define 	X_MAX_ACCELERATION 	5.0F 		// unit : m/s^2
#define 	X_MAX_DECELERATION 	3.0F		// unit : m/s^2
#define 	X_SPEED_KP 			600.0F		// this is a speed parameter
#define 	X_SPEED_KI 			10.0F		// this is a position parameter
#define 	X_SPEED_KD 			0.0F 		// this is an acceleration, so we do not use this input
#define 	W_SPEED_KP 			0.1F 		// this is a speed parameter
#define 	W_SPEED_KI 			0.004F 		// this is a position parameter
#define 	W_SPEED_KD 			0.0F 		// this is an acceleration, so we do not use this input
#define 	GEAR_BOX_RATIO		1.0F/50.0F	// 1:50
#define 	WHEEL_DIAMETER      0.026000F	// unit : mm
#define 	INIT_GYRO_BIAS 						-1.530F	// unit : dps
#define 	GYRO_AUTOCAL_VARIANCE_THRESHOLD 	0.040F	// unit : dps^2 (don't change this)
#define 	GYRO_SENSITIVITY_CORRECTION 		1.000F	// unit : %
#define 	W_SPEED 			330.0		// unit : dps
#define 	W_T1 				280 		// unit : ms
#define 	W_T2 				345			// unit : ms
#define 	W_MAX_ACCELERATION 	5000		// unit : dps^2
#define		W_MAX_DECELERATION 	5000		// unit : dpz^2

#endif

#ifdef __PATRICK__

// Recommendation : Always charge up battery to maximum capacity/voltage (8.4V)

// First, we set the parameters for running forward with controlled acceleration, speed and position.

// 1.00. Define FIXED_MOVES for the following tests and configuration
#define FIXED_MOVES // disable AI

// 1.01. Set forward speed of learning run and turns
#define 	X_SPEED 			0.450F 		// unit : m/s

// 1.02. Set forward acceleration
#define 	X_MAX_ACCELERATION 	5.0F 		// unit : m/s^2
#define 	X_MAX_DECELERATION 	3.0F		// unit : m/s^2

// 1.03. Set PID parameters (Kp,Ki) for X SPEED PID and W SPEED PID
#define 	X_SPEED_KP 			600.0F		// this is a speed parameter
#define 	X_SPEED_KI 			10.0F		// this is a position parameter
#define 	X_SPEED_KD 			0.0F 		// this is an acceleration, so we do not use this input
#define 	W_SPEED_KP 			0.1F 		// this is a speed parameter
#define 	W_SPEED_KI 			0.004F 		// this is a position parameter
#define 	W_SPEED_KD 			0.0F 		// this is an acceleration, so we do not use this input

// 1.04. Set wheel diameter and micro gear ratio
#define 	GEAR_BOX_RATIO		1.0F/50.0F	// 1:50
#define 	WHEEL_DIAMETER      0.026000F	// unit : mm

// 1.05, use visualization and physics to adjust all these parameters in order to :
// >>> physics : distance traveled must be equal to 180 mm
// >>> visualization : actual forward speed must follow current forward speed
//#define SC1_START_STOP // 180mm
//#define SC1_START_RUN3_STOP // 720mm

// 1.06. Undefine SC1_xxx (comment) and continue tests and configuration

// Next, we set the parameters for turning with controlled acceleration, speed and position.

// 2.00 Set the default gyro bias
#define 	INIT_GYRO_BIAS 						-1.530F	// unit : dps
#define 	GYRO_AUTOCAL_VARIANCE_THRESHOLD 	0.040F	// unit : dps^2 (don't change this)
// define IMU_TRACE to see variance and bias when IDLE

// 2.01 Set the sensitivity correction
// >>> physics using turn table : one turn gives 360° heading exactly (display heading)
#define 	GYRO_SENSITIVITY_CORRECTION 		1.025F	// unit : %
// define IMU_TRACE to see heading when IDLE, or upload datalogger and watch telemetry

// 2.02 Set rotation speed of curve turn
// >>> use Trapezoidal-Curve-Turn-Profile-Generator
//     forward speed = X_SPEED
//     angle = 90
//     radius = 70
//     acceleration = W_MAX_ACCELERATION
//     deceleration = W_MAX_DECELERATION
//     max angular vel = W_SPEED
//     mouse width = 70
#define 	W_SPEED 			460.0F		// unit : dps
#define 	W_T1 				200 		// unit : ms
#define 	W_T2 				256			// unit : ms

// 2.03. Set rotation acceleration
#define 	W_MAX_ACCELERATION 	8000		// unit : dps^2
#define		W_MAX_DECELERATION 	8000		// unit : dps^2

// 2.04, use visualization and physics to adjust all these parameters in order to :
// >>> physics : return to home exactly after each turn
// >>> visualization : actual forward and rotation speeds must follow current speeds
#define SC2_SQUARE_TEST_1_TURN






#endif

#ifdef __PREM__



#endif

#ifdef __REMI__



#endif

/*****************************************************************************/


#endif /* PERSONNALISATION_H_ */
