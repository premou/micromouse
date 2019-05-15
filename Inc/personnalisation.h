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
#define __FRANCOIS__
//#define __PATRICK__ 	// Patrick has the prototype HW
//#define __PREM__
//#define __REMI__

/*****************************************************************************/
/* 									SETTINGS								 */
/*****************************************************************************/

//#define FIXED_MOVES // disable AI

// THIS FILE CONFIGURE EVERY THING (except wall sensor, see WallSensor.h)

#ifdef __ALICE__

// Recommendation : Always charge up battery to maximum capacity/voltage (8.4V)

// 0.01. Adjust voltage divider ration
// >>> use a power supply with voltage output settings to 8.00V
#define 	VOLTAGE_RATIO 			0.177F 		// unit : ratio
//#define VOLTAGE_TRACE //to see voltage when IDLE

// 0.02. Use "banc micromouse" to calibrate wall IR sensors (see. WallSensorCallibration.h)
// >> first check IR sensor orientation
// 		>> front wall sensor open 5°
// 		>> diagonal sensor point to middle of side walls when mouse placed before cell
// >> then use "banc micrmouse" to compute linear regression (theta)
//#define RAW_IR_TRACE //to calibrate wall IR sensors when IDLE
//#define CALIBRATED_IR_TRACE //to check calibrate wall IR sensors when IDLE

// 0.03. Use "banc micromouse " to set the distance to walls
// maximal distance to front wall (sum of both sensors)
#define 	FRONT_WALL_DISTANCE 				350		// unit : mm
// maximal distance to left or right wall
#define 	SIDE_WALL_DISTANCE 					120		// unit : mm
// distance to left wall when mouse in middle
#define 	LEFT_WALL_DISTANCE_NO_SIDE_ERROR 	70.0	// unit : mm
// distance to right wall when mouse in middle
#define 	RIGHT_WALL_DISTANCE_NO_SIDE_ERROR 	60.0	// unit : mm
// position of micromouse when wall fades (140mm)
#define REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL 0.120 	// unit : mm
// position of micromouse when post fades (140mm)
#define REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST 0.115 	// unit : mm
// distance to front wall when micromouse doint dead end turn back
#define WALL_FRONT_DISTANCE_mm 					26.0 	// unit : mm
// distance offset to front wall when micromouse doint dead end turn back
#define WALL_FRONT_ANGLE_mm 					0.0 	// unit : mm
// distance to front wall when micromouse doing curve turn
#define WALL_FRONT_ANGLE_TURNING_SUM_mm 			240.0 	// unit : mm
// adjust distances LEFT_WALL_DISTANCE_NO_SIDE_ERROR
// adjust distances RIGHT_WALL_DISTANCE_NO_SIDE_ERROR
// to get smooth transition between one or two wall following.
//#define WALL_FOLLOWING_TRACE  //to see wall position when IDLE, or upload datalogger and watch telemetry

// First, we set the parameters for running forward with controlled acceleration, speed and position.

// 1.00. Define FIXED_MOVES for the following tests and configuration
//#define FIXED_MOVES // disable AI

// 1.01. Set forward speed of learning run and turns
#define 	X_SPEED 			0.35 		// unit : m/s

// 1.02. Set forward acceleration
#define 	X_MAX_ACCELERATION 	5.0F 		// unit : m/s^2
#define 	X_MAX_DECELERATION 	3.0F		// unit : m/s^2

// 1.03. Set PID parameters (Kp,Ki) for X SPEED PID and W SPEED PID
#define 	X_SPEED_KP 			600.0F		// this is a speed parameter
#define 	X_SPEED_KI 			10.0F		// this is a position parameter
#define 	X_SPEED_KD 			0.0F 		// this is an acceleration, so we do not use this input
#define 	W_SPEED_KP 			0.04 //0.08F 		// this is a speed parameter
#define 	W_SPEED_KI 			0.002 //0.002F 		// this is a position parameter
#define 	W_SPEED_KD 			0.0F 		// this is an acceleration, so we do not use this input
#define 	W_SPEED_KF 			0.05F 		// this is a feed forward speed input

// 1.04. Set wheel diameter and micro gear ratio
#define 	GEAR_BOX_RATIO		1.0F/30.0F	// 1:30
#define 	WHEEL_DIAMETER      0.0265F	// unit : mm
// the greater is the wheel diameter, lesser is the distance traveled

// 1.05, use visualization and physics to adjust all these parameters in order to :
// >>> physics : distance traveled must be equal to 180 mm
// >>> visualization : actual forward speed must follow current forward speed
//#define SC_START_STOP // 180mm
//#define SC_START_RUN_STOP // 360mm
//#define SC_START_RUN2_STOP // 540mm
//#define SC_START_RUN3_STOP // 720mm

// Next, we set the parameters for turning with controlled acceleration, speed and position.

// 2.00 Set the default gyro bias
#define 	INIT_GYRO_BIAS 						-5.860F	// unit : dps
#define 	GYRO_AUTOCAL_VARIANCE_THRESHOLD 	0.040F	// unit : dps^2 (don't change this)
//#define IMU_TRACE //to see variance and bias when IDLE

// 2.01 Set the sensitivity correction
// >>> physics using turn table : one turn gives 360° heading exactly (display heading)
#define 	GYRO_SENSITIVITY_CORRECTION 		0.985F //1.025F	// unit : %
// the greater is the correction, lesser the robot turn
// #define IMU_TRACE //to see heading when IDLE, or upload datalogger and watch telemetry
// use carré test to check real heading with estimated heading with telemetry

// 2.02 Set rotation speed of curve turn
// >>> use Trapezoidal-Curve-Turn-Profile-Generator
//     forward speed = X_SPEED
//     angle = 90
//     radius = 70
//     acceleration = W_MAX_ACCELERATION
//     deceleration = W_MAX_DECELERATION
//     max angular vel = W_SPEED
//     mouse width = 70
#define 	W_SPEED 			330.0F		// unit : dps
#define 	W_T1 				273 		// unit : ms
#define 	W_T2 				328			// unit : ms

// 2.03. Set rotation acceleration
#define 	W_MAX_ACCELERATION 	6000		// unit : dps^2
#define		W_MAX_DECELERATION 	6000		// unit : dps^2

// 2.04, use visualization and physics to adjust all these parameters in order to :
// >>> physics : return to home exactly after each turn
// >>> visualization : actual forward and rotation speeds must follow current speeds
//#define SC_F_L90_F
//#define SC_F_R90_F
//#define SC_SQUARE_TEST_1_TURN
//#define SC_SQUARE_TEST_2_TURN
//#define SC_F_R90_R90_F
#define SC_ZIGZAG

// Next, we set the parameters for wall following

// 3.01. Set PID parameters (Kp,Kd) for WALL POSITION PID
#define 	WALL_POSITION_KP 	0.5		// this is a position parameter
#define 	WALL_POSITION_KI 	0.0
#define 	WALL_POSITION_KD 	10.0	// this is a speed parameter

// 3.02. use visualization and physics to adjust all these parameters in order to :
// >>> physics : mouse moves forward in the middle of cell and avoids walls
//#define SC_START_RUN_STOP
//#define SC_START_RUN3_STOP

// Next, we set the parameters for wall-to-wall calibration

// 4.01. use visualization and physics to adjust all the following parameters
// REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL 0.120 	// unit : mm
// REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST 0.115 	// unit : mm
// in order to stop the mouse in the middle of the last cell
// > try first without wall
// > try with one side wall, two side walls, one post, two post, one post and a wall, ...
//#define SC_START_RUN1_STOP
//#define SC_START_RUN3_STOP

// Next, we set the parameters for front-wall cornering calibration

// 5.01. use visualization and physics to adjust all the following parameters
// WALL_FRONT_ANGLE_TURNING_mm
// in order to stop the mouse in the middle of the last cell, and never crash front wall corners; or side wall at the end of corners
// > try first without wall > carre must be OK
// > try with one front wall in front, two, three, .. at each corner
//#define SC_SQUARE_TEST_1_TURN
//#define SC_SQUARE_TEST_2_TURN

// Next, we set the parameters for uturn

// 6.01 Set timing for dead end rotation
#define 	W_U_SPEED 			360.0F		// unit : dps
#define 	W_U_T1_90			260 		// unit : ms
#define 	W_U_T2_90			305			// unit : ms
#define 	W_U_T1_180			502 		// unit : ms
#define 	W_U_T2_180			562			// unit : ms

// 6.02. Set front wall distance position  PID
#define X_WALL_FRONT_KP 1.0
#define X_WALL_FRONT_KI 0.0002
#define X_WALL_FRONT_KD 0.0

// 6.03. Set front wall angle position  PID
#define W_WALL_FRONT_KP 1.0
#define W_WALL_FRONT_KI 0.0001
#define W_WALL_FRONT_KD 0.0

// 6.04. use visualization and physics to adjust all the those parameters
// in order to stop the mouse in the middle of the last cell, and never crash front or side walls
// > try first without wall
// > try with one front wall, two or three walls
//#define SC_U_TURN
//#define SC_RUN1_UTURN_RUN1
//#define SC_TURN_RIGHT_TEST


// max speed
#define X_SPEED_FAST_RUN 0.7 // m/s
#define X_SPEED_FAST_RUN_IMPROVED 1.0 // m/s

// RAND moves test stress

// AI



#endif

#ifdef __FRANCOIS__

// Recommendation : Always charge up battery to maximum capacity/voltage (8.4V)

// 0.01. Adjust voltage divider ration
// >>> use a power supply with voltage output settings to 8.00V
#define 	VOLTAGE_RATIO 			0.09F 		// unit : ratio
//#define VOLTAGE_TRACE //to see voltage when IDLE

// 0.02. Use "banc micromouse" to calibrate wall IR sensors (see. WallSensorCallibration.h)
// >> first check IR sensor orientation
// 		>> front wall sensor open 5°
// 		>> diagonal sensor point to middle of side walls when mouse placed before cell
// >> then use "banc micrmouse" to compute linear regression (theta)
//#define RAW_IR_TRACE //to calibrate wall IR sensors when IDLE
//#define CALIBRATED_IR_TRACE //to check calibrate wall IR sensors when IDLE

// 0.03. Use "banc micromouse " to set the distance to walls
// maximal distance to front wall (sum of both sensors)
#define 	FRONT_WALL_DISTANCE 				320		// unit : mm
// maximal distance to left or right wall
#define 	SIDE_WALL_DISTANCE 					65		// unit : mm
// distance to left wall when mouse in middle
#define 	LEFT_WALL_DISTANCE_NO_SIDE_ERROR 	30.0	// unit : mm
// distance to right wall when mouse in middle
#define 	RIGHT_WALL_DISTANCE_NO_SIDE_ERROR 	30.0	// unit : mm
// position of micromouse when wall fades (140mm)
#define REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL 0.130 	// unit : mm
// position of micromouse when post fades (140mm)
#define REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST 0.115 	// unit : mm
// distance to front wall when micromouse doint dead end turn back
#define WALL_FRONT_DISTANCE_mm 					35.0 	// unit : mm
// distance offset to front wall when micromouse doint dead end turn back
#define WALL_FRONT_ANGLE_mm 					0.0 	// unit : mm
// sum distance to front wall when micromouse doing curve turn
#define WALL_FRONT_ANGLE_TURNING_SUM_mm 			160.0 	// unit : mm
// delta distance to front wall when micromouse doing curve turn
#define WALL_FRONT_ANGLE_TURNING_DELTA_mm 			0.0 	// unit : mm
#define WALL_FRONT_ANGLE_TURNING_DELTA_coef			0.0 	// unit : mm to ms
// adjust distances LEFT_WALL_DISTANCE_NO_SIDE_ERROR
// adjust distances RIGHT_WALL_DISTANCE_NO_SIDE_ERROR
// to get smooth transition between one or two wall following.
//#define WALL_FOLLOWING_TRACE  //to see wall position when IDLE, or upload datalogger and watch telemetry

// First, we set the parameters for running forward with controlled acceleration, speed and position.

// 1.00. Define FIXED_MOVES for the following tests and configuration
//#define FIXED_MOVES // disable AI

// 1.01. Set forward speed of learning run and turns
//#define 	X_SPEED 			0.35 		// unit : m/s
#define 	X_SPEED 			0.5 		// unit : m/s

// 1.02. Set forward acceleration
#define 	X_MAX_ACCELERATION 	5.0F 		// unit : m/s^2
#define 	X_MAX_DECELERATION 	3.0F		// unit : m/s^2

// 1.03. Set PID parameters (Kp,Ki) for X SPEED PID and W SPEED PID
#define 	X_SPEED_KP 			600.0F		// this is a speed parameter
#define 	X_SPEED_KI 			10.0F		// this is a position parameter
#define 	X_SPEED_KD 			0.0F 		// this is an acceleration, so we do not use this input
#define 	W_SPEED_KP 			0.04 //0.08F 		// this is a speed parameter
#define 	W_SPEED_KI 			0.002 //0.002F 		// this is a position parameter
#define 	W_SPEED_KD 			0.0F 		// this is an acceleration, so we do not use this input
#define 	W_SPEED_KF 			0.08F 		// this is a feed forward speed input

// 1.04. Set wheel diameter and micro gear ratio
#define 	GEAR_BOX_RATIO		1.0F/50.0F	// 1:50
#define 	WHEEL_DIAMETER      0.0259F	// unit : mm
// the greater is the wheel diameter, lesser is the distance traveled

// 1.05, use visualization and physics to adjust all these parameters in order to :
// >>> physics : distance traveled must be equal to 180 mm
// >>> visualization : actual forward speed must follow current forward speed
//#define SC_START_STOP // 180mm
//#define SC_START_RUN_STOP // 360mm
//#define SC_START_RUN2_STOP // 540mm
//#define SC_START_RUN3_STOP // 720mm

// Next, we set the parameters for turning with controlled acceleration, speed and position.

// 2.00 Set the default gyro bias
#define 	INIT_GYRO_BIAS 						-3.1F	// unit : dps
#define 	GYRO_AUTOCAL_VARIANCE_THRESHOLD 	0.040F	// unit : dps^2 (don't change this)
//#define     IMU_TRACE // to see variance and bias when IDLE

// 2.01 Set the sensitivity correction
// >>> physics using turn table : one turn gives 360° heading exactly (display heading)
#define 	GYRO_SENSITIVITY_CORRECTION 		1.01F //0.990F	// unit : %
// #define 	SCALE_500_DPS // default
#define 	SCALE_1000_DPS
// the greater is the correction, lesser the robot turn
// #define IMU_TRACE //to see heading when IDLE, or upload datalogger and watch telemetry
// use carré test to check real heading with estimated heading with telemetry

// 2.02 Set rotation speed of curve turn
// >>> use Trapezoidal-Curve-Turn-Profile-Generator
//     forward speed = X_SPEED
//     angle = 90
//     radius = 70
//     acceleration = W_MAX_ACCELERATION
//     deceleration = W_MAX_DECELERATION
//     max angular vel = W_SPEED
//     mouse width = 70
//#define 	W_SPEED 			330.0F		// unit : dps
//#define 	W_T1 				273 		// unit : ms
//#define 	W_T2 				328			// unit : ms
#define 	W_SPEED 			540.0F		// unit : dps
#define 	W_T1 				167 		// unit : ms
#define 	W_T2 				235		    // unit : ms

// 2.03. Set rotation acceleration
//#define 	W_MAX_ACCELERATION 	6000		// unit : dps^2
//#define		W_MAX_DECELERATION 	6000		// unit : dps^2
#define 	W_MAX_ACCELERATION 	8000		// unit : dps^2
#define		W_MAX_DECELERATION 	8000		// unit : dps^2

// 2.04, use visualization and physics to adjust all these parameters in order to :
// >>> physics : return to home exactly after each turn
// >>> visualization : actual forward and rotation speeds must follow current speeds
//#define SC_F_L90_F
//#define SC_F_R90_F
//#define SC_F_R90_F_R90_F
//#define SC_SQUARE_TEST_1_TURN
//#define SC_SQUARE_TEST_2_TURN
//#define SC_F_R90_R90_F
//#define SC_ZIGZAG

// Next, we set the parameters for wall following

// 3.01. Set PID parameters (Kp,Kd) for WALL POSITION PID
#define 	WALL_POSITION_KP 	0.5		// this is a position parameter
#define 	WALL_POSITION_KI 	0.0
#define 	WALL_POSITION_KD 	10.0	// this is a speed parameter

// 3.02. use visualization and physics to adjust all these parameters in order to :
// >>> physics : mouse moves forward in the middle of cell and avoids walls
//#define SC_START_RUN_STOP
//#define SC_START_RUN3_STOP

// Next, we set the parameters for wall-to-wall calibration

// 4.01. use visualization and physics to adjust all the following parameters
// REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL 0.120 	// unit : mm
// REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST 0.115 	// unit : mm
// in order to stop the mouse in the middle of the last cell
// > try first without wall
// > try with one side wall, two side walls, one post, two post, one post and a wall, ...
//#define SC_START_RUN1_STOP
//#define SC_START_RUN3_STOP

// Next, we set the parameters for front-wall cornering calibration

// 5.01. use visualization and physics to adjust all the following parameters
// WALL_FRONT_ANGLE_TURNING_mm
// in order to stop the mouse in the middle of the last cell, and never crash front wall corners; or side wall at the end of corners
// > try first without wall > carre must be OK
// > try with one front wall in front, two, three, .. at each corner
//#define SC_SQUARE_TEST_1_TURN
//#define SC_SQUARE_TEST_2_TURN

// Next, we set the parameters for uturn

// 6.01 Set timing for dead end rotation
#define 	W_U_SPEED 			360.0F		// unit : dps
#define 	W_U_T1_90			250 		// unit : ms
#define 	W_U_T2_90			305			// unit : ms
#define 	W_U_T1_180			500 		// unit : ms
#define 	W_U_T2_180			555			// unit : ms

// 6.02. Set front wall distance position  PID
#define X_WALL_FRONT_KP 1.0
#define X_WALL_FRONT_KI 0.0002
#define X_WALL_FRONT_KD 0.0

// 6.03. Set front wall angle position  PID
#define W_WALL_FRONT_KP 1.0
#define W_WALL_FRONT_KI 0.0001
#define W_WALL_FRONT_KD 0.0

// 6.04. use visualization and physics to adjust all the those parameters
// in order to stop the mouse in the middle of the last cell, and never crash front or side walls
// > try first without wall
// > try with one front wall, two or three walls
//#define SC_U_TURN
//#define SC_RUN1_UTURN_RUN1
//#define SC_TURN_RIGHT_TEST
//#define UTURN_TRACE

// max speed
#define X_SPEED_FAST_RUN 0.7 // m/s
#define X_SPEED_FAST_RUN_IMPROVED 1.0 // m/s

// RAND moves test stress
//#define RAND_MOVES_3
//#define RAND_MOVES_6
//#define RAND_MOVES_12
//#define RAND_MOVES_16

#endif // endif __FRANCOIS__

#ifdef __FRANCOIS___OLD


#define 	VOLTAGE_RATIO 		0.092F 		// unit : ratio
#define 	FRONT_WALL_DISTANCE 				320		// unit : mm
#define 	SIDE_WALL_DISTANCE 					110		// unit : mm
#define 	LEFT_WALL_DISTANCE_NO_SIDE_ERROR 	87.0	// unit : mm
#define 	RIGHT_WALL_DISTANCE_NO_SIDE_ERROR 	63.0	// unit : mm
#define 	WALL_POSITION_OFFSET				0.0		// unit : mm
#define 	REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL 0.110 	// unit : mm
#define 	REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST 0.100 	// unit : mm
#define 	WALL_FRONT_DISTANCE_mm 					32.0 	// unit : mm
#define 	WALL_FRONT_ANGLE_mm 					0.0 	// unit : mm
#define		 WALL_FRONT_ANGLE_TURNING_SUM_mm 			170.0 	// unit : mm
#define 	X_SPEED 			0.34F 		// unit : m/s
#define 	X_MAX_ACCELERATION 	5.0F 		// unit : m/s^2
#define 	X_MAX_DECELERATION 	3.0F		// unit : m/s^2
#define 	X_SPEED_KP 			600.0F		// this is a speed parameter
#define 	X_SPEED_KI 			10.0F		// this is a position parameter
#define 	X_SPEED_KD 			0.0F 		// this is an acceleration, so we do not use this input
#define 	W_SPEED_KP 			0.1F 		// this is a speed parameter
#define 	W_SPEED_KI 			0.004F 		// this is a position parameter
#define 	W_SPEED_KD 			0.0F 		// this is an acceleration, so we do not use this input
#define 	W_SPEED_KF 			0.02F 		// this is a feed forward speed input
#define 	GEAR_BOX_RATIO		1.0F/50.0F	// 1:50
#define 	WHEEL_DIAMETER      0.026000F	// unit : mm
#define 	INIT_GYRO_BIAS 						-1.530F	// unit : dps
#define 	GYRO_AUTOCAL_VARIANCE_THRESHOLD 	0.040F	// unit : dps^2 (don't change this)
#define 	GYRO_SENSITIVITY_CORRECTION 		1.000F	// unit : %
#define 	W_SPEED 			320.0		// unit : dps
#define 	W_T1 				280 		// unit : ms
#define 	W_T2 				345			// unit : ms
#define 	W_MAX_ACCELERATION 	5000		// unit : dps^2
#define		W_MAX_DECELERATION 	5000		// unit : dpz^2
#define 	WALL_POSITION_KP 	0.3		// this is a position parameter
#define 	WALL_POSITION_KI 	0.0
#define 	WALL_POSITION_KD 	1.0		// this is a speed parameter
#define 	W_U_SPEED 			360.0F		// unit : dps
#define 	W_U_T1_90			260 		// unit : ms
#define 	W_U_T2_90			310			// unit : ms
#define 	W_U_T1_180			510 		// unit : ms
#define 	W_U_T2_180			560			// unit : ms
#define 	X_WALL_FRONT_KP 	1.0
#define 	X_WALL_FRONT_KI 	0.0001
#define 	X_WALL_FRONT_KD 	0.0
#define 	W_WALL_FRONT_KP 	1.0
#define 	W_WALL_FRONT_KI 	0.0001
#define 	W_WALL_FRONT_KD 	0.0
#define 	X_SPEED_FAST_RUN 	0.5 		// m/s
#define 	X_SPEED_FAST_RUN_IMPROVED 0.7 	// m/s

//#define FIXED_MOVES // disable AI
//#define SC_START_STOP
//#define SC_START_RUN3_STOP
//#define SC_SQUARE_TEST_1_TURN
//#define SC_SQUARE_TEST_2_TURN
//#define SC_U_TURN
//#define SC_RUN1_UTURN_RUN1
#endif

#ifdef __PATRICK__

// Recommendation : Always charge up battery to maximum capacity/voltage (8.4V)

// 0.01. Adjust voltage divider ration
// >>> use a power supply with voltage output settings to 8.00V
#define 	VOLTAGE_RATIO 			0.178F 		// unit : ratio
//#define VOLTAGE_TRACE //to see voltage when IDLE

// 0.02. Use "banc micromouse" to calibrate wall IR sensors (see. WallSensorCallibration.h)
// >> first check IR sensor orientation
// 		>> front wall sensor open 5°
// 		>> diagonal sensor point to middle of side walls when mouse placed before cell
// >> then use "banc micrmouse" to compute linear regression (theta)
//#define RAW_IR_TRACE //to calibrate wall IR sensors when IDLE
#define CALIBRATED_IR_TRACE //to check calibrate wall IR sensors when IDLE

// 0.03. Use "banc micromouse " to set the distance to walls
// maximal distance to front wall (sum of both sensors)
#define 	FRONT_WALL_DISTANCE 				315		// unit : mm
// maximal distance to left or right wall
#define 	SIDE_WALL_DISTANCE 					100		// unit : mm
// distance to left wall when mouse in middle
#define 	LEFT_WALL_DISTANCE_NO_SIDE_ERROR 	35.0	// unit : mm
// distance to right wall when mouse in middle
#define 	RIGHT_WALL_DISTANCE_NO_SIDE_ERROR 	35.0	// unit : mm
// position of micromouse when wall fades (140mm)
#define REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL 0.105 	// unit : mm
// position of micromouse when post fades (140mm)
#define REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST 0.100 	// unit : mm
// distance to front wall when micromouse doint dead end turn back
#define WALL_FRONT_DISTANCE_mm 					45.0 	// unit : mm
// distance offset to front wall when micromouse doint dead end turn back
#define WALL_FRONT_ANGLE_mm 					4.0 	// unit : mm
// sum distance to front wall when micromouse doing curve turn
#define WALL_FRONT_ANGLE_TURNING_SUM_mm 			220.0 	// unit : mm
// delta distance to front wall when micromouse doing curve turn
#define WALL_FRONT_ANGLE_TURNING_DELTA_mm 			18.0 	// unit : mm
#define WALL_FRONT_ANGLE_TURNING_DELTA_coef			0.0 	// 1.0 unit : mm to ms
// adjust distances LEFT_WALL_DISTANCE_NO_SIDE_ERROR
// adjust distances RIGHT_WALL_DISTANCE_NO_SIDE_ERROR
// to get smooth transition between one or two wall following.
//#define WALL_FOLLOWING_TRACE  //to see wall position when IDLE, or upload datalogger and watch telemetry

// First, we set the parameters for running forward with controlled acceleration, speed and position.

// 1.00. Define FIXED_MOVES for the following tests and configuration

// 1.01. Set forward speed of learning run and turns
#define 	X_SPEED 			0.35 		// unit : m/s

// 1.02. Set forward acceleration
#define 	X_MAX_ACCELERATION 	5.0F 		// unit : m/s^2
#define 	X_MAX_DECELERATION 	3.0F		// unit : m/s^2

// 1.03. Set PID parameters (Kp,Ki) for X SPEED PID and W SPEED PID
#define 	X_SPEED_KP 			600.0F		// this is a speed parameter
#define 	X_SPEED_KI 			10.0F		// this is a position parameter
#define 	X_SPEED_KD 			0.0F 		// this is an acceleration, so we do not use this input
#define 	W_SPEED_KP 			0.04 //0.08F 		// this is a speed parameter
#define 	W_SPEED_KI 			0.002 //0.002F 		// this is a position parameter
#define 	W_SPEED_KD 			0.0F 		// this is an acceleration, so we do not use this input
#define 	W_SPEED_KF 			0.08F 		// this is a feed forward speed input

// 1.04. Set wheel diameter and micro gear ratio
#define 	GEAR_BOX_RATIO		1.0F/50.0F	// 1:50
#define 	WHEEL_DIAMETER      0.0259F	// unit : mm
// the greater is the wheel diameter, lesser is the distance traveled

// 1.05, use visualization and physics to adjust all these parameters in order to :
// >>> physics : distance traveled must be equal to 180 mm
// >>> visualization : actual forward speed must follow current forward speed
//#define SC_START_STOP // 180mm
//#define SC_START_RUN_STOP // 360mm
//#define SC_START_RUN2_STOP // 540mm
//#define SC_START_RUN3_STOP // 720mm

// Next, we set the parameters for turning with controlled acceleration, speed and position.

// 2.00 Set the default gyro bias
#define 	INIT_GYRO_BIAS 						-1.530F	// unit : dps
#define 	GYRO_AUTOCAL_VARIANCE_THRESHOLD 	0.040F	// unit : dps^2 (don't change this)
// define IMU_TRACE to see variance and bias when IDLE

// 2.01 Set the sensitivity correction
// >>> physics using turn table : one turn gives 360° heading exactly (display heading)
#define 	GYRO_SENSITIVITY_CORRECTION 		1.00F //0.990F	// unit : %
// the greater is the correction, lesser the robot turn
// #define IMU_TRACE //to see heading when IDLE, or upload datalogger and watch telemetry
// use carré test to check real heading with estimated heading with telemetry

// 2.02 Set rotation speed of curve turn
// >>> use Trapezoidal-Curve-Turn-Profile-Generator
//     forward speed = X_SPEED
//     angle = 90
//     radius = 70
//     acceleration = W_MAX_ACCELERATION
//     deceleration = W_MAX_DECELERATION
//     max angular vel = W_SPEED
//     mouse width = 70
#define 	W_SPEED 			330.0F		// unit : dps
#define 	W_T1 				273 		// unit : ms
#define 	W_T2 				328			// unit : ms

// 2.03. Set rotation acceleration
#define 	W_MAX_ACCELERATION 	6000		// unit : dps^2
#define		W_MAX_DECELERATION 	6000		// unit : dps^2

// 2.04, use visualization and physics to adjust all these parameters in order to :
// >>> physics : return to home exactly after each turn
// >>> visualization : actual forward and rotation speeds must follow current speeds
//#define SC_F_L90_F
//#define SC_F_R90_F
//#define SC_F_R90_F_R90_F
//#define SC_SQUARE_TEST_1_TURN
#define SC_SQUARE_TEST_2_TURN
//#define SC_F_R90_R90_F
//#define SC_ZIGZAG

// Next, we set the parameters for wall following

// 3.01. Set PID parameters (Kp,Kd) for WALL POSITION PID
#define 	WALL_POSITION_KP 	0.5		// this is a position parameter
#define 	WALL_POSITION_KI 	0.0
#define 	WALL_POSITION_KD 	10.0	// this is a speed parameter

// 3.02. use visualization and physics to adjust all these parameters in order to :
// >>> physics : mouse moves forward in the middle of cell and avoids walls
//#define SC_START_RUN_STOP
//#define SC_START_RUN3_STOP

// Next, we set the parameters for wall-to-wall calibration

// 4.01. use visualization and physics to adjust all the following parameters
// REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL 0.120 	// unit : mm
// REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST 0.115 	// unit : mm
// in order to stop the mouse in the middle of the last cell
// > try first without wall
// > try with one side wall, two side walls, one post, two post, one post and a wall, ...
//#define SC_START_RUN1_STOP
//#define SC_START_RUN3_STOP

// Next, we set the parameters for front-wall cornering calibration

// 5.01. use visualization and physics to adjust all the following parameters
// WALL_FRONT_ANGLE_TURNING_mm
// in order to stop the mouse in the middle of the last cell, and never crash front wall corners; or side wall at the end of corners
// > try first without wall > carre must be OK
// > try with one front wall in front, two, three, .. at each corner
//#define SC_SQUARE_TEST_1_TURN
//#define SC_SQUARE_TEST_2_TURN

// Next, we set the parameters for uturn

// 6.01 Set timing for dead end rotation
#define 	W_U_SPEED 			360.0F		// unit : dps
#define 	W_U_T1_90			250 		// unit : ms
#define 	W_U_T2_90			305			// unit : ms
#define 	W_U_T1_180			500 		// unit : ms
#define 	W_U_T2_180			555			// unit : ms

// 6.02. Set front wall distance position  PID
#define X_WALL_FRONT_KP 1.0
#define X_WALL_FRONT_KI 0.0002
#define X_WALL_FRONT_KD 0.0

// 6.03. Set front wall angle position  PID
#define W_WALL_FRONT_KP 1.0
#define W_WALL_FRONT_KI 0.0001
#define W_WALL_FRONT_KD 0.0

// 6.04. use visualization and physics to adjust all the those parameters
// in order to stop the mouse in the middle of the last cell, and never crash front or side walls
// > try first without wall
// > try with one front wall, two or three walls
//#define SC_U_TURN
//#define SC_RUN1_UTURN_RUN1
//#define SC_TURN_RIGHT_TEST
//#define UTURN_TRACE

// max speed
#define X_SPEED_FAST_RUN 0.7 // m/s
#define X_SPEED_FAST_RUN_IMPROVED 1.0 // m/s

// RAND moves test stress
//#define RAND_MOVES_3
//#define RAND_MOVES_6
//#define RAND_MOVES_12
//#define RAND_MOVES_16

// AI

#endif

#ifdef __PREM__

// Recommendation : Always charge up battery to maximum capacity/voltage (8.4V)

// 0.01. Adjust voltage divider ration
// >>> use a power supply with voltage output settings to 8.00V
#define 	VOLTAGE_RATIO 			0.09F 		// unit : ratio
//#define VOLTAGE_TRACE //to see voltage when IDLE

// 0.02. Use "banc micromouse" to calibrate wall IR sensors (see. WallSensorCallibration.h)
// >> first check IR sensor orientation
// 		>> front wall sensor open 5°
// 		>> diagonal sensor point to middle of side walls when mouse placed before cell
// >> then use "banc micrmouse" to compute linear regression (theta)
//#define RAW_IR_TRACE //to calibrate wall IR sensors when IDLE
//#define CALIBRATED_IR_TRACE //to check calibrate wall IR sensors when IDLE

// 0.03. Use "banc micromouse " to set the distance to walls
// maximal distance to front wall (sum of both sensors)
#define 	FRONT_WALL_DISTANCE 				320		// unit : mm
// maximal distance to left or right wall
#define 	SIDE_WALL_DISTANCE 					65		// unit : mm
// distance to left wall when mouse in middle
#define 	LEFT_WALL_DISTANCE_NO_SIDE_ERROR 	30.0	// unit : mm
// distance to right wall when mouse in middle
#define 	RIGHT_WALL_DISTANCE_NO_SIDE_ERROR 	25.0	// unit : mm
// position of micromouse when wall fades (140mm)
#define REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL 0.110 	// unit : mm
// position of micromouse when post fades (140mm)
#define REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST 0.100 	// unit : mm
// distance to front wall when micromouse doint dead end turn back
#define WALL_FRONT_DISTANCE_mm 					35.0 	// unit : mm
// distance offset to front wall when micromouse doint dead end turn back
#define WALL_FRONT_ANGLE_mm 					0.0 	// unit : mm
// sum distance to front wall when micromouse doing curve turn
#define WALL_FRONT_ANGLE_TURNING_SUM_mm 			190.0 	// unit : mm
// delta distance to front wall when micromouse doing curve turn
#define WALL_FRONT_ANGLE_TURNING_DELTA_mm 			0.0 	// unit : mm
#define WALL_FRONT_ANGLE_TURNING_DELTA_coef			0.0 	// unit : mm to ms
// adjust distances LEFT_WALL_DISTANCE_NO_SIDE_ERROR
// adjust distances RIGHT_WALL_DISTANCE_NO_SIDE_ERROR
// to get smooth transition between one or two wall following.
//#define WALL_FOLLOWING_TRACE  //to see wall position when IDLE, or upload datalogger and watch telemetry

// First, we set the parameters for running forward with controlled acceleration, speed and position.

// 1.00. Define FIXED_MOVES for the following tests and configuration
//#define FIXED_MOVES // disable AI

// 1.01. Set forward speed of learning run and turns
#define 	X_SPEED 			0.35 		// unit : m/s

// 1.02. Set forward acceleration
#define 	X_MAX_ACCELERATION 	5.0F 		// unit : m/s^2
#define 	X_MAX_DECELERATION 	3.0F		// unit : m/s^2

// 1.03. Set PID parameters (Kp,Ki) for X SPEED PID and W SPEED PID
#define 	X_SPEED_KP 			600.0F		// this is a speed parameter
#define 	X_SPEED_KI 			10.0F		// this is a position parameter
#define 	X_SPEED_KD 			0.0F 		// this is an acceleration, so we do not use this input
#define 	W_SPEED_KP 			0.04 //0.08F 		// this is a speed parameter
#define 	W_SPEED_KI 			0.002 //0.002F 		// this is a position parameter
#define 	W_SPEED_KD 			0.0F 		// this is an acceleration, so we do not use this input
#define 	W_SPEED_KF 			0.08F 		// this is a feed forward speed input

// 1.04. Set wheel diameter and micro gear ratio
#define 	GEAR_BOX_RATIO		1.0F/50.0F	// 1:50
#define 	WHEEL_DIAMETER      0.0259F	// unit : mm
// the greater is the wheel diameter, lesser is the distance traveled

// 1.05, use visualization and physics to adjust all these parameters in order to :
// >>> physics : distance traveled must be equal to 180 mm
// >>> visualization : actual forward speed must follow current forward speed
//#define SC_START_STOP // 180mm
//#define SC_START_RUN_STOP // 360mm
//#define SC_START_RUN2_STOP // 540mm
//#define SC_START_RUN3_STOP // 720mm

// Next, we set the parameters for turning with controlled acceleration, speed and position.

// 2.00 Set the default gyro bias
#define 	INIT_GYRO_BIAS 						-5.6F	// unit : dps
#define 	GYRO_AUTOCAL_VARIANCE_THRESHOLD 	0.040F	// unit : dps^2 (don't change this)
//#define     IMU_TRACE // to see variance and bias when IDLE

// 2.01 Set the sensitivity correction
// >>> physics using turn table : one turn gives 360° heading exactly (display heading)
#define 	GYRO_SENSITIVITY_CORRECTION 		0.98F //0.990F	// unit : %
// the greater is the correction, lesser the robot turn
 #define IMU_TRACE //to see heading when IDLE, or upload datalogger and watch telemetry
// use carré test to check real heading with estimated heading with telemetry

// 2.02 Set rotation speed of curve turn
// >>> use Trapezoidal-Curve-Turn-Profile-Generator
//     forward speed = X_SPEED
//     angle = 90
//     radius = 70
//     acceleration = W_MAX_ACCELERATION
//     deceleration = W_MAX_DECELERATION
//     max angular vel = W_SPEED
//     mouse width = 70
#define 	W_SPEED 			330.0F		// unit : dps
#define 	W_T1 				273 		// unit : ms
#define 	W_T2 				328			// unit : ms

// 2.03. Set rotation acceleration
#define 	W_MAX_ACCELERATION 	6000		// unit : dps^2
#define		W_MAX_DECELERATION 	6000		// unit : dps^2

// 2.04, use visualization and physics to adjust all these parameters in order to :
// >>> physics : return to home exactly after each turn
// >>> visualization : actual forward and rotation speeds must follow current speeds
//#define SC_F_L90_F
//#define SC_F_R90_F
//#define SC_F_R90_F_R90_F
//#define SC_SQUARE_TEST_1_TURN
//#define SC_SQUARE_TEST_2_TURN
//#define SC_F_R90_R90_F
//#define SC_ZIGZAG

// Next, we set the parameters for wall following

// 3.01. Set PID parameters (Kp,Kd) for WALL POSITION PID
#define 	WALL_POSITION_KP 	0.5		// this is a position parameter
#define 	WALL_POSITION_KI 	0.0
#define 	WALL_POSITION_KD 	10.0	// this is a speed parameter

// 3.02. use visualization and physics to adjust all these parameters in order to :
// >>> physics : mouse moves forward in the middle of cell and avoids walls
//#define SC_START_RUN_STOP
//#define SC_START_RUN3_STOP

// Next, we set the parameters for wall-to-wall calibration

// 4.01. use visualization and physics to adjust all the following parameters
// REMAINING_DIST_RUN_AFTER_WALL_TO_NO_WALL 0.120 	// unit : mm
// REMAINING_DIST_RUN_AFTER_POST_TO_NO_POST 0.115 	// unit : mm
// in order to stop the mouse in the middle of the last cell
// > try first without wall
// > try with one side wall, two side walls, one post, two post, one post and a wall, ...
//#define SC_START_RUN1_STOP
//#define SC_START_RUN3_STOP

// Next, we set the parameters for front-wall cornering calibration

// 5.01. use visualization and physics to adjust all the following parameters
// WALL_FRONT_ANGLE_TURNING_mm
// in order to stop the mouse in the middle of the last cell, and never crash front wall corners; or side wall at the end of corners
// > try first without wall > carre must be OK
// > try with one front wall in front, two, three, .. at each corner
//#define SC_SQUARE_TEST_1_TURN
//#define SC_SQUARE_TEST_2_TURN

// Next, we set the parameters for uturn

// 6.01 Set timing for dead end rotation
#define 	W_U_SPEED 			360.0F		// unit : dps
#define 	W_U_T1_90			250 		// unit : ms
#define 	W_U_T2_90			305			// unit : ms
#define 	W_U_T1_180			500 		// unit : ms
#define 	W_U_T2_180			555			// unit : ms

// 6.02. Set front wall distance position  PID
#define X_WALL_FRONT_KP 1.0
#define X_WALL_FRONT_KI 0.0002
#define X_WALL_FRONT_KD 0.0

// 6.03. Set front wall angle position  PID
#define W_WALL_FRONT_KP 1.0
#define W_WALL_FRONT_KI 0.0001
#define W_WALL_FRONT_KD 0.0

// 6.04. use visualization and physics to adjust all the those parameters
// in order to stop the mouse in the middle of the last cell, and never crash front or side walls
// > try first without wall
// > try with one front wall, two or three walls
//#define SC_U_TURN
//#define SC_RUN1_UTURN_RUN1
//#define SC_TURN_RIGHT_TEST
//#define UTURN_TRACE

// max speed
#define X_SPEED_FAST_RUN 0.7 // m/s
#define X_SPEED_FAST_RUN_IMPROVED 1.0 // m/s

// RAND moves test stress
//#define RAND_MOVES_3
//#define RAND_MOVES_6
//#define RAND_MOVES_12
//#define RAND_MOVES_16

#endif

#ifdef __REMI__



#endif

/*****************************************************************************/


#endif /* PERSONNALISATION_H_ */
