/*
 * WallSensorCalibration.h
 *
 *  Created on: 26 avr. 2019
 *      Author: Robotique2
 */

#ifndef WALLSENSORCALIBRATION_H_
#define WALLSENSORCALIBRATION_H_

#include "WallSensor.h"
#include "personnalisation.h"

#include <math.h>

#ifdef __ALICE__

// DL, FL, FR, DR
float theta0[WALL_SENSOR_COUNT] =
	{
			-0.451667230000375,
			-0.214148588750962,
			-0.514381300859526,
			-0.383779201410519
	};
float theta1[WALL_SENSOR_COUNT] =
	{
			5000.3772726321713,
			5000.9480100218689,
			5500.7825797504011,
			5500.7356524911202
	};
float theta2[WALL_SENSOR_COUNT] =
	{
			500000.6972101126,
			500000.74478182383,
			500000.73746078881,
			500000.3538577068
	};

#endif

#ifdef __FRANCOIS__

// DL, FL, FR, DR
float theta0[WALL_SENSOR_COUNT] =
	{
			-31.451667230000375,
			-31.214148588750962,
			-37.514381300859526,
			-27.383779201410519
	};
float theta1[WALL_SENSOR_COUNT] =
	{
			2491.3772726321713,
			2943.9480100218689,
			3355.7825797504011,
			2152.7356524911202
	};
float theta2[WALL_SENSOR_COUNT] =
	{
			5091840.6972101126,
			-241993.74478182383,
			289366.73746078881,
			2049489.3538577068
	};

#endif

#ifdef __PATRICK__

#include "../Src/wall_sensor_table_patrick.h"

// DL, FL, FR, DR
float theta0[WALL_SENSOR_COUNT] =
	{
			56.701496501020365,
			-27.419185242867712,
			-40.225457807790477,
			-15.04983176196154
	};
float theta1[WALL_SENSOR_COUNT] =
	{
			-3165.6638985969357,
			2508.5685238238189,
			3596.5208049422517,
			1425.1961695396594
	};
float theta2[WALL_SENSOR_COUNT] =
	{
			138575963.03447884,
			40520731.405684084,
			20042235.41456762,
			70496551.214650705
	};

#endif

#ifdef __PREM__

// DL, FL, FR, DR
float theta0[WALL_SENSOR_COUNT] =
	{
			-33.77006272041082,
			-38.438670090025028,
			-31.287718504163188,
			-25.864724473373023
	};
float theta1[WALL_SENSOR_COUNT] =
	{
			2527.2522208504511,
			3612.4530285252222,
			2923.5317374817873,
			1951.5147672084659
	};
float theta2[WALL_SENSOR_COUNT] =
	{
			-229298.28086831042,
			95287.209617226617,
			456097.61435105314,
			-183996.05745171936
	};


#endif

#ifdef __REMI__

// DL, FL, FR, DR
float theta0[WALL_SENSOR_COUNT] =
	{
			-31.451667230000375,
			-31.214148588750962,
			-37.514381300859526,
			-27.383779201410519
	};
float theta1[WALL_SENSOR_COUNT] =
	{
			2491.3772726321713,
			2943.9480100218689,
			3355.7825797504011,
			2152.7356524911202
	};
float theta2[WALL_SENSOR_COUNT] =
	{
			5091840.6972101126,
			-241993.74478182383,
			289366.73746078881,
			2049489.3538577068
	};

#endif


//void raw_to_distance(int32_t const * raw, float * distance)
//{
//	for(uint32_t sensor=0;sensor<WALL_SENSOR_COUNT;++sensor)
//		distance[sensor] =
//				theta0[sensor] * 1.0 +
//				theta1[sensor] / sqrt( (float)raw[sensor] ) +
//				theta2[sensor] / pow( (float)raw[sensor], 2 );
//}

void raw_to_distance(int32_t const * raw, float * distance)
{
	distance[WALL_SENSOR_LEFT_DIAG]		=	table_dl[	raw[WALL_SENSOR_LEFT_DIAG]		];
	distance[WALL_SENSOR_LEFT_STRAIGHT]	=	table_fl[	raw[WALL_SENSOR_LEFT_STRAIGHT]	];
	distance[WALL_SENSOR_RIGHT_STRAIGHT]=	table_fr[	raw[WALL_SENSOR_RIGHT_STRAIGHT]	];
	distance[WALL_SENSOR_RIGHT_DIAG]	=	table_dr[	raw[WALL_SENSOR_RIGHT_DIAG]		];
}

#endif /* WALLSENSORCALIBRATION_H_ */
