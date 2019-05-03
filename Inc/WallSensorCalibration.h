/*
 * WallSensorCalibration.h
 *
 *  Created on: 26 avr. 2019
 *      Author: Robotique2
 */

#ifndef WALLSENSORCALIBRATION_H_
#define WALLSENSORCALIBRATION_H_

#include "WallSensor.h"

#include <math.h>

#define __PREM__


#ifdef __BERNARD__

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

#ifdef __PREM__

// DL, FL, FR, DR
float theta0[WALL_SENSOR_COUNT] =
	{
			-38.643731068414603,
			-38.438670090025028,
			-31.287718504163188,
			-28.624272892490396
	};
float theta1[WALL_SENSOR_COUNT] =
	{
			2712.9795780048316,
			3612.4530285252222,
			2923.5317374817873,
			1958.6399829409258
	};
float theta2[WALL_SENSOR_COUNT] =
	{
			91098.72716377629,
			95287.209617226617,
			456097.61435105314,
			-182198.20692549637
	};


#endif


void raw_to_distance(int32_t const * raw, float * distance)
{
	for(uint32_t sensor=0;sensor<WALL_SENSOR_COUNT;++sensor)
		distance[sensor] =
				theta0[sensor] * 1.0 +
				theta1[sensor] / sqrt( (float)raw[sensor] ) +
				theta2[sensor] / pow( (float)raw[sensor], 2 );
}


#endif /* WALLSENSORCALIBRATION_H_ */
