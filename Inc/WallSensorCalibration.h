/*
 * WallSensorCalibration.h
 *
 *  Created on: 26 avr. 2019
 *      Author: Robotique2
 */

#ifndef WALLSENSORCALIBRATION_H_
#define WALLSENSORCALIBRATION_H_

#include "WallSensor.h"

#define __BERNARD__


#ifdef __BERNARD__

// DL, FL, FR, DR
float theta0[WALL_SENSOR_COUNT] =
	{
			89.783549163781984,
			89.783549163781984,
			89.783549163781998,
			89.783549163781984
	};
float theta1[WALL_SENSOR_COUNT] =
	{
			51.595267655205198,
			50.579665361660197,
			50.922560384729024,
			51.548023645188088
	};
float mu[WALL_SENSOR_COUNT] =
	{
			0.0017861672732610281,
			0.0020687933547682788,
			0.0016429757797189024,
			0.0024579590390316837
	};
float sigma[WALL_SENSOR_COUNT] =
	{
			0.0010247903129681592,
			0.0015631163274954065,
			0.0011504001026292729,
			0.0015151345784583646
	};

#else

#endif

void raw_to_distance(int32_t const * raw, float * distance)
{
	for(uint32_t sensor=0;sensor<WALL_SENSOR_COUNT;++sensor)
		distance[sensor] = theta1[sensor] * ( 1.0/(float)raw[sensor] - mu[sensor] ) / sigma[sensor] + theta0[sensor];
}


#endif /* WALLSENSORCALIBRATION_H_ */
