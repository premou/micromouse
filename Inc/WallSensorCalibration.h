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

#include "../Src/wall_sensor_table_alice.h"
//#include "../Src/wall_sensor_table_francois.h"

#endif

#ifdef __FRANCOIS__

#include "../Src/wall_sensor_table_francois.h"

#endif

#ifdef __PATRICK__

#include "../Src/wall_sensor_table_patrick.h"

#endif

#ifdef __PREM__

#include "../Src/wall_sensor_table_prem.h"

#endif

#ifdef __REMI__

#include "../Src/wall_sensor_table_remi.h"

#endif

void raw_to_distance(int32_t const * raw, float * distance)
{
	distance[WALL_SENSOR_LEFT_DIAG]		=	table_dl[	raw[WALL_SENSOR_LEFT_DIAG]		];
	distance[WALL_SENSOR_LEFT_STRAIGHT]	=	table_fl[	raw[WALL_SENSOR_LEFT_STRAIGHT]	];
	distance[WALL_SENSOR_RIGHT_STRAIGHT]=	table_fr[	raw[WALL_SENSOR_RIGHT_STRAIGHT]	];
	distance[WALL_SENSOR_RIGHT_DIAG]	=	table_dr[	raw[WALL_SENSOR_RIGHT_DIAG]		];
}

#endif /* WALLSENSORCALIBRATION_H_ */
