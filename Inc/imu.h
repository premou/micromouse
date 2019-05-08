/*
 * imu.h
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */

#ifndef APPLICATION_USER_HAL_IMU_H_
#define APPLICATION_USER_HAL_IMU_H_

#include "stm32f7xx_hal.h"

#ifdef __cplusplus
 extern "C" {
#endif

 // GYRO return codes
#define GYRO_OK 0
#define GYRO_NOT_DETECTED 1
#define GYRO_NOT_IDENTIFIED 2
#define GYRO_SETUP_FAILURE 2

uint32_t gyro_init(); 	// initialise and configure gyro through I2C, return GYRO error code (ZERO is OK)
void gyro_update(float duration_s); 	// retreive gyro measures thourgh I2C and do some calculation
float gyro_get_dps(); 	// get the current rotation speed in degrees per second, ccw : positive dps, cw : negative dps
void gyro_auto_calibrate();

void gyro_reset_heading();
float gyro_get_heading();

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_USER_HAL_IMU_H_ */



