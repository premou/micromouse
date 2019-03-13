/*
 * imu.h
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */

#ifndef APPLICATION_USER_HAL_IMU_H_
#define APPLICATION_USER_HAL_IMU_H_

#include "stm32f7xx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
 extern "C" {
#endif

 // GYRO return codes
#define GYRO_OK 0
#define GYRO_NOT_DETECTED 1
#define GYRO_NOT_IDENTIFIED 2
#define GYRO_SETUP_FAILURE 2

uint32_t gyro_init(); 	// initialise and configure gyro through I2C, return GYRO error code (ZERO is OK)
void gyro_update(); 	// retreive gyro measures thourgh I2C and do some calculation
float gyro_get_dps(); 	// get the current rotation speed in degrees per second

#ifdef __cplusplus
}
#endif

 #endif /* APPLICATION_USER_HAL_IMU_H_ */





 ///**
 //  * @brief  IMU Handler Structure definition
 //  */
 //typedef struct {
 //	I2C_HandleTypeDef * hi2c;		/*!< Specifies the GPIO pin used to generate STEP/PULSE signal. */
 //	// Acquisition Data
 //	float * sensitivity_correction; // 1.0 default
 //	int16_t raw_rate; 			// digit
 //	float scaled_rate; 		// dps
 //	// Bias Correction
 //	float rate_mean; 		// Moyenne du gyroscope
 //	float rate_qmean;		// Moyenne quadratique du gyroscope
 //	float rate_deviation; 	// Variance du gyroscope
 //	float rate_bias;
 //	// User data
 //	float rate; // dps
 //	double heading; // degres
 //} IMU_HandleTypeDef;
//void HAL_IMU_Init();
//
//void HAL_IMU_Add(
//		IMU_HandleTypeDef * himu,
//		I2C_HandleTypeDef * hi2c,
//		float * sensitivity_correction
//);
//
//void HAL_IMU_Read_Sensors(IMU_HandleTypeDef * himu, bool autocalibrate, float period_us);
//
//void HAL_IMU_Zero_Heading(IMU_HandleTypeDef * himu);
