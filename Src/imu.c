/*
 * imu.c
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */

#include "stm32f7xx_hal.h"
#include "serial.h"
#include "imu.h"
#include "robot_math.h"
#include "personnalisation.h"

#include <string.h>
#include <math.h>

// DS : https://www.st.com/resource/en/datasheet/lsm6ds33.pdf
// AN : https://www.pololu.com/file/0J1088/LSM6DS33-AN4682.pdf

// device I2C address (LSM6DS33)
#define GYRO_I2C_ADDRESS 	0x6b

// device internal register addresses (LSM6DS33)
#define INT1_CTRL 			0x0D
#define INT2_CTRL 			0x0E
#define WHO_AM_I_ADDRESS 	0x0F
#define CTRL2_G 			0x11
#define CTRL3_C 			0x12
#define CTRL4_C 			0x13
#define CTRL5_C 			0x14
#define CTRL6_C 			0x15
#define CTRL7_C 			0x16
#define CTRL10_C 			0x19
#define STATUS_REG 			0x1E
#define OUTZ_L_G 			0x26
#define OUTZ_H_G 			0x27

// register default value (LSM6DS33)
#define WHO_AM_I_VALUE 0x69

// register configuration values (LSM6DS33)
#define CTRL10_C_value_init 0x20
#define CTRL2_G_value_init  0x74
#define CTRL3_C_value_init  0x40 // BDU=1
// constants
#define ANGULAR_RATE_SENSITIVITY_500 0.0175 // factory sensititvy (p.15 datasheet)
// TODO tune sensitivity using turn table

// globals
extern I2C_HandleTypeDef hi2c3;
extern HAL_Serial_Handler com;

// private data ///////////////////////////////////////////////////////////////

typedef struct {
	int16_t raw_value; // 12-bit measure
	float rate; //dps
	float bias; // dps
	float heading; //degres
} ctx_gyro;

static ctx_gyro ctx;

// private functions //////////////////////////////////////////////////////////

// read helper for I2C operation
// input : device (7bit, not shifted) and register (8bit) addresses
// output : register value (8bit)
uint8_t gyro_read_8bit_register(
		uint8_t device_address,
		uint8_t register_address,
		HAL_StatusTypeDef * res
	)
{
	// send the register address to I2C device
	*res = HAL_I2C_Master_Transmit(&hi2c3, device_address << 1, &register_address , 1, 10);
	if(*res==HAL_OK)
	{
		uint8_t data = 0;
		// read the register value from I2C device
		*res = HAL_I2C_Master_Receive(&hi2c3, device_address << 1, &data, 1, 10);
		if(*res==HAL_OK)
		{
			// return the register value
			return data;
		}
		else
		{
			return 0xFF;
		}
	}
	else
	{
		return 0xFF;
	}
}

// write helper for I2C operation
// input : device (7bit, not shifted) and register (8bit) addresses, register value (8bit)
void gyro_write_8bit_register(
		uint8_t device_address,
		uint8_t register_address,
		uint8_t data,
		HAL_StatusTypeDef * res
	)
{
	// send the register address and data to I2C device
	uint8_t data_buf[]= {register_address, data};
	*res = HAL_I2C_Master_Transmit(&hi2c3, device_address << 1, data_buf , 2, 10);
}

// public functions ///////////////////////////////////////////////////////////

uint32_t gyro_init()
{
	ctx.raw_value = 0;
	ctx.rate = 0.0;
	ctx.bias = INIT_GYRO_BIAS;
	ctx.heading = 0.0f;
	HAL_StatusTypeDef result;
	uint8_t who_am_i = gyro_read_8bit_register(GYRO_I2C_ADDRESS,WHO_AM_I_ADDRESS,&result);
	if(result != HAL_OK)
	{
		return GYRO_NOT_DETECTED;
	}
	if(who_am_i != WHO_AM_I_VALUE)
	{
		return GYRO_NOT_IDENTIFIED;
	}
	gyro_write_8bit_register(GYRO_I2C_ADDRESS, CTRL10_C, CTRL10_C_value_init, &result);
	uint8_t res_read = gyro_read_8bit_register(GYRO_I2C_ADDRESS, CTRL10_C, &result);
	if(res_read!=CTRL10_C_value_init)
	{
		return GYRO_SETUP_FAILURE;
	}
	gyro_write_8bit_register(GYRO_I2C_ADDRESS, CTRL2_G, CTRL2_G_value_init, &result);
	res_read = gyro_read_8bit_register(GYRO_I2C_ADDRESS, CTRL2_G, &result);
	if(res_read!=CTRL2_G_value_init)
	{
		return GYRO_SETUP_FAILURE;
	}
	gyro_write_8bit_register(GYRO_I2C_ADDRESS, CTRL3_C, CTRL3_C_value_init, &result); // enforce BDU
	res_read = gyro_read_8bit_register(GYRO_I2C_ADDRESS, CTRL3_C, &result);
	if(res_read!=CTRL3_C_value_init)
	{
		return GYRO_SETUP_FAILURE;
	}
	return GYRO_OK;
}

void gyro_update(float duration_s)
{
	HAL_StatusTypeDef result;
	// TODO : burst read (16bits)
	uint8_t res_read_H = gyro_read_8bit_register(GYRO_I2C_ADDRESS, OUTZ_H_G, &result);
	uint8_t res_read_L = gyro_read_8bit_register(GYRO_I2C_ADDRESS, OUTZ_L_G, &result);
	ctx.raw_value = ((uint16_t)(res_read_H) << 8) + (uint16_t) res_read_L;
	ctx.rate = (float)(ctx.raw_value*ANGULAR_RATE_SENSITIVITY_500*GYRO_SENSITIVITY_CORRECTION);
	ctx.heading += (ctx.rate - ctx.bias)*duration_s;
}

float gyro_get_dps()
{
	return ctx.rate- ctx.bias;
}

void gyro_reset_heading()
{
	ctx.heading = 0.0F;
}

float gyro_get_heading()
{
	return ctx.heading;
}

float mean = 0.0;
float variance = 0.0;
float alpha_mean_update = 0.01;
float alpha_variance_update = 0.05;
float alpha_bias_update = 0.01;

void gyro_auto_calibrate()
{
	static uint32_t time = 0;
	if(HAL_GetTick() > time + 20)
	{
		time = HAL_GetTick();
		gyro_update(0.02);
		// update mean and variance
		mean = alpha_mean_update *ctx.rate + (1.0-alpha_mean_update) * mean;
		variance = alpha_variance_update * pow( ctx.rate-mean,2)  + (1.0-alpha_variance_update) * variance;
		// if mean stable, update bias
		if(variance<GYRO_AUTOCAL_VARIANCE_THRESHOLD)
			ctx.bias = alpha_bias_update*mean + (1.0-alpha_bias_update)* ctx.bias;

#ifdef IMU_TRACE
		HAL_Serial_Print(&com,"dps=%d m=%d v=%d b=%d h=%d\r\n",
				(int32_t)(ctx.rate*1000.0),
				(int32_t)(mean*1000.0),
				(int32_t)(variance*1000.0),
				(int32_t)(ctx.bias*1000.0),
				(int32_t)(ctx.heading)
								  );
#endif
	}
}
