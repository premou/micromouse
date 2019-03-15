/*
 * imu.c
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */

#include "stm32f7xx_hal.h"
#include "serial.h"
#include "imu.h"

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
#define CTRL10_C_value_init 0x20
#define CTRL2_G_value_init 0x74
#define ANGULAR_RATE_SENSITIVITY_500 0.0175 //p.15 datasheet
// globals
extern I2C_HandleTypeDef hi2c3;
extern HAL_Serial_Handler com;

// private data ///////////////////////////////////////////////////////////////

// TODO : GYRO CONTEXT with measure, calibration data, etc
typedef struct {
	int16_t raw_value;
	float rate; //dps
}ctx_gyro;

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
	// send the register address to I2C device
	uint8_t data_buf[]= {register_address, data};
	*res = HAL_I2C_Master_Transmit(&hi2c3, device_address << 1, data_buf , 2, 10);
}

// public functions ///////////////////////////////////////////////////////////

uint32_t gyro_init()
{
	ctx.raw_value = 0;
	ctx.rate = 0.0;
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
	// To Be Completed

	// AN : https://www.pololu.com/file/0J1088/LSM6DS33-AN4682.pdf p.23


	return GYRO_OK;
}

void gyro_update()
{
	HAL_StatusTypeDef result;
	// TODO : read Z gyro raw value (16bits)
	uint8_t res_read_H = gyro_read_8bit_register(GYRO_I2C_ADDRESS, OUTZ_H_G, &result);
	uint8_t res_read_L = gyro_read_8bit_register(GYRO_I2C_ADDRESS, OUTZ_L_G, &result);
	ctx.raw_value = ((uint16_t)(res_read_H) << 8) + (uint16_t) res_read_L;
	ctx.rate = (float)(ctx.raw_value*ANGULAR_RATE_SENSITIVITY_500);
	// TODO : do continious and power-on gyro calibrations (drift)
	// TODO : apply drift and sensitivity corrections to raw measure
	// TODO : store last corrected measure in internal state
}

float gyro_get_dps()
{
	// TODO : return last corrected measure from internal state
	return ctx.rate;
}



///* Private Configuration Data ----------------------------------------------------------*/
//
//#define ODR833
////#define ODR1666
//
//#ifdef ODR833
//
//	static float const initial_rate_bias = -106.0f;
//	static float const mean_alpha = 0.01f; // EWMA
//	static float const bias_alpha = 0.005f; // EWMA
//	static float const deviation_threshold = 22.0f; // raw unit 22.0 minimum
//	static float const rate_alpha = 0.9f; // EWMA
//
//#else
//
//	static float const initial_rate_bias = -110.0f;
//	static float const mean_alpha = 0.002f; // EWMA
//	static float const bias_alpha = 0.01f; // EWMA
//	static float const deviation_threshold = 90.0f; // 86 minimum raw unit
//	static float const rate_alpha = 0.65f; // EWMA
//
//#endif
//
//	static float const rate_sensitivity = 0.01750f; // 500dps sensitivity
//
//typedef enum
//{
//	IMU_READ_CHECK,
//	IMU_WRITE,
//	IMU_WRITE_AND_CHECK,
//} HAL_Imu_AccessTypeDef;
//
//typedef struct
//{
//	int addr;
//	int reg;
//	uint8_t value;
//	HAL_Imu_AccessTypeDef access;
//} HAL_Imu_ConfigurationElementTypeDef;
//
//static HAL_Imu_ConfigurationElementTypeDef const gyro_config[] = {
//
//		{ IMU_LSM6DS33_ADDRESS, IMU_LSM6DS33_REGISTER_WHO_AM_I, IMU_LSM6DS33_WHO_AM_I, IMU_READ_CHECK },
//		//! check sensor communication
//
//		{ IMU_LSM6DS33_ADDRESS, IMU_LSM6DS33_REGISTER_FIFO_CTRL1,	0x00, IMU_WRITE_AND_CHECK},
//		// write 0000 0000 = 0x00
//		// [7-0]: FTH [7:0] = 00000000b - FTH = 0
//
//		{ IMU_LSM6DS33_ADDRESS, IMU_LSM6DS33_REGISTER_FIFO_CTRL2,	0x00, IMU_WRITE_AND_CHECK},
//		// write 0000 0000 = 0x00
//		// [7]: TIMER_PEDO_FIFO_EN = 0b - Disable
//		// [6]: TIMER_PEDO_FIFO_DRDY = 0b - Disable
//		// [5-4]: unused
//		// [3-0]: FTH [11:8] =0000b - FTH = 0
//
//		{ IMU_LSM6DS33_ADDRESS, IMU_LSM6DS33_REGISTER_FIFO_CTRL3,	0x00, IMU_WRITE_AND_CHECK},
//		// write 0000 0000 = 0x00
//		// [7-6]: unused
//		// [5-3]: DEC_FIFO_GYRO [2:0] = 000b - No Decimation
//		// [2-0]: DEC_FIFO_XL [2:0] = 000b - Accelerometer sensor not in FIFO
//
//		{ IMU_LSM6DS33_ADDRESS, IMU_LSM6DS33_REGISTER_FIFO_CTRL4,	0x00, IMU_WRITE_AND_CHECK},
//		// write 0000 0000 = 0x00
//		// [7]: unused
//		// [6]: ONLY_HIGH_DATA = 0b -  disable MSByte only memorization in FIFO
//		// [5-3]: TIMER_PEDO_DEC_FIFO [2:0] = 000b - Third FIFO data set not in FIFO
//		// [2-0]: unused
//
//		{ IMU_LSM6DS33_ADDRESS, IMU_LSM6DS33_REGISTER_FIFO_CTRL5,	0x00, IMU_WRITE_AND_CHECK},
//		// RESET/EMPTY FIFO
//
//#ifdef ODR833
//
//		{ IMU_LSM6DS33_ADDRESS, IMU_LSM6DS33_REGISTER_CTRL2_G,	0x74, IMU_WRITE_AND_CHECK}, // 833 ODR
//		// write 0111 0100= 0x74
//		// [7-4]: ODR_G [3:0] = 0111b - ODR = 833 Hz
//		// [3-2]: FS_G [1:0] = 01b - Scale =  500 dps
//		// [1]: FS_125 = 0b - Disable Gyroscope full-scale at 125 dps
//		// [0]: unused
//
//#else
//
//		{ IMU_LSM6DS33_ADDRESS, IMU_LSM6DS33_REGISTER_CTRL2_G,	0x84, IMU_WRITE_AND_CHECK}, // 1666 ODR
//		// write 1000 0100= 0x84
//		// [7-4]: ODR_G [3:0] = 1000b - ODR = 1666 Hz
//		// [3-2]: FS_G [1:0] = 01b - Scale =  500 dps
//		// [1]: FS_125 = 0b - Disable Gyroscope full-scale at 125 dps
//		// [0]: unused
//
//#endif
//
//		{ IMU_LSM6DS33_ADDRESS, IMU_LSM6DS33_REGISTER_CTRL3_C,	0x46, IMU_WRITE_AND_CHECK},
//		// write 0100 0110= 0x46
//		// [7]: BOOT = 0
//		// [6]: BDU = 1 - output registers not updated until MSB and LSB have been read
//		// [5]: H_LACTIVE = 0
//		// [4]: PP_OD = 0
//		// [3]: SIM = 0
//		// [2]: IF_INC = 1 - Register address automatically incremented during a multiple byte access
//		// [1]: BLE = 1 -  data MSB @ lower address
//		// [0]: SW_RESET = 0
//
//		{ IMU_LSM6DS33_ADDRESS, IMU_LSM6DS33_REGISTER_CTRL10_C,	0x38, IMU_WRITE_AND_CHECK},
//		// write 0011 1000= 0x38
//		// [7]: unused
//		// [6]: unused
//		// [5]: Zen_G = 1 - Gyroscope yaw axis (Z) output enable
//		// [4]: Yen_G = 0
//		// [3]: Xen_G = 0
//		// [2]: FUNC_EN = 0
//		// [1]: PEDO_RST_STEP = 0
//		// [0]: SIGN_MOTION_EN = 0
//
//		{0,0,0,0} //! end of config
//};
//
///* Private functions ----------------------------------------------------------*/
//
//void HAL_IMU_Gyro_Bias_Correction(IMU_HandleTypeDef * himu, bool autocalibrate)
//{
//	// Update rate variance (EWMA)
//	himu->rate_mean = (1-mean_alpha)*himu->rate_mean + mean_alpha*himu->raw_rate;
//	himu->rate_qmean = (1-mean_alpha)*himu->rate_qmean + mean_alpha*himu->raw_rate*himu->raw_rate;
//	himu->rate_deviation = sqrt(himu->rate_qmean-himu->rate_mean*himu->rate_mean);
//
//	// Update bias
//	static uint32_t counter = 0;
//	if(counter++>2000 && autocalibrate) // wait for 2000 data before updating rate bias
//	{
//		if(himu->rate_deviation<=deviation_threshold)
//		{
//			himu->rate_bias = himu->rate_bias*(1.0f-bias_alpha) + bias_alpha*himu->rate_mean;
//		}
//	}
//}
//
///*****************************************************************************/
//
//void HAL_IMU_Process_Failure(void)
//{
//	while(1);
//}
//
///* Public functions ----------------------------------------------------------*/
//
//void HAL_IMU_Init(void)
//{
//	HAL_Delay(300); // Boot time of MEMS
//}
//
//void HAL_IMU_Add(
//		IMU_HandleTypeDef * himu,
//		I2C_HandleTypeDef * hi2c,
//		float * sensitivity_correction
//)
//{
//	// Init
//	himu->hi2c = hi2c;
//	himu->sensitivity_correction = sensitivity_correction;
//	// Data Init
//	himu->raw_rate = 0;
//	himu->scaled_rate = 0.0f;
//	// Bias Correction Init
//	himu->rate_mean = 0.0f;
//	himu->rate_qmean = 0.0f;
//	himu->rate_deviation = 0.0f;
//	himu->rate_bias = initial_rate_bias;
//	// User Data Init
//	himu->rate = 0.0f;
//	himu->heading = 0.0f;
//
//	/// configure
//	HAL_StatusTypeDef result = HAL_OK;
//	{
//		uint8_t donnee = 0x5A;
//		HAL_Imu_ConfigurationElementTypeDef const * ptr = gyro_config;
//		while(ptr->addr!=0)
//		{
//			if(ptr->access!=IMU_READ_CHECK)
//			{
//				result = HAL_I2C_Mem_Write(himu->hi2c, ptr->addr<<1, ptr->reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&(ptr->value), 1, 100);
//				if(result!=HAL_OK)
//					while(1);
//			}
//			if(ptr->access!=IMU_WRITE)
//			{
//				result = HAL_I2C_Mem_Read(himu->hi2c, ptr->addr<<1, ptr->reg, I2C_MEMADD_SIZE_8BIT, &donnee, 1, 100);
//				if(result!=HAL_OK)
//					while(1);
//				if(donnee != ptr->value)
//					while(1);
//			}
//			++ptr;
//		}
//	}
//}
//
//void HAL_IMU_Zero_Heading(IMU_HandleTypeDef * himu)
//{
//	himu->heading = 0.0f;
//}
//
//void HAL_IMU_Read_Sensors(IMU_HandleTypeDef * himu, bool autocalibrate, float period_us)
//{
//	static uint8_t buffer[2];
//	HAL_StatusTypeDef result = HAL_OK;
//	result = HAL_I2C_Mem_Read(himu->hi2c, (IMU_LSM6DS33_ADDRESS)<<1, IMU_LSM6DS33_REGISTER_OUTZ_L_G, I2C_MEMADD_SIZE_8BIT, buffer, 2, 10);
//	if(result!=HAL_OK)
//	{
//		//HAL_IMU_Process_Failure();
//	}
//	else
//	{
//		himu->raw_rate = (int16_t)((buffer[0] << 8) | buffer[1]);
//		// bias
//		HAL_IMU_Gyro_Bias_Correction(himu,autocalibrate);
//		// scale rate
//		himu->scaled_rate = ((float)(himu->raw_rate)-himu->rate_bias)*rate_sensitivity**himu->sensitivity_correction;
//		// update user data
//		himu->rate = himu->rate*(1.0f-rate_alpha) + himu->scaled_rate*rate_alpha;
//		himu->heading += (double)(himu->scaled_rate)*period_us;
//	}
//}
//
