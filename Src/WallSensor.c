/* Includes ------------------------------------------------------------------*/
#include "WallSensor.h"
#include "WallSensorCalibration.h"
#include "timer_us.h"
#include "main.h"
#include "personnalisation.h"

#include <string.h>
#include <math.h>

/* APP settings ------------------------------------------------------------------*/

static uint16_t const id_to_pin[WALL_SENSOR_COUNT] = {
	IR_LED_DL_Pin, // DL
	IR_LED_FL_Pin, // FL
	IR_LED_FR_Pin, // FR
	IR_LED_DR_Pin  // DR
};

static GPIO_TypeDef * const id_to_port[WALL_SENSOR_COUNT] = {
	IR_LED_DL_GPIO_Port,
	IR_LED_FL_GPIO_Port,
	IR_LED_FR_GPIO_Port,
	IR_LED_DR_GPIO_Port
};

static uint16_t const id_to_channel[WALL_SENSOR_COUNT] = {
    ADC_CHANNEL_6, // DL
    ADC_CHANNEL_4, // FL
    ADC_CHANNEL_7, // FR
    ADC_CHANNEL_5  // DR
};

/* APP private data  ------------------------------------------------------------------*/

typedef struct
{
	int32_t raw[WALL_SENSOR_COUNT];
	float distance[WALL_SENSOR_COUNT];
} wall_sensor_ctx;

static wall_sensor_ctx ctx;

/* APP private functions  ------------------------------------------------------------------*/

extern ADC_HandleTypeDef hadc1;

int32_t read_adc(ADC_HandleTypeDef * phadc, uint32_t sensor_id)
{
    ADC_ChannelConfTypeDef sConfig;
    sConfig.Channel = id_to_channel[sensor_id];
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES; // = 18µs = 15 + 480 cycles ADC à 108MHz/4
    HAL_ADC_ConfigChannel(phadc,&sConfig);
    if (HAL_ADC_Start(phadc)!= HAL_OK)
    		return -1;
    while(HAL_ADC_PollForConversion(phadc,100)==HAL_BUSY);
    uint32_t value = HAL_ADC_GetValue(phadc);
    if (HAL_ADC_Stop(phadc)!= HAL_OK)
		return -1;
    return value;
}

void wall_sensor_update_one(uint32_t sensor_id)
{
	HAL_GPIO_WritePin(id_to_port[sensor_id],id_to_pin[sensor_id],GPIO_PIN_SET); // Turn ON IR LED
	timer_us_delay(100); // Wait for 100us (IR LED warm-up)
	ctx.raw[sensor_id]= read_adc(&hadc1, sensor_id);
	HAL_GPIO_WritePin(id_to_port[sensor_id],id_to_pin[sensor_id],GPIO_PIN_RESET); // Turn OFF IR LED
	timer_us_delay(30); // Wait for 50us (IR LED cooling)
}

/* APP public functions  ------------------------------------------------------------------*/

void wall_sensor_init()
{
	memset(&ctx,0,sizeof(wall_sensor_ctx));
}

void wall_sensor_update()
{
	// 1) FL sensor
	wall_sensor_update_one(WALL_SENSOR_LEFT_STRAIGHT);
	// 1) FR sensor
	wall_sensor_update_one(WALL_SENSOR_RIGHT_STRAIGHT);
	// 1) DL sensor
	wall_sensor_update_one(WALL_SENSOR_LEFT_DIAG);
	// 1) DR sensor
	wall_sensor_update_one(WALL_SENSOR_RIGHT_DIAG);
	// convert raw to distance
	raw_to_distance(ctx.raw,ctx.distance);
}

int32_t wall_sensor_get_raw(uint32_t sensor_id)
{
	return ctx.raw[sensor_id];
}


float wall_sensor_get_dist(uint32_t sensor_id)
{
	return ctx.distance[sensor_id];
}

///////////////////////////////////////////////////////////////////////////////

bool wall_sensor_is_left_wall_detected()
{
	return ctx.distance[WALL_SENSOR_LEFT_DIAG] < SIDE_WALL_DISTANCE; //mm
}

bool wall_sensor_is_front_wall_detected()
{
	return (ctx.distance[WALL_SENSOR_RIGHT_STRAIGHT]+ctx.distance[WALL_SENSOR_LEFT_STRAIGHT]) < FRONT_WALL_DISTANCE; //mm
}

bool wall_sensor_is_right_wall_detected()
{
	return ctx.distance[WALL_SENSOR_RIGHT_DIAG] < SIDE_WALL_DISTANCE; //mm
}

float wall_sensor_get_side_error(){

	if(wall_sensor_is_left_wall_detected() && wall_sensor_is_right_wall_detected())
	{
		return ctx.distance[WALL_SENSOR_RIGHT_DIAG] - ctx.distance[WALL_SENSOR_LEFT_DIAG] + WALL_POSITION_OFFSET;
	}
	else if(wall_sensor_is_left_wall_detected())
	{
		return LEFT_WALL_DISTANCE_NO_SIDE_ERROR - ctx.distance[WALL_SENSOR_LEFT_DIAG] ;
	}
	else if(wall_sensor_is_right_wall_detected())
	{
		return ctx.distance[WALL_SENSOR_RIGHT_DIAG] - RIGHT_WALL_DISTANCE_NO_SIDE_ERROR;
	}
	else
	{
		return(0.0);
	}

	// This part of code should be never reached
	return(0.0);
}


