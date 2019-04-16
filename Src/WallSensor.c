/* Includes ------------------------------------------------------------------*/
#include "WallSensor.h"
//#include "WallSensorCalibration.h"
#include "timer_us.h"
#include "main.h"
#include "controller.h"
#include <string.h>

/* APP settings ------------------------------------------------------------------*/

#define WALL_POSITION_MIN_DEFAULT 300
#define WALL_POSITION_MAX_DEFAULT 1900
//#define WALL_POSITION_MIN 700
//#define WALL_POSITION_MAX 2000
#define WALL_FRONT_DISTANCE 160
#define W_MIDDLE_LEFT 850			//su
#define W_MIDDLE_RIGHT 850		//su

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

typedef struct
{
	int32_t raw[WALL_SENSOR_COUNT];

	int32_t wall_position_min;
	int32_t wall_position_max;
} wall_sensor_ctx;

static wall_sensor_ctx ctx;

void wall_sensor_init()
{
	memset(&ctx,0,sizeof(wall_sensor_ctx));
	ctx.wall_position_min = WALL_POSITION_MIN_DEFAULT;
	ctx.wall_position_max = WALL_POSITION_MAX_DEFAULT;
}

int32_t read_adc(ADC_HandleTypeDef * phadc, uint32_t sensor_id); // forward declaration

extern ADC_HandleTypeDef hadc1;

void wall_sensor_update_one(uint32_t sensor_id)
{
	HAL_GPIO_WritePin(id_to_port[sensor_id],id_to_pin[sensor_id],GPIO_PIN_SET); // Turn ON IR LED
	timer_us_delay(100); // Wait for 100us (IR LED warm-up)
	ctx.raw[sensor_id]= read_adc(&hadc1, sensor_id);
	HAL_GPIO_WritePin(id_to_port[sensor_id],id_to_pin[sensor_id],GPIO_PIN_RESET); // Turn OFF IR LED
	timer_us_delay(30); // Wait for 50us (IR LED cooling)
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
}

int32_t wall_sensor_get(uint32_t sensor_id)
{
	return ctx.raw[sensor_id];
}


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

int32_t wall_sensor_get_side_error(){
	wall_sensor_update_one(WALL_SENSOR_LEFT_DIAG);
	wall_sensor_update_one(WALL_SENSOR_RIGHT_DIAG);

	if(wall_sensor_both_wall_presence())
	{
		return ctx.raw[WALL_SENSOR_LEFT_DIAG] - ctx.raw[WALL_SENSOR_RIGHT_DIAG];
	}
	else if(wall_sensor_wall_left_presence())
	{
		return ctx.raw[WALL_SENSOR_LEFT_DIAG] - W_MIDDLE_RIGHT;
	}
	else if(wall_sensor_wall_right_presence())
	{
		return W_MIDDLE_LEFT - ctx.raw[WALL_SENSOR_RIGHT_DIAG];
	}

	// This part of code should be never reached
	return(0);
}

bool wall_sensor_wall_left_presence()
{
	if((ctx.raw[WALL_SENSOR_LEFT_DIAG] > ctx.wall_position_min) && (ctx.raw[WALL_SENSOR_LEFT_DIAG] < ctx.wall_position_max))
	{
		//there is a wall on the left
 		return true;
	}
	else
	{
		//there is no wall on the left
		return false;
	}
}

bool wall_sensor_wall_right_presence()
{
	if((ctx.raw[WALL_SENSOR_RIGHT_DIAG] > ctx.wall_position_min) && (ctx.raw[WALL_SENSOR_RIGHT_DIAG] < ctx.wall_position_max))
	{
		//there is a wall on the right
 		return true;
	}
	else
	{
		//there is no wall on the right
		return false;
	}
}

bool wall_sensor_both_wall_presence()
{
	if(((ctx.raw[WALL_SENSOR_LEFT_DIAG] > ctx.wall_position_min) && (ctx.raw[WALL_SENSOR_LEFT_DIAG] < ctx.wall_position_max)) &&
	   ((ctx.raw[WALL_SENSOR_RIGHT_DIAG] > ctx.wall_position_min) && (ctx.raw[WALL_SENSOR_RIGHT_DIAG] < ctx.wall_position_max))	   )
	{
		//there is at least one wall
		return true;
	}
	else
	{
		//there is no wall
		return false;
	}
}

bool wall_sensor_wall_presence()
{
	if(((ctx.raw[WALL_SENSOR_LEFT_DIAG] > ctx.wall_position_min) && (ctx.raw[WALL_SENSOR_LEFT_DIAG] < ctx.wall_position_max)) ||
	   ((ctx.raw[WALL_SENSOR_RIGHT_DIAG] > ctx.wall_position_min) && (ctx.raw[WALL_SENSOR_RIGHT_DIAG] < ctx.wall_position_max))	   )
	{
		//there is at least one wall
		return true;
	}
	else
	{
		//there is no wall
		return false;
	}
}


bool wall_sensor_left_front_presence()
{
	if(controller_get_distance_led(wall_sensor_get(WALL_SENSOR_LEFT_STRAIGHT)) < WALL_FRONT_DISTANCE)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void wall_sensor_set_wall_position_min(int32_t pos_min){
	ctx.wall_position_min = pos_min;
}

void wall_sensor_set_wall_position_max(int32_t pos_max){
	ctx.wall_position_max = pos_max;
}
