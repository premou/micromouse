/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_WALLSENSOR_H
#define __HAL_WALLSENSOR_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f7xx_hal.h"
#include <stdbool.h>

/* HAL Public Data ------------------------------------------------------------------*/

enum WALL_SENSOR_ID
{
	WALL_SENSOR_LEFT_DIAG= 0,
	WALL_SENSOR_LEFT_STRAIGHT = 1,
    WALL_SENSOR_RIGHT_STRAIGHT = 2,
	WALL_SENSOR_RIGHT_DIAG = 3,
    WALL_SENSOR_COUNT
};

/* HAL Functions ------------------------------------------------------------------*/

// init wall sensors
void wall_sensor_init();

// update all 4 wall sensors
void wall_sensor_update();

// get one wall sensor raw value (12-bit ADC measure)
int32_t wall_sensor_get(uint32_t sensor_id);
int32_t wall_sensor_get_side_error();
bool wall_sensor_wall_presence();
bool wall_sensor_wall_right_presence();
bool wall_sensor_wall_left_presence();
bool wall_sensor_left_front_presence();
bool wall_sensor_both_wall_presence();

#endif /* __APP_WALLSENSOR_H */


