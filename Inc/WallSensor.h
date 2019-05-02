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

// accessers
int32_t wall_sensor_get_raw(uint32_t sensor_id);
float wall_sensor_get_dist(uint32_t sensor_id);

// new API
bool wall_sensor_is_left_wall_detected();
bool wall_sensor_is_front_wall_detected();
bool wall_sensor_is_right_wall_detected();

// old API

// get one wall sensor raw value (12-bit ADC measure)
int32_t wall_sensor_get(uint32_t sensor_id);
int32_t wall_sensor_get_side_error(); // wall following in controller
int32_t wall_sensor_get_straight_adc(); // used bu U turn PID front wall => TBD , replaced by get_dist in controller
int32_t wall_sensor_get_straight_diff_adc(); // used bu U turn PID front wall => TBD , replaced by get_dist in controller

bool wall_sensor_wall_presence();
bool wall_sensor_wall_right_presence();
bool wall_sensor_wall_left_presence();
bool wall_sensor_left_front_presence();
bool wall_sensor_both_wall_presence(); // used by wall_sensor_get_side_error

//void wall_sensor_set_wall_position_min(int32_t pos_min);
//void wall_sensor_set_wall_position_max(int32_t pos_max);

#endif /* __APP_WALLSENSOR_H */


