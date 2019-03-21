/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_WALLSENSOR_H
#define __HAL_WALLSENSOR_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f7xx_hal.h"

/* HAL Public Data ------------------------------------------------------------------*/

enum WALL_SENSOR_ID
{
    WALL_SENSOR_LEFT_DIAG = 0,
	WALL_SENSOR_LEFT_STRAIGHT = 1,
    WALL_SENSOR_RIGHT_STRAIGHT = 2,
    WALL_SENSOR_RIGHT_DIAG = 3,
    WALL_SENSOR_COUNT
};

/* HAL Functions ------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

void HAL_WallSensor_Init(void);

void HAL_WallSensor_Process(void);

int32_t HAL_WallSensor_Get(int id);
int32_t HAL_WallSensor_Get_Raw(int id);

#ifdef __cplusplus
}
#endif

#endif /* __APP_WALLSENSOR_H */


