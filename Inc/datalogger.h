
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_DATALOGGER_H
#define __HAL_DATALOGGER_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f7xx_hal.h"

/* HAL Public Data ------------------------------------------------------------------*/

/* HAL Functions ------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

void HAL_DataLogger_Init(uint8_t size, ...);
void HAL_DataLogger_Clear(void);
void HAL_DataLogger_Record(uint8_t size, ...);
void HAL_DataLogger_Send(void);

#ifdef __cplusplus
}
#endif

/* IRS ------------------------------------------------------------------*/


#endif /* __APP_WALLMAZEDISCOVER_MODE_H */


