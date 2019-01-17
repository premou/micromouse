/*
 * serial.h
 *
 *  Created on: 29 nov. 2015
 *      Author: Patrick
 */

#ifndef HAL_SERIAL_H_
#define HAL_SERIAL_H_

#include "stm32f7xx_hal.h"

/* HAL Public Data ------------------------------------------------------------------*/
/*
#define size_of_tx_pool 16
#define size_of_tx_buffer 1600
#define size_of_rx_circular_buffer 16000
*/

#define size_of_tx_pool 32
#define size_of_tx_buffer 1600
#define size_of_rx_circular_buffer 8000

/*
#define size_of_tx_pool 32
#define size_of_tx_buffer 1600
#define size_of_rx_circular_buffer 32000
*/

typedef struct
{
	uint8_t data[size_of_tx_buffer];
	uint32_t length;
} tx_buffer;

typedef struct
{
    UART_HandleTypeDef * huart;

    tx_buffer tx_buffer_pool[size_of_tx_pool];
    uint32_t tx_head_position;
    uint32_t tx_tail_position;
    volatile uint32_t tx_dma;

    uint8_t rx_circular_buffer[size_of_rx_circular_buffer];
    uint8_t const * rx_tail_ptr;

} HAL_Serial_Handler;

#define HAL_Serial_Handler_Count 2

/* HAL Functions ------------------------------------------------------------------*/

void HAL_Serial_Init(UART_HandleTypeDef * huart, HAL_Serial_Handler * hserial);
int HAL_Serial_Available(HAL_Serial_Handler * hserial);
char HAL_Serial_GetChar(HAL_Serial_Handler * hserial);
int HAL_Serial_Read(HAL_Serial_Handler * hserial, uint8_t * ptr, int len );
int HAL_Serial_Write(HAL_Serial_Handler * hserial, uint8_t const * ptr, int len );
int HAL_Serial_Print(HAL_Serial_Handler * hserial,const char *fmt, ...);

#endif /* HAL_SERIAL_H_ */
