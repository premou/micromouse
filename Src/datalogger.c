/* Includes -----------------------------------------------------------------*/
#include "datalogger.h"
#include "serial.h"

#include <stdarg.h>

/* HAL settings -------------------------------------------------------------*/

extern HAL_Serial_Handler com;

/* HAL Private data ---------------------------------------------------------*/

#define DATALOGGER_SIZE 360000
#define DATALOGGER_DATA_PER_ROW_MAX 24

static uint8_t data[DATALOGGER_SIZE];
static uint8_t * _current_data = data;
static uint8_t * const _data_end = data+DATALOGGER_SIZE;
static uint8_t _data_per_row = 0;
static uint8_t _data_size_per_row[DATALOGGER_DATA_PER_ROW_MAX];

/* HAL FunctionS    ---------------------------------------------------------*/


void HAL_DataLogger_Init(uint8_t size, ...)
{
    for( uint32_t index = 0; index<DATALOGGER_SIZE; ++index )
            data[index]=0;
    for( uint32_t index = 0; index<DATALOGGER_DATA_PER_ROW_MAX; ++index )
            _data_size_per_row[index]=0;
    _current_data = data;
    _data_per_row = size;
    va_list vl;
    va_start(vl,size);
    for( uint8_t index = 0; index<size; ++index )
    {
        _data_size_per_row[index]= (uint8_t)va_arg(vl,uint32_t);
    }
    va_end(vl);
}

void HAL_DataLogger_Clear(void)
{
    _current_data = data;
}

void HAL_DataLogger_Record(uint8_t size, ...)
{
    va_list vl;
    va_start(vl,size);
    // for each data param
    for( uint8_t index = 0; index<size; ++index )
    {
        uint8_t tmp_size = _data_size_per_row[index];
        // prevent buffer overflow
        if(_current_data+tmp_size<_data_end)
        {
            uint32_t tmp = va_arg(vl,uint32_t);

            do
            {
                *_current_data = (uint8_t)( tmp & 0x000000FF );
                ++_current_data;
                tmp = ( tmp >> 8 ) ;
                --tmp_size;
            }
            while (tmp_size>0);
        }
    }
    va_end(vl);
}

void HAL_DataLogger_Send(void)
{
    uint8_t * _ptr = data;
    uint32_t row_index = 0;
    uint32_t data_index = 0;
    //HAL_Serial_Print(&serial, "\n");
    while(_ptr<_current_data)
    {
        if(data_index==0)
        	HAL_Serial_Print(&com, "\nLOG %d ", row_index);
        uint8_t tmp_size = _data_size_per_row[data_index];
        uint32_t tmp = 0;
        int32_t tmp_index = 0;
        while(tmp_index<tmp_size)
        {
            tmp += (((uint32_t)(*_ptr)) << (tmp_index*8));
            ++_ptr;
            ++tmp_index;
        }
        if(tmp_size==4)
        	HAL_Serial_Print(&com, "%d ",(int32_t)tmp);
        else if(tmp_size==2)
        	HAL_Serial_Print(&com, "%d ",(int16_t)tmp);
        else
        	HAL_Serial_Print(&com, "%d ",(int8_t)tmp);
        ++data_index;
        // next row
        if(data_index==_data_per_row)
        {
            data_index = 0;
            ++row_index;
            //HAL_Serial_Print(&serial, "\n");
            HAL_Delay(5);
        }
    }
    HAL_Serial_Print(&com, "\nEOL\n");
}


