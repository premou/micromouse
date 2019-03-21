/* Includes ------------------------------------------------------------------*/
#include "WallSensor.h"
#include "WallSensorCalibration.h"

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

/* Private data --------------------------------------------------------------*/

extern TIM_HandleTypeDef htim6;
extern ADC_HandleTypeDef hadc1;

static TIM_HandleTypeDef * phtim = &htim6;
ADC_HandleTypeDef * phadc = &hadc1;

static __IO uint16_t ADC_Converted_Values[WALL_SENSOR_COUNT] = {0,0,0,0};
static __IO uint32_t offset[WALL_SENSOR_COUNT] = {0,0,0,0};
static __IO uint32_t distance[WALL_SENSOR_COUNT] = {0,0,0,0};

void elapse_micros(int16_t delayus)
{
    uint16_t start_time = phtim->Instance->CNT;
	while((int16_t)((uint16_t)phtim->Instance->CNT-start_time)<delayus);
}

uint32_t read_adc(ADC_HandleTypeDef * phadc, int id)
{
    ADC_ChannelConfTypeDef sConfig;
    sConfig.Channel = id_to_channel[id];
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES; // = 18µs = 15 + 480 cycles ADC à 108MHz/4
    HAL_ADC_ConfigChannel(phadc,&sConfig);
    HAL_ADC_Start(phadc);
    while(HAL_ADC_PollForConversion(phadc,1)==HAL_BUSY);
    uint32_t value = HAL_ADC_GetValue(phadc);
    //HAL_ADC_Stop(phadc);
    return value;
}

void all_led_off(void)
{
    HAL_GPIO_WritePin(id_to_port[WALL_SENSOR_RIGHT_STRAIGHT],id_to_pin[WALL_SENSOR_RIGHT_STRAIGHT],GPIO_PIN_RESET);
    HAL_GPIO_WritePin(id_to_port[WALL_SENSOR_LEFT_DIAG],id_to_pin[WALL_SENSOR_LEFT_DIAG],GPIO_PIN_RESET);
    HAL_GPIO_WritePin(id_to_port[WALL_SENSOR_RIGHT_DIAG],id_to_pin[WALL_SENSOR_RIGHT_DIAG],GPIO_PIN_RESET);
    HAL_GPIO_WritePin(id_to_port[WALL_SENSOR_LEFT_STRAIGHT],id_to_pin[WALL_SENSOR_LEFT_STRAIGHT],GPIO_PIN_RESET);
}

void acquire_one(int id)
{
        // led on
        HAL_GPIO_WritePin(id_to_port[id],id_to_pin[id],GPIO_PIN_SET);
        // pause
        elapse_micros(80); //80 (tempo) + 20 (ADC) = 100µs
        // acquire
        uint32_t value = read_adc(phadc, id);
        if(value>=offset[id])
            distance[id]=value-offset[id];
        else
            distance[id]=value;
        // led off
        all_led_off();
}

//void acquire_two(int id1, int id2)
//{
//        // led on
//        HAL_GPIO_WritePin(id_to_port[id1],id_to_pin[id1],GPIO_PIN_SET);
//        HAL_GPIO_WritePin(id_to_port[id2],id_to_pin[id2],GPIO_PIN_SET);
//        // pause
//        elapse_micros(100);
//        // acquire
//        uint32_t value[2] = { read_adc(phadc, id1), read_adc(phadc, id2) };
//        if(value[0]>=offset[id1])
//            distance[id1]=value[0]-offset[id1];
//        else
//            distance[id1]=value[0];
//        if(value[1]>=offset[id2])
//            distance[id2]=value[1]-offset[id2];
//        else
//            distance[id2]=value[1];
//        // led off
//        all_led_off();
//}

/* HAL functions ---------------------------------------------------------*/

void HAL_WallSensor_Init(void)
{
    //HAL_TIM_Base_Start(phtim);
    //__HAL_TIM_ENABLE_IT(phtim, TIM_IT_UPDATE);
	all_led_off();
}

void HAL_WallSensor_Process(void)
{
    // ambiant

    // led off
    //all_led_off();
    // pause
    //elapse_micros(100);
    // acquire
    offset[WALL_SENSOR_RIGHT_STRAIGHT] = read_adc(phadc, WALL_SENSOR_RIGHT_STRAIGHT);
    offset[WALL_SENSOR_LEFT_DIAG] = read_adc(phadc, WALL_SENSOR_LEFT_DIAG);
    offset[WALL_SENSOR_RIGHT_DIAG] = read_adc(phadc, WALL_SENSOR_RIGHT_DIAG);
    offset[WALL_SENSOR_LEFT_STRAIGHT] = read_adc(phadc, WALL_SENSOR_LEFT_STRAIGHT);
    // FL
    acquire_one(WALL_SENSOR_RIGHT_STRAIGHT);
    // FL
    acquire_one(WALL_SENSOR_LEFT_STRAIGHT);
    // DR
    acquire_one(WALL_SENSOR_RIGHT_DIAG);
    // DL
    acquire_one(WALL_SENSOR_LEFT_DIAG);
    // DR+DL
    //acquire_two(WALL_SENSOR_LEFT_DIAG,WALL_SENSOR_RIGHT_DIAG);

}

int32_t HAL_WallSensor_Get(int id)
{
    return wall_sensor_cal[id][distance[id]/4];
}

int32_t HAL_WallSensor_Get_Raw(int id)
{
    return distance[id];
}
