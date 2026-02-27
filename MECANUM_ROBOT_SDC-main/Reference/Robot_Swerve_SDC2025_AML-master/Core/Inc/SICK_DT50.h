#ifndef SICK_DT50_H
#define SICK_DT50_H

#include "stdint.h"

#ifdef STM32H7
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_adc.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
#endif

#define ADC_SAMPLE_NUMS 5
#define MIN_DISTANCE 200   // mm
#define MAX_DISTANCE 5000  // mm
#define ADC_RESOLUTION 4096 // 12-bit ADC
#define ADC_MIN_VAL 0

typedef struct
{
    ADC_HandleTypeDef *hadc;
    uint32_t channelX;
    uint32_t channelY;

    uint32_t adcDMA_Buffer[2]; 
    float lowPassFiltered[2];  

    uint32_t avgBuffer[ADC_SAMPLE_NUMS][2]; 
    uint32_t sum[2];                        
    uint8_t sampleCounter;              

    uint16_t conv[2]; 
} SickDT50_HandleTypeDef;

extern SickDT50_HandleTypeDef sick_dt50;

void SickDT50_Init(SickDT50_HandleTypeDef *sick, ADC_HandleTypeDef *hadc, uint32_t channelX, uint32_t channelY);
uint16_t SickDT50_GetDistanceX(SickDT50_HandleTypeDef *sick);
uint16_t SickDT50_GetDistanceY(SickDT50_HandleTypeDef *sick);
uint16_t ADC_2_Meter(uint32_t adcValue);
void SickDT50_Update(SickDT50_HandleTypeDef *sick);


#endif // SICK_DT50_H
