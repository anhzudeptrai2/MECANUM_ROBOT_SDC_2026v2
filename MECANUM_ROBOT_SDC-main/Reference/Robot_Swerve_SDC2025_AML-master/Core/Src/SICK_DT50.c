#include "SICK_DT50.h"
#include "stm32h7xx_hal_adc.h"

SickDT50_HandleTypeDef sick_dt50;

static float LowPass_Filter(float prevFiltered, uint32_t newValue, float alpha)
{
    return alpha * (float)newValue + (1.0f - alpha) * prevFiltered;
}

static void Average_ADC_Filter(SickDT50_HandleTypeDef *sick, uint32_t rawValue0, uint32_t rawValue1)
{
    sick->avgBuffer[sick->sampleCounter][0] = rawValue0;
    sick->avgBuffer[sick->sampleCounter][1] = rawValue1;

    sick->sum[0] += rawValue0;
    sick->sum[1] += rawValue1;

    sick->sampleCounter++;

    if (sick->sampleCounter >= ADC_SAMPLE_NUMS)
    {
        uint32_t avg0 = sick->sum[0] / ADC_SAMPLE_NUMS;
        uint32_t avg1 = sick->sum[1] / ADC_SAMPLE_NUMS;

        sick->conv[0] = ADC_2_Meter(avg0);
        sick->conv[1] = ADC_2_Meter(avg1);

        sick->sampleCounter = 0;
        sick->sum[0] = 0;
        sick->sum[1] = 0;
    }
}

/**
 * @brief  Khởi tạo cấu trúc SickDT50
 * @param  sick: con trỏ tới cấu trúc SickDT50_HandleTypeDef
 * @param  hadc: con trỏ tới cấu trúc ADC_HandleTypeDef
 * @param  channelX: kênh ADC cho trục X
 * @param  channelY: kênh ADC cho trục Y
 * @retval None
 */
void SickDT50_Init(SickDT50_HandleTypeDef *sick, ADC_HandleTypeDef *hadc, uint32_t channelX, uint32_t channelY)
{
    sick->hadc = hadc;
    sick->channelX = channelX;
    sick->channelY = channelY;

    sick->sampleCounter = 0;
    sick->sum[0] = 0;
    sick->sum[1] = 0;
    sick->lowPassFiltered[0] = 0;
    sick->lowPassFiltered[1] = 0;

    for (uint8_t i = 0; i < ADC_SAMPLE_NUMS; i++)
    {
        sick->avgBuffer[i][0] = 0;
        sick->avgBuffer[i][1] = 0;
    }
}

/**
 * @brief  Cập nhật dữ liệu ADC theo chế độ blocking (polling).
 *         Hàm sẽ đọc lần lượt kênh X và kênh Y, áp dụng low-pass filter và gọi hàm tính trung bình.
 * @param  sick: con trỏ tới cấu trúc SickDT50_HandleTypeDef
 * @retval None
 */
void SickDT50_Update(SickDT50_HandleTypeDef *sick)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t adcValue = 0;
    HAL_StatusTypeDef status;

    // --- Đọc kênh X ---
    sConfig.Channel = sick->channelX;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5; // Chọn thời gian lấy mẫu phù hợp
    HAL_ADC_ConfigChannel(sick->hadc, &sConfig);

    HAL_ADC_Start(sick->hadc);
    status = HAL_ADC_PollForConversion(sick->hadc, 10); // Timeout 10ms
    if (status == HAL_OK)
    {
        adcValue = HAL_ADC_GetValue(sick->hadc);
        sick->lowPassFiltered[0] = LowPass_Filter(sick->lowPassFiltered[0], adcValue, 0.1f);
    }
    HAL_ADC_Stop(sick->hadc);

    // --- Đọc kênh Y ---
    sConfig.Channel = sick->channelY;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(sick->hadc, &sConfig);

    HAL_ADC_Start(sick->hadc);
    status = HAL_ADC_PollForConversion(sick->hadc, 10); // Timeout 10ms
    if (status == HAL_OK)
    {
        adcValue = HAL_ADC_GetValue(sick->hadc);
        sick->lowPassFiltered[1] = LowPass_Filter(sick->lowPassFiltered[1], adcValue, 0.1f);
    }
    HAL_ADC_Stop(sick->hadc);

    // --- Cập nhật bộ lọc trung bình cho cả 2 kênh ---
    uint32_t rawValue0 = (uint32_t)sick->lowPassFiltered[0];
    uint32_t rawValue1 = (uint32_t)sick->lowPassFiltered[1];
    Average_ADC_Filter(sick, rawValue0, rawValue1);
}

uint16_t SickDT50_GetDistanceX(SickDT50_HandleTypeDef *sick)
{
    return sick->conv[0];
}

uint16_t SickDT50_GetDistanceY(SickDT50_HandleTypeDef *sick)
{
    return sick->conv[1];
}

uint16_t ADC_2_Meter(uint32_t adcValue)
{
    if (adcValue <= ADC_MIN_VAL)
    {
        return MIN_DISTANCE;
    }

    if (adcValue >= (ADC_RESOLUTION - 1))
    {
        return MAX_DISTANCE;
    }

    return (uint16_t)(MIN_DISTANCE + ((adcValue - ADC_MIN_VAL) * (MAX_DISTANCE - MIN_DISTANCE)) / (ADC_RESOLUTION - ADC_MIN_VAL));
}
