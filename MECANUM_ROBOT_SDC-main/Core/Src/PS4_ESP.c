#include "PS4_ESP.h"
#include <string.h>

#include "stm32h7xx_hal.h" //thay bang f4 neu dung main f4

UART_HandleTypeDef *huart_ps4;
// PS4 data instances
PS4_DATA PS4_Dat;
BS Button_State = BUTTON_NONE;
volatile uint32_t PS4_LastRxTick = 0;

uint8_t rx_buffer[sizeof(PS4_Dat)];
static const uint8_t Req_Char = 'p';

void PS4_Init(UART_HandleTypeDef *uart_in)
{
    huart_ps4 = uart_in;

    /* Enable UART IDLE interrupt */
    __HAL_UART_ENABLE_IT(huart_ps4, UART_IT_IDLE);
}

void Extract_Button(uint16_t button_val, BS *extracted_button)
{
    *extracted_button = (BS)button_val;
}

void PS4_UART_Req(void)
{
    HAL_UART_Transmit_IT(huart_ps4, &Req_Char, 1);
    HAL_UART_Receive_DMA(huart_ps4, rx_buffer, sizeof(PS4_Dat));
}

void PS4_UART_Rx_IDLE_Handle(void)
{
    if (rx_buffer[8] == 0xff)
    {
        memcpy(&PS4_Dat, rx_buffer, sizeof(PS4_Dat));
        memset(rx_buffer, 0, sizeof(PS4_Dat));
        Extract_Button(PS4_Dat.button, &Button_State);
        PS4_LastRxTick = HAL_GetTick();
    }
    else
    {
        memset(rx_buffer, 0, sizeof(PS4_Dat));
        HAL_UART_Receive_DMA(huart_ps4, rx_buffer, sizeof(PS4_Dat));
    }
}
