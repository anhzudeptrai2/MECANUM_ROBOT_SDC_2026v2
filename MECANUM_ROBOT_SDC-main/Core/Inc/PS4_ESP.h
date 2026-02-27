#ifndef PS4_ESP_H
#define PS4_ESP_H

#include <stdint.h>

#include "main.h"

#pragma pack(push, 1)
typedef struct
{
    uint16_t button;
    uint8_t l2_analog;
    uint8_t r2_analog;
    int8_t l_stick_x;
    int8_t l_stick_y;
    int8_t r_stick_x;
    int8_t r_stick_y;
    uint8_t sync;
} PS4_DATA;
#pragma pack(pop)

extern PS4_DATA PS4_Dat;
extern volatile uint32_t PS4_LastRxTick;

typedef volatile enum {
    BUTTON_NONE = 0,
    BUTTON_RIGHT = 1 << 0,
    BUTTON_LEFT = 1 << 1,
    BUTTON_UP = 1 << 2,
    BUTTON_DOWN = 1 << 3,
    BUTTON_CROSS = 1 << 4,
    BUTTON_CIRCLE = 1 << 5,
    BUTTON_SQUARE = 1 << 6,
    BUTTON_TRIANGLE = 1 << 7,
    BUTTON_L1 = 1 << 8,
    BUTTON_R1 = 1 << 9,
    BUTTON_SHARE = 1 << 10,
    BUTTON_OPTIONS = 1 << 11,
    BUTTON_L3 = 1 << 12,
    BUTTON_R3 = 1 << 13,
    BUTTON_PSBUTTON = 1 << 14,
    BUTTON_TOUCHPAD = 1 << 15
} BS;

extern BS Button_State;

void PS4_Init(UART_HandleTypeDef *uart_in);
void PS4_UART_Req(void);
void PS4_UART_Rx_IDLE_Handle(void);

#endif
