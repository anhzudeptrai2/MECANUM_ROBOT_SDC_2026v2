#include "DRIVER_PID_AML.h"

#include <stdint.h>
#include <string.h>

/* Some editor/IntelliSense setups for embedded projects don't have a valid C
 * standard library include path, which makes <stdint.h> effectively missing.
 * Provide safe fallbacks for types only in that case.
 */
#ifndef __uint8_t_defined
typedef unsigned char uint8_t;
#define __uint8_t_defined 1
#endif

#ifndef __int16_t_defined
typedef signed short int16_t;
#define __int16_t_defined 1
#endif

#ifndef __uint16_t_defined
typedef unsigned short uint16_t;
#define __uint16_t_defined 1
#endif

#ifndef __uint32_t_defined
typedef unsigned int uint32_t;
#define __uint32_t_defined 1
#endif

#include "stm32h7xx_hal_uart.h"

#define Low_Byte(w) ((uint8_t)((uint16_t)(w) & 0xffu))
#define High_Byte(w) ((uint8_t)(((uint16_t)(w) >> 8) & 0xffu))

uint8_t buffer[4];

volatile uint32_t AML_TxBusy = 0;
volatile uint32_t AML_TxDmaOk = 0;
volatile uint32_t AML_TxDmaErr = 0;
volatile uint32_t AML_TxBlockingOk = 0;

volatile uint8_t AML_ForceBlocking = 0;

volatile uint8_t AML_LastTx[40] = {0};
volatile uint16_t AML_LastTxLen = 0;
volatile uint32_t AML_LastTxW[10] = {0};

static void AML_PackLastTxWords(void)
{
    /* Pack 4 bytes per word for easier Watch view in Keil.
     * Word 0 contains AML_LastTx[0..3] as 0xB3B2B1B0.
     */
    const uint16_t len = AML_LastTxLen;
    const uint16_t words = (uint16_t)((len + 3u) / 4u);
    const uint16_t max_words = (uint16_t)(sizeof(AML_LastTxW) / sizeof(AML_LastTxW[0]));
    const uint16_t n = (words < max_words) ? words : max_words;

    for (uint16_t i = 0; i < n; i++)
    {
        const uint16_t base = (uint16_t)(i * 4u);
        uint32_t w = 0;
        if (base + 0u < len)
            w |= ((uint32_t)AML_LastTx[base + 0u]) << 0;
        if (base + 1u < len)
            w |= ((uint32_t)AML_LastTx[base + 1u]) << 8;
        if (base + 2u < len)
            w |= ((uint32_t)AML_LastTx[base + 2u]) << 16;
        if (base + 3u < len)
            w |= ((uint32_t)AML_LastTx[base + 3u]) << 24;
        AML_LastTxW[i] = w;
    }
    for (uint16_t i = n; i < max_words; i++)
    {
        AML_LastTxW[i] = 0;
    }
}

void Driver_PID_AML_Init_UART(UART_HandleTypeDef *uart_par, Motor_Driver *motor)
{
    motor->Driver_UART = uart_par;
}

void Assign_PID_AML_Id(Motor_Driver *motor, uint8_t id)
{
    motor->ID = id;
}

void Driver_Home_Request(Motor_Driver motor)
{
    buffer[0] = motor.ID;
    buffer[1] = 0xff;
    buffer[2] = 0xff;
    buffer[3] = 0xf3;

    HAL_UART_Transmit(motor.Driver_UART, (uint8_t *)buffer, 4, 200);
}

void Driver_Set_Zero_Position(Motor_Driver motor)
{
    buffer[0] = motor.ID;
    buffer[1] = 0xff;
    buffer[2] = 0xff;
    buffer[3] = 0xf2;

    HAL_UART_Transmit(motor.Driver_UART, (uint8_t *)buffer, 4, 200);
}

void Driver_Send_Setpoints_U1(Motor_Driver *motors, uint8_t num_motors)
{
    static uint8_t buffer_tx_setpoints_u1[40];
    memset(buffer_tx_setpoints_u1, 0, sizeof(buffer_tx_setpoints_u1));

    for (uint8_t i = 0; i < num_motors; i++)
    {
        buffer_tx_setpoints_u1[i * 4] = motors[i].ID;
        buffer_tx_setpoints_u1[i * 4 + 1] = Low_Byte(motors[i].Set_Point);
        buffer_tx_setpoints_u1[i * 4 + 2] = High_Byte(motors[i].Set_Point);
        buffer_tx_setpoints_u1[i * 4 + 3] = 0xFF;
    }

    AML_LastTxLen = (uint16_t)(4u * num_motors);
    if (AML_LastTxLen > sizeof(AML_LastTx))
        AML_LastTxLen = sizeof(AML_LastTx);
    memcpy((void *)AML_LastTx, (void *)buffer_tx_setpoints_u1, AML_LastTxLen);
    AML_PackLastTxWords();

    HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(motors->Driver_UART, buffer_tx_setpoints_u1, 4 * num_motors);
    if (st == HAL_OK)
    {
        AML_TxDmaOk++;
    }
    else
    {
        AML_TxDmaErr++;
    }
}

void Driver_Send_Setpoints_U2(Motor_Driver *motors, uint8_t num_motors)
{
    static uint8_t buffer_tx_setpoints_u2[40];
    memset(buffer_tx_setpoints_u2, 0, sizeof(buffer_tx_setpoints_u2));

    for (uint8_t i = 0; i < num_motors; i++)
    {
        buffer_tx_setpoints_u2[i * 4] = motors[i].ID;
        buffer_tx_setpoints_u2[i * 4 + 1] = Low_Byte(motors[i].Set_Point);
        buffer_tx_setpoints_u2[i * 4 + 2] = High_Byte(motors[i].Set_Point);
        buffer_tx_setpoints_u2[i * 4 + 3] = 0xFF;
    }

    AML_LastTxLen = (uint16_t)(4u * num_motors);
    if (AML_LastTxLen > sizeof(AML_LastTx))
        AML_LastTxLen = sizeof(AML_LastTx);
    memcpy((void *)AML_LastTx, (void *)buffer_tx_setpoints_u2, AML_LastTxLen);
    AML_PackLastTxWords();

    HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(motors->Driver_UART, buffer_tx_setpoints_u2, 4 * num_motors);
    if (st == HAL_OK)
    {
        AML_TxDmaOk++;
    }
    else
    {
        AML_TxDmaErr++;
    }
}
