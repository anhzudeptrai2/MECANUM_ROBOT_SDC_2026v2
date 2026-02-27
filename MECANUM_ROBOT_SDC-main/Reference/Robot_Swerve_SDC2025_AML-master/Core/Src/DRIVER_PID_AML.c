#include "DRIVER_PID_AML.h"

#include "stm32h7xx_hal_uart.h"

#define Low_Byte(w) ((uint8_t)((w) & 0xff))
#define High_Byte(w) ((uint8_t)((w) >> 8))


uint8_t buffer[4];

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

    HAL_UART_Transmit_DMA(motors->Driver_UART, buffer_tx_setpoints_u1, 4 * num_motors);
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

    HAL_UART_Transmit_DMA(motors->Driver_UART, buffer_tx_setpoints_u2, 4 * num_motors);
}
