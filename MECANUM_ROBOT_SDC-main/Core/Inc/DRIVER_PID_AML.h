#ifndef DRIVER_PID_AML_H
#define DRIVER_PID_AML_H

#include <stdint.h>
#include <stdlib.h>

/* Pull in HAL types (UART_HandleTypeDef, HAL_StatusTypeDef, ...) via CubeMX main.h.
 * Do not rely on custom STM32H7/STM32F4 macros which may not be defined in Keil.
 */
#include "main.h"

#define MAX_UARTS 4

typedef struct
{
    uint8_t ID;
    volatile int16_t Set_Point;
    UART_HandleTypeDef *Driver_UART;
} Motor_Driver;

/* Debug counters / snapshots (optional, for Keil watch) */
extern volatile uint32_t AML_TxBusy;
extern volatile uint32_t AML_TxDmaOk;
extern volatile uint32_t AML_TxDmaErr;
extern volatile uint32_t AML_TxBlockingOk;
extern volatile uint8_t AML_LastTx[40];
extern volatile uint16_t AML_LastTxLen;
extern volatile uint32_t AML_LastTxW[10];
extern volatile uint8_t AML_ForceBlocking;

void Driver_PID_AML_Init_UART(UART_HandleTypeDef *uart_par, Motor_Driver *motor);
void Assign_PID_AML_Id(Motor_Driver *motor, uint8_t id);
void Driver_Home_Request(Motor_Driver motor);
void Driver_Set_Zero_Position(Motor_Driver motor);
void Driver_Send_Setpoints_U1(Motor_Driver *motors, uint8_t num_motors);
void Driver_Send_Setpoints_U2(Motor_Driver *motors, uint8_t num_motors);
#endif // DRIVER_PID_AML_H
