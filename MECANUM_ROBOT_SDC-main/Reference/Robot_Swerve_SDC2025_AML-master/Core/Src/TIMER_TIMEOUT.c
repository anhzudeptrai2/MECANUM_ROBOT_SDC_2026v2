#include "TIMER_TIMEOUT.h"
#include "stm32h7xx_hal.h"


void Timer_Timeout_Init(Task_Timeout *t_to, uint16_t timeout_ms)
{
    t_to->Time_Out_Set = timeout_ms;
    t_to->Task_Time = 0;
    t_to->Time_Out_Flag = 0;
}

void Timer_Timeout_Start(TIM_HandleTypeDef *htim)
{
    HAL_TIM_Base_Start_IT(htim);
}

void Timer_Timeout_Reset(Task_Timeout *t_to)
{
    t_to->Task_Time = 0;
    t_to->Time_Out_Flag = 0;
}

void Timer_Timeout_Check(Task_Timeout *t_to)
{
    for (int i = 0; i < TASK_NUMS; i++)
    {
        if (t_to[i].Time_Out_Flag == 0)
        {
            t_to[i].Task_Time++;
            if (t_to[i].Task_Time >= t_to[i].Time_Out_Set)
            {
                t_to[i].Time_Out_Flag = 1;
            }
        }
    }
}

void Timeout_Begin(void)
{
    for (uint8_t i = 0; i < TASK_NUMS; i++)
    {
        Timer_Timeout_Init(&Task_TO[i], Task_Timeout_ms[i]);
        Timer_Timeout_Reset(&Task_TO[i]);
    }
}
