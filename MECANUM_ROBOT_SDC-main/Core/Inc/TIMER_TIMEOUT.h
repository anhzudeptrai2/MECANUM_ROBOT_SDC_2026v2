#ifndef TIMER_TIMEOUT_H
#define TIMER_TIMEOUT_H

#include "stdint.h"

/* HAL timer types (TIM_HandleTypeDef, ...) */
#include "main.h"

#define TASK_NUMS 5 // Extend number of task for larger number task
static const uint16_t Task_Timeout_ms[TASK_NUMS] = {100, 200, 300, 800, 1000}; // Timeout for each task
typedef struct
{
    volatile uint8_t Time_Out_Flag; // Be set if timeout occur
    uint16_t Time_Out_Set;
    volatile uint16_t Task_Time; // ms unit
} Task_Timeout;

extern Task_Timeout Task_TO[TASK_NUMS]; // declare this struct in main.c

void Timer_Timeout_Init(Task_Timeout *t_to, uint16_t timeout_ms);
void Timer_Timeout_Start(TIM_HandleTypeDef *htim);
/*Put this function after a completed task to reset timeout timer*/
void Timer_Timeout_Reset(Task_Timeout *t_to);
/*Put this function in 1ms timer interrupt handle to update all available timeout*/
void Timer_Timeout_Check(Task_Timeout *t_to);
void Timeout_Begin(void);

#endif // TIMER_TIMEOUT_H
