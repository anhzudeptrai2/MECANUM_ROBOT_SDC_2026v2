/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PS4_ESP.h"
#include "NEXTION_HMI.h"
#include "TIMER_TIMEOUT.h"
#include "PID.h"
#include "WT901C.h"
#include "DRIVER_PID_AML.h"
#include "SWERVE_X4_KINEMATIC.h"
#include "ROBOT_ACCELERATION.h"
#include "SICK_DT50.h"
#include "TACTILE.h"
#include "ENCODER.h"
#include "WAYPOINTS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_uart7_tx;
DMA_HandleTypeDef hdma_uart8_tx;
DMA_HandleTypeDef hdma_uart8_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
/*----------------------------Tactile Variables----------------------------*/
uint8_t Robot_Side = 0; // 0:Blue, 1:Red

volatile uint8_t action_done_flag = 0;

volatile uint8_t mp_count = 0;

MotionProfile profile[5];
int profile_max_index = sizeof(profile) / sizeof(profile[0]);
// Bong 1
Waypoint A_W = {0.0f, 0.0f, 0.0f, 0.0f};
Waypoint B_W = {1084.0f, -1000.0f, 0.0f, 0.0f};
Waypoint C_W = {1084.0f, -600.0f, 0.0f, 0.0f};
Waypoint D_W = {100.0f, -4100.0f, 0.0f, 0.0f};

// Bong 2
Waypoint E_W = {1342.0f, -900.0f, 0.0f, 0.0f};
// Waypoint F_W = {1372.0f, -2686.0f, 0.0f, 0.0f};
// Waypoint G_W = {1587.0f, -397.0f, 0.0f, 0.0f};

volatile float current_time = 0.0f;
volatile uint8_t Is_Moving_Waypoints;
/*--------------------------Arm_Gripper_Variables--------------------------*/
volatile int16_t Current_Arm_Position = 0;
volatile int16_t Target_Arm_Position = 0;

int16_t Home_Pulse = 0;
int16_t Stanby_Picking_Ball_Pulse = -400;
int16_t Home_Suck_Ball_Offset = -80;
int16_t Picking_Ball_Pulse = -1590;
int16_t High_Silo_Pulse = -1000;
int16_t Mid_Silo_Pulse = -350;
int16_t Low_Silo_Pulse = -100;

volatile uint8_t Is_Arm_Moving = 0;

#define ARM_IDLE 0
#define ARM_TO_PICKING 1
#define ARM_RETURNING_HOME 2

volatile uint8_t arm_action_state = ARM_IDLE;

/*------------------------Encoder_Measure_Variables------------------------*/
Encoder_HandleTypeDef Encoder_X_Local;
Encoder_HandleTypeDef Encoder_Y_Local;
volatile float Encoder_X_Loc = 0;
volatile float Encoder_Y_Loc = 0;
volatile float Distance_Encoder = 0;
/*------------------------------IMU Variables------------------------------*/
WT901C IMU;
PID_TypeDef Omega_PID, Vx_PID, Vy_PID;
uint8_t Robot_Reset_Home = 0;

/*---------------------------SICK DT50 Variables---------------------------*/

/*---------------------------PS4 Data Variables---------------------------*/
PS4_DATA PS4_Dat;
volatile BS Button_State = BUTTON_NONE;
volatile BS Last_Button = BUTTON_NONE;
typedef enum
{
  BUTTON_IDLE,
  BUTTON_PRESSED,
  BUTTON_RELEASED
} ButtonState_t;
volatile ButtonState_t Button_Hold_State = BUTTON_IDLE;

/*-------------------------Timeout Timer Variables-------------------------*/
Task_Timeout Task_TO[TASK_NUMS];
uint16_t Timer_OVF_Flag_0 = 0;
uint16_t Timer_OVF_Flag_1 = 0;
uint16_t Timer_OVF_Flag_2 = 0;

/*--------------------------Swerve robot variables--------------------------*/
SRb Swerve_Robot;

/*--------------------------Motor driver variables--------------------------*/
Motor_Driver Wheel_Motor[5], Steering_Motor[5];

/*-----------------Robot acceleration variables for auto & semi auto-----------------*/
RobotAcceleration Swerve_Acceleration;
uint8_t V_Robot_Target = 0;
float Accel_Time = 2; // Second

/*----------------------------------LED ENUM----------------------------------*/
typedef enum
{
  LED_A,
  LED_B,
  LED_RUN,
  LED_FAULT
} LED_TypeDef;

struct Bz
{
  uint8_t state;
  uint16_t duration;
  uint16_t stm_time;
} Buzzer;

/*--------------------------------HMI_Variables--------------------------------*/
Nextion nextion;

NexComp IMU_TEXT;

NexComp imu_val;
NexComp Encoder_X_val;
NexComp Encoder_Y_val;
NexComp Lazer_X_val;
NexComp Lazer_Y_val;
NexComp Side;

NexComp up_ps4_val;
NexComp down_ps4_val;
NexComp right_ps4_val;
NexComp left_ps4_val;
NexComp triag_ps4_val;
NexComp cross_ps4_val;
NexComp circle_ps4_val;
NexComp square_ps4_val;
NexComp r1_ps4_val;
NexComp l1_ps4_val;
NexComp r3_ps4_val;
NexComp l3_ps4_val;
NexComp opt_ps4_val;
NexComp share_ps4_val;
NexComp ps_ps4_val;
NexComp kp_ps4_val;

NexComp rx_bar;
NexComp ry_bar;
NexComp lx_bar;
NexComp ly_bar;
NexComp r2_bar;
NexComp l2_bar;

NexComp ps4_fault;
NexComp imu_fault;

NexComp test_ps4_button;
NexComp home_button;
NexComp robot_home_button;
NexComp change_side_button;
NexComp run_button;
NexComp stop_button;
typedef enum
{
  HOME_PAGE,
  PS4_PAGE
} PageState;
PageState Page_State = HOME_PAGE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_UART7_Init(void);
static void MX_UART8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*-------------------------------Function prototypes-------------------------------*/
void Robot_Hold_Stop(void);

/*----------------------------------HMI_Functions----------------------------------*/
void Test_PS4_Button_Handle(void) { Page_State = PS4_PAGE; }
void Home_Button_Handle(void) { Page_State = HOME_PAGE; }
void Home_Robot_Button_Handle(void) { Robot_Reset_Home = 1; }
void Run_Button_Handle(void)
{
}
void Stop_Button_Handle(void) {}
void Change_Side_Button_Handle(void)
{
  Swerve_Robot.Play_Field = (Swerve_Robot.Play_Field == 0) ? 1 : 0;
  Set_Buzzer_State(200);
  if (!Swerve_Robot.Play_Field)
  {
    A_W = (Waypoint){0.0f, 0.0f, 0.0f, 0.0f};
    B_W = (Waypoint){1084.0f, -1000.0f, 0.0f, 0.0f};
    C_W = (Waypoint){1084.0f, -600.0f, 0.0f, 0.0f};
    D_W = (Waypoint){100.0f, -4100.0f, 0.0f, 0.0f};

    // Bong 2
    E_W = (Waypoint){1342.0f, -900.0f, 0.0f, 0.0f};
  }
  else
  {
    A_W = (Waypoint){0.0f, 0.0f, 0.0f, 0.0f};
    B_W = (Waypoint){-950.0f, -1000.0f, 0.0f, 0.0f};
    C_W = (Waypoint){-1000.0f, -600.0f, 0.0f, 0.0f};
    D_W = (Waypoint){150.0f, -4100.0f, 0.0f, 0.0f};

    // Bong 2
    E_W = (Waypoint){-1342.0f, -900.0f, 0.0f, 0.0f};
  }
  Motion_Profile_Init(&profile[0], A_W, B_W, 400, 800);
  Motion_Profile_Init(&profile[1], B_W, C_W, 200, 800);
  Motion_Profile_Init(&profile[2], C_W, D_W, 850, 1600);

  Motion_Profile_Init(&profile[3], D_W, E_W, 800, 1600);
}

void Init_Nextion_Comp(void)
{
  /////////////////////////////home-page/////////////////////////////
  NextionAddComp(&nextion, &imu_val, "n0", 0, 10, NULL, NULL);
  NextionAddComp(&nextion, &Encoder_X_val, "n1", 0, 11, NULL, NULL);
  NextionAddComp(&nextion, &Encoder_Y_val, "n2", 0, 12, NULL, NULL);
  NextionAddComp(&nextion, &Lazer_X_val, "n3", 0, 13, NULL, NULL);
  NextionAddComp(&nextion, &Lazer_Y_val, "n4", 0, 14, NULL, NULL);
  NextionAddComp(&nextion, &IMU_TEXT, "t0", 0, 2, NULL, NULL);
  NextionAddComp(&nextion, &ps4_fault, "va1", 0, 20, NULL, NULL);
  NextionAddComp(&nextion, &imu_fault, "va0", 0, 19, NULL, NULL);
  NextionAddComp(&nextion, &Side, "va2", 0, 25, NULL, NULL);

  NextionAddComp(&nextion, &robot_home_button, "rb_home", 0, 9, NULL, Home_Robot_Button_Handle);
  NextionAddComp(&nextion, &test_ps4_button, "ps4", 0, 16, NULL, Test_PS4_Button_Handle);
  NextionAddComp(&nextion, &change_side_button, "change_side", 0, 21, NULL, Change_Side_Button_Handle);
  NextionAddComp(&nextion, &run_button, "run", 0, 88, NULL, Run_Button_Handle);
  NextionAddComp(&nextion, &stop_button, "stop", 0, 80, NULL, Stop_Button_Handle);

  /////////////////////////////ps4_page//////////////////////////////
  NextionAddComp(&nextion, &up_ps4_val, "va0", 1, 33, NULL, NULL);
  NextionAddComp(&nextion, &down_ps4_val, "va1", 1, 34, NULL, NULL);
  NextionAddComp(&nextion, &right_ps4_val, "va2", 1, 35, NULL, NULL);
  NextionAddComp(&nextion, &left_ps4_val, "va3", 1, 36, NULL, NULL);
  NextionAddComp(&nextion, &triag_ps4_val, "va4", 1, 37, NULL, NULL);
  NextionAddComp(&nextion, &cross_ps4_val, "va5", 1, 38, NULL, NULL);
  NextionAddComp(&nextion, &circle_ps4_val, "va6", 1, 39, NULL, NULL);
  NextionAddComp(&nextion, &square_ps4_val, "va7", 1, 40, NULL, NULL);
  NextionAddComp(&nextion, &r1_ps4_val, "va8", 1, 41, NULL, NULL);
  NextionAddComp(&nextion, &l1_ps4_val, "va9", 1, 42, NULL, NULL);
  NextionAddComp(&nextion, &r3_ps4_val, "va10", 1, 43, NULL, NULL);
  NextionAddComp(&nextion, &l3_ps4_val, "va11", 1, 44, NULL, NULL);
  NextionAddComp(&nextion, &opt_ps4_val, "va12", 1, 45, NULL, NULL);
  NextionAddComp(&nextion, &share_ps4_val, "va13", 1, 46, NULL, NULL);
  NextionAddComp(&nextion, &ps_ps4_val, "va14", 1, 47, NULL, NULL);
  NextionAddComp(&nextion, &kp_ps4_val, "va15", 1, 48, NULL, NULL);

  NextionAddComp(&nextion, &rx_bar, "j0", 1, 4, NULL, NULL);
  NextionAddComp(&nextion, &ry_bar, "j1", 1, 5, NULL, NULL);
  NextionAddComp(&nextion, &lx_bar, "j2", 1, 6, NULL, NULL);
  NextionAddComp(&nextion, &ly_bar, "j3", 1, 7, NULL, NULL);
  NextionAddComp(&nextion, &r2_bar, "j4", 1, 8, NULL, NULL);
  NextionAddComp(&nextion, &l2_bar, "j5", 1, 9, NULL, NULL);

  NextionAddComp(&nextion, &home_button, "home", 1, 3, NULL, Home_Button_Handle);
}

void Home_Page_Nextion_Print(void)
{
  NextionSetVal(&nextion, &imu_val, IMU.Yaw);
  NextionSetVal(&nextion, &Encoder_X_val, (int32_t)Encoder_X_Loc);
  NextionSetVal(&nextion, &Encoder_Y_val, (int32_t)Encoder_Y_Loc);
  NextionSetVal(&nextion, &Lazer_X_val, (int32_t)Distance_Encoder);
  //  NextionSetVal(&nextion, &Lazer_Y_val, Distance_Lazer_Y);

  NextionSetVal(&nextion, &ps4_fault, Task_TO[0].Time_Out_Flag ? 0 : 1);
  NextionSetVal(&nextion, &imu_fault, Task_TO[1].Time_Out_Flag ? 0 : 1);
  NextionSetVal(&nextion, &Side, Swerve_Robot.Play_Field);
}
void PS4_Page_Nextion_Print(void)
{
  NextionSetVal(&nextion, &up_ps4_val, Button_State == BUTTON_UP ? 1 : 0);
  NextionSetVal(&nextion, &down_ps4_val, Button_State == BUTTON_DOWN ? 1 : 0);
  NextionSetVal(&nextion, &right_ps4_val, Button_State == BUTTON_RIGHT ? 1 : 0);
  NextionSetVal(&nextion, &left_ps4_val, Button_State == BUTTON_LEFT ? 1 : 0);
  NextionSetVal(&nextion, &triag_ps4_val, Button_State == BUTTON_TRIANGLE ? 1 : 0);
  NextionSetVal(&nextion, &cross_ps4_val, Button_State == BUTTON_CROSS ? 1 : 0);
  NextionSetVal(&nextion, &circle_ps4_val, Button_State == BUTTON_CIRCLE ? 1 : 0);
  NextionSetVal(&nextion, &square_ps4_val, Button_State == BUTTON_SQUARE ? 1 : 0);
  NextionSetVal(&nextion, &r1_ps4_val, Button_State == BUTTON_R1 ? 1 : 0);
  NextionSetVal(&nextion, &l1_ps4_val, Button_State == BUTTON_L1 ? 1 : 0);
  NextionSetVal(&nextion, &r3_ps4_val, Button_State == BUTTON_R3 ? 1 : 0);
  NextionSetVal(&nextion, &l3_ps4_val, Button_State == BUTTON_L3 ? 1 : 0);
  NextionSetVal(&nextion, &opt_ps4_val, Button_State == BUTTON_OPTIONS ? 1 : 0);
  NextionSetVal(&nextion, &share_ps4_val, Button_State == BUTTON_SHARE ? 1 : 0);
  NextionSetVal(&nextion, &ps_ps4_val, Button_State == BUTTON_PSBUTTON ? 1 : 0);
  NextionSetVal(&nextion, &kp_ps4_val, Button_State == BUTTON_TOUCHPAD ? 1 : 0);

  NextionSetVal(&nextion, &rx_bar, (uint8_t)((128 + PS4_Dat.r_stick_x) / 2.55f));
  NextionSetVal(&nextion, &ry_bar, (uint8_t)((128 + PS4_Dat.r_stick_y) / 2.55f));
  NextionSetVal(&nextion, &lx_bar, (uint8_t)((128 + PS4_Dat.l_stick_x) / 2.55f));
  NextionSetVal(&nextion, &ly_bar, (uint8_t)((128 + PS4_Dat.l_stick_y) / 2.55f));
  NextionSetVal(&nextion, &r2_bar, (uint8_t)((PS4_Dat.r2_analog) / 2.55f));
  NextionSetVal(&nextion, &l2_bar, (uint8_t)((PS4_Dat.l2_analog) / 2.55f));
}

void Update_Nextion_Page(void)
{
  switch (Page_State)
  {
  case HOME_PAGE:
    Home_Page_Nextion_Print();
    break;
  case PS4_PAGE:
    PS4_Page_Nextion_Print();
    break;
  default:
    break;
  }
}
/*----------------------------------LEDS & BUZZER----------------------------------*/
void Set_LED_State(LED_TypeDef led, GPIO_PinState state)
{
  switch (led)
  {
  case LED_A:
    HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, state);
    break;
  case LED_B:
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, state);
    break;
  case LED_RUN:
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, state);
    break;
  case LED_FAULT:
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, state);
    break;
  default:
    break;
  }
}

void Set_Buzzer_State(uint16_t duration)
{
  Buzzer.state = 1;
  Buzzer.duration = duration;
  Buzzer.stm_time = 0;
}
void Buzzer_Update(void)
{
  if (Buzzer.stm_time >= Buzzer.duration)
  {
    Buzzer.state = 0;
    Buzzer.stm_time = 0;
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
  }
  else
  {
    Buzzer.stm_time++;
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
  }
}

/*----------------------------------Arm Calculate----------------------------------*/
void Arm_Calculate(void)
{
  /*---------------------------------Arm_Pulse Calculate---------------------------------*/
  if (Is_Arm_Moving)
  {
    if (Current_Arm_Position > Target_Arm_Position)
    {
      Current_Arm_Position -= 1;
    }
    else if (Current_Arm_Position < Target_Arm_Position)
    {
      Current_Arm_Position += 1;
    }
    else
    {
      Is_Arm_Moving = 0;
    }
  }

  if (!Is_Arm_Moving && arm_action_state != ARM_IDLE)
  {
    if (arm_action_state == ARM_TO_PICKING)
    {
      HAL_GPIO_WritePin(OUTPUT_5_GPIO_Port, OUTPUT_5_Pin, 1);

      Target_Arm_Position = Home_Pulse;
      Is_Arm_Moving = 1;
      arm_action_state = ARM_RETURNING_HOME;
    }
    else if (arm_action_state == ARM_RETURNING_HOME)
    {
      HAL_GPIO_WritePin(OUTPUT_5_GPIO_Port, OUTPUT_5_Pin, 0);
      arm_action_state = ARM_IDLE;
    }
  }
}
/*----------------------------------Timer Handle------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /*----------------------------------1ms TIMER INTERRUPT----------------------------------*/
  if (htim->Instance == TIM17)
  {
    /*-----------------------------------Update Waypoint-----------------------------------*/
    if (Is_Moving_Waypoints)
    {
      if (current_time < profile[mp_count].T)
      {
        current_time += 0.001f;
      }

      Waypoint sp = Motion_Profile_Compute(&profile[mp_count], current_time);

      Swerve_Robot.setpoint_x = sp.x;
      Swerve_Robot.setpoint_y = sp.y;
      Swerve_Robot.fix_angle = sp.theta;

      if (Motion_Profile_Finished(&profile[mp_count], current_time))
      {
        mp_count++;
        Is_Moving_Waypoints = 0;
        Swerve_Robot.Is_Auto = 0;
        current_time = 0.0f;
      }
    }

    /*-----------------------------------Accel_Calculate-----------------------------------*/
    if (Swerve_Robot.Is_Auto)
    {
      RobotAcceleration_Update(&Swerve_Acceleration, 0.001);
    }

    Swerve_Robot.max_speed = 2.8f - (1.8f * (PS4_Dat.r2_analog) / 255.0f);
    Swerve_Robot.max_omega = Swerve_Robot.max_speed * 1.2f;
    /*----------------------------ROBOT CONTROL CALCULATIONS----------------------------*/

    Encoder_UpdateOdometry(&Encoder_X_Local, &Encoder_Y_Local, IMU.Yaw, &Encoder_X_Loc, &Encoder_Y_Loc);
    Swerve_4X_Field_Control(&Swerve_Robot, &PS4_Dat, IMU.Yaw, 0, Encoder_X_Loc, Encoder_Y_Loc);
    for (uint8_t i = 0; i < 4; i++)
    {
      Wheel_Motor[i].Set_Point = (int16_t)(Swerve_Robot.u[i]);
      Steering_Motor[i].Set_Point = (int16_t)(Swerve_Robot.steering_angles[i] * 3050.0f / 360.0f); // 3032 pulse/rev of steering motor
    }

    /*--------------------------TIMEOUT TIMER UPDATE & HANDLE--------------------------*/
    Timer_Timeout_Check(&Task_TO);

    if (Task_TO[0].Time_Out_Flag == 1 || Page_State != HOME_PAGE || !Swerve_Robot.Is_Fiels_Control)
    {
      Set_LED_State(LED_FAULT, 1);
      Set_LED_State(LED_RUN, 0);
      Robot_Hold_Stop();
    }
    else
    {
      Set_LED_State(LED_FAULT, 0);
      Set_LED_State(LED_RUN, 1);
    }

    if (Task_TO[1].Time_Out_Flag == 1)
    {
      Swerve_Robot.Is_Yaw_Fix = 0;
      Swerve_Robot.Is_Fiels_Control = 0;
    }

    /*----------------------------------SUB_TIMER TASKS----------------------------------*/
    Timer_OVF_Flag_0++;
    Timer_OVF_Flag_1++;
    Timer_OVF_Flag_2++;

    if (Timer_OVF_Flag_0 > 500)
    {
      Timer_OVF_Flag_0 = 0;
    }
    if (Timer_OVF_Flag_1 > 25)
    {
      Timer_OVF_Flag_1 = 0;
      Update_Nextion_Page();
      Nextion_Process(&nextion);
    }
    if (Timer_OVF_Flag_2 > 10)
    {
      Timer_OVF_Flag_2 = 0;
      PS4_UART_Req();
    }

    /*--------------------------------Buzzer & LED indicator--------------------------------*/
    Buzzer.state ? Buzzer_Update() : 0;

    Set_LED_State(LED_A, Swerve_Robot.Play_Field ? 0 : 1);
    Set_LED_State(LED_B, Swerve_Robot.Play_Field ? 1 : 0);
  }
  else if (htim->Instance == TIM16)
  {
    HandleButtonPress();
    Arm_Calculate();
  }
}

/*---------------------------------------Sub Functions---------------------------------------*/
void Robot_Hold_Stop(void)
{
  Wheel_Motor[0].Set_Point = 0;
  Wheel_Motor[1].Set_Point = 0;
  Wheel_Motor[2].Set_Point = 0;
  Wheel_Motor[3].Set_Point = 0;
  Steering_Motor[0].Set_Point = 400;
  Steering_Motor[1].Set_Point = -400;
  Steering_Motor[2].Set_Point = 400;
  Steering_Motor[3].Set_Point = -400;
}

/*------------------------------------UART Tx Rx IT Handle--------------------------------------*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == Steering_Motor->Driver_UART->Instance)
  {
    Driver_Send_Setpoints_U2(&Steering_Motor, 5);
  }
  if (huart->Instance == Wheel_Motor->Driver_UART->Instance)
  {
    Wheel_Motor[4].Set_Point = Current_Arm_Position;
    Driver_Send_Setpoints_U1(&Wheel_Motor, 5);
  }
  if (huart->Instance == nextion.nextionUARTHandle->Instance)
  {
    Nextion_UART_TxCpltCallback(&nextion);
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  NextionUpdate(huart, &nextion);
  if (huart->Instance == USART1)
  {
    PS4_UART_Rx_IDLE_Handle();
    Timer_Timeout_Reset(&Task_TO[0]);
  }
  if (huart->Instance == USART3)
  {
    WT901C_UART_Rx_IDLE_Hanlde(&IMU);
    Timer_Timeout_Reset(&Task_TO[1]);
  }
}

/*------------------------Pick-Standby-Release Ball------------------------*/
void Arm_Standby_Pick(void)
{
  Steering_Motor[4].Set_Point = 100;
  Target_Arm_Position = Picking_Ball_Pulse;
  Is_Arm_Moving = 1;
}
void Arm_Pick_Ball(void)
{
  Steering_Motor[4].Set_Point = 100;
  Target_Arm_Position = Picking_Ball_Pulse;
  Is_Arm_Moving = 1;
  while (Current_Arm_Position != Picking_Ball_Pulse)
    ;
  HAL_Delay(150);
  Steering_Motor[4].Set_Point = 0;
  HAL_Delay(150);
}
void Arm_Relese_Ball(void)
{
  Steering_Motor[4].Set_Point == 100;
  HAL_Delay(1000);
}
void Arm_Stanby_LowSilo(void)
{
  Target_Arm_Position = Low_Silo_Pulse;
  Is_Arm_Moving = 1;
}
void Arm_Stanby_MidSilo(void)
{
  Target_Arm_Position = Mid_Silo_Pulse;
  Is_Arm_Moving = 1;
}
void Arm_Stanby_HighSilo(void)
{
  Target_Arm_Position = High_Silo_Pulse;
  Is_Arm_Moving = 1;
}
void Toggle_Solenoid(void)
{
  if (Steering_Motor[4].Set_Point == 100)
  {
    Steering_Motor[4].Set_Point = 0;
  }
  else if (Steering_Motor[4].Set_Point == 0)
  {
    Steering_Motor[4].Set_Point = 100;
  }
}

/*-----------------------------Button PS4 Handles-----------------------------*/
void HandleButtonPress(void)
{
  switch (Button_Hold_State)
  {
  case BUTTON_IDLE:
    if (Button_State != 0) // Nếu có nút được nhấn
    {
      Last_Button = Button_State;
      Button_Hold_State = BUTTON_PRESSED;
    }
    break;

  case BUTTON_PRESSED:
    if (Button_State == 0) // Khi nút được thả ra
    {
      Button_Hold_State = BUTTON_RELEASED;
    }
    break;

  case BUTTON_RELEASED:
    switch (Last_Button)
    {
    case BUTTON_TRIANGLE:
      Target_Arm_Position = High_Silo_Pulse;
      Is_Arm_Moving = 1;
      break;

    case BUTTON_SQUARE:
      Target_Arm_Position = Mid_Silo_Pulse;
      Is_Arm_Moving = 1;
      break;

    case BUTTON_CROSS:
      Target_Arm_Position = Low_Silo_Pulse;
      Is_Arm_Moving = 1;
      break;

    case BUTTON_TOUCHPAD:
      Steering_Motor[4].Set_Point = 100;
      Target_Arm_Position = Picking_Ball_Pulse;
      Is_Arm_Moving = 1;
      break;

    case BUTTON_L1:
      Toggle_Solenoid();
      break;

    case BUTTON_R1:
      Swerve_Robot.Is_Yaw_Fix = 0;
      Set_Buzzer_State(200);
      break;

    case BUTTON_CIRCLE:
      Swerve_Robot.fix_angle = 0;
      Swerve_Robot.Is_Yaw_Fix = 1;
      Set_Buzzer_State(100);
      break;

    case BUTTON_SHARE:
      Swerve_Robot.Is_Auto = 1;
      Is_Moving_Waypoints = 1;
      break;
    default:
      break;
    }
    Button_Hold_State = BUTTON_IDLE; // Reset trạng thái sau khi xử lý xong
    break;
  }
}

/*--------------------------------Exit function--------------------------------*/

/*------------------------------Robot Move Function------------------------------*/

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM17_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  /*--------------------------------Init Waypoints--------------------------------*/

  Motion_Profile_Init(&profile[0], A_W, B_W, 400, 900);
  Motion_Profile_Init(&profile[1], B_W, C_W, 300, 800);
  Motion_Profile_Init(&profile[2], C_W, D_W, 900, 1600);

  Motion_Profile_Init(&profile[3], D_W, E_W, 500, 1200);
  // Motion_Profile_Init(&profile[4], E_W, F_W, 800, 1600);
  // Motion_Profile_Init(&profile[5], F_W, G_W, 500, 1000);
  /*----------------------------------INIT IMU----------------------------------*/
  WT901C_Init(&IMU, &huart3);
  WT901C_Begin_Recieve(&IMU);

  /*----------------------------------INIT PS4----------------------------------*/
  PS4_Init(&huart1);

  /*----------------------------------INIT HMI----------------------------------*/
  NextionInit(&nextion, &huart6);
  Init_Nextion_Comp();

  /*----------------------------Init Encoder Measure----------------------------*/
  Encoder_Init(&Encoder_X_Local, &htim1, 1, 800, 60);
  Encoder_Init(&Encoder_Y_Local, &htim4, 1, 800, 60);
  /*Init Sick DT50*/

  /*-----------------------INIT SWERVE ROBOT CALCULATIONS-----------------------*/
  Swerve_4X_Init(&Swerve_Robot, 3, 2.5);

  /*-----------------------------INIT MOTOR DRIVERS-----------------------------*/
  Driver_PID_AML_Init_UART(&huart7, Steering_Motor);
  Driver_PID_AML_Init_UART(&huart8, Wheel_Motor);

  Assign_PID_AML_Id(&Wheel_Motor[0], 1);
  Assign_PID_AML_Id(&Wheel_Motor[1], 2);
  Assign_PID_AML_Id(&Wheel_Motor[2], 3);
  Assign_PID_AML_Id(&Wheel_Motor[3], 4);
  Assign_PID_AML_Id(&Wheel_Motor[4], 5);

  Assign_PID_AML_Id(&Steering_Motor[0], 1);
  Assign_PID_AML_Id(&Steering_Motor[1], 2);
  Assign_PID_AML_Id(&Steering_Motor[2], 3);
  Assign_PID_AML_Id(&Steering_Motor[3], 4);
  Assign_PID_AML_Id(&Steering_Motor[4], 5);
  Steering_Motor[4].Set_Point = 0;
  Driver_Send_Setpoints_U1(&Wheel_Motor, 5);
  Driver_Send_Setpoints_U2(&Steering_Motor, 5);

  /*----------------------------INIT ROBOT ACCELERATION----------------------------*/
  float V_Robot_Target = 0;

  RobotAcceleration_Init(&Swerve_Acceleration, 0, 0);
  Set_Buzzer_State(500);

  /*-----------------------------INIT TIMEOUT TIMER-----------------------------*/
  Timeout_Begin();
  Timer_Timeout_Start(&htim17);
  HAL_TIM_Base_Start_IT(&htim16);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Robot_Reset_Home ? (Robot_Reset_Home = 0, Encoder_Reset(&Encoder_Y_Local, &Encoder_Y_Loc), Encoder_Reset(&Encoder_X_Local, &Encoder_X_Loc), WT901C_Reset_Angles(&IMU), Set_Buzzer_State(100)) : 0;

    switch (mp_count)
    {
    case 1:
      Arm_Standby_Pick();
      Swerve_Robot.Is_Auto = 1;
      Is_Moving_Waypoints = 1;
      break;
    case 2:
      if (!action_done_flag)
      {
        action_done_flag = 1;
        Arm_Pick_Ball();
        Arm_Stanby_LowSilo();
      }
      Swerve_Robot.Is_Auto = 1;
      Is_Moving_Waypoints = 1;
      break;
    case 3:
      // action_done_flag = 0;
      if (action_done_flag)
      {
        action_done_flag = 0;
        Swerve_Robot.Is_Auto = 0;
        Is_Moving_Waypoints = 0;

        Arm_Relese_Ball();
        Arm_Standby_Pick();
      }
      Swerve_Robot.Is_Auto = 1;
      Is_Moving_Waypoints = 1;
      break;

      break;
    default:
      break;
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
   */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 0;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 239;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);
}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 3;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 53999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 3;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 59999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */
}

/**
 * @brief UART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */
}

/**
 * @brief UART7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */
}

/**
 * @brief UART8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart8, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart8, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TIM5_DIR1_Pin | TIM5_DIR2_Pin | TIM5_DIR3_Pin | TIM5_DIR4_Pin | OUTPUT_1_Pin | OUTPUT_0_Pin | LED_2_Pin | LED_1_Pin | LED_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OUTPUT_5_Pin | OUTPUT_4_Pin | OUTPUT_3_Pin | OUTPUT_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI4_CS_Pin */
  GPIO_InitStruct.Pin = SPI4_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI4_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TIM5_DIR1_Pin TIM5_DIR2_Pin TIM5_DIR3_Pin TIM5_DIR4_Pin
                           OUTPUT_1_Pin OUTPUT_0_Pin LED_2_Pin LED_1_Pin
                           LED_0_Pin */
  GPIO_InitStruct.Pin = TIM5_DIR1_Pin | TIM5_DIR2_Pin | TIM5_DIR3_Pin | TIM5_DIR4_Pin | OUTPUT_1_Pin | OUTPUT_0_Pin | LED_2_Pin | LED_1_Pin | LED_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_0_Pin INPUT_1_Pin INPUT_2_Pin INPUT_3_Pin */
  GPIO_InitStruct.Pin = INPUT_0_Pin | INPUT_1_Pin | INPUT_2_Pin | INPUT_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_4_Pin INPUT_5_Pin */
  GPIO_InitStruct.Pin = INPUT_4_Pin | INPUT_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_5_Pin OUTPUT_4_Pin OUTPUT_3_Pin OUTPUT_2_Pin */
  GPIO_InitStruct.Pin = OUTPUT_5_Pin | OUTPUT_4_Pin | OUTPUT_3_Pin | OUTPUT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_3_Pin */
  GPIO_InitStruct.Pin = LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
   */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
