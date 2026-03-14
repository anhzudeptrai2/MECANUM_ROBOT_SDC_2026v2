/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "PS4_ESP.h"
#include "PID.h"
#include "DRIVER_PID_AML.h"
#include "TIMER_TIMEOUT.h"
#include "MECANUM_FIELD_KIN.h"
#include "WT901C.h"
#include "ROBOT_ACCELERATION.h"

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

/* USER CODE BEGIN PV */
// Bien PS4
extern volatile BS Button_State;
volatile BS Last_Button = BUTTON_NONE;
typedef enum
{
  BUTTON_IDLE,
  BUTTON_PRESSED,
  BUTTON_RELEASED
} ButtonState_t;
volatile ButtonState_t Button_Hold_State = BUTTON_IDLE;

volatile uint8_t Button_Debounce_Counter = 0;
#define BUTTON_DEBOUNCE_THRESHOLD 5

// Timeout va Flags
Task_Timeout Task_TO[TASK_NUMS];
uint16_t Timer_OVF_Flag_2 = 0;
volatile uint16_t tick_1ms_pending = 0;
volatile uint8_t ps4_req_flag = 0;
volatile uint8_t wheel_send_flag = 0;
volatile uint8_t steering_send_flag = 0;

// Robot Mecanum
MRb Mecanum_4_Bot;
PID_TypeDef Mecanum_Omega_PID;

// IMU
WT901C IMU;
uint8_t Robot_Reset_Home = 0;
float initial_angle = 0.0f;

// Trang thai nut bam
uint8_t Button_X_Active = 0;
uint8_t Button_Circle_Active = 0;
int8_t Robot_Direction = 1;
uint8_t PS4_Ever_Connected = 0;

// Motor drivers
#define WHEEL_MOTOR_COUNT 4
#define STEERING_MOTOR_COUNT 2
#define STEERING0_MASK 0x01u
#define STEERING1_MASK 0x02u
#define STEERING0_PRESET_COUNT 3
Motor_Driver Wheel_Motors[WHEEL_MOTOR_COUNT];
Motor_Driver Steering_Motors[STEERING_MOTOR_COUNT];

volatile int16_t Steering_Target[STEERING_MOTOR_COUNT] = {0, 0};
volatile int16_t Steering_Home_Target[STEERING_MOTOR_COUNT] = {0, 0};
volatile uint8_t Steering_Dirty_Mask = 0;
static const int16_t Steering0_Presets[3] = {8400, 13000, 18500};
static const int16_t Steering0_OpenPresets[3] = {7000, 10000, 15500};
volatile uint8_t Steering0_Preset_Index = 0;

#define UP_DOUBLE_PRESS_MS 350
#define RIGHT_MULTI_PRESS_MS 450
#define TOUCHPAD_WHEEL_LOCK_MS 250
#define STEERING_HOME_REQUEST_DELAY_MS 120
#define STEERING_HOME_FIXED_ID5 0
#define STEERING_HOME_FIXED_ID6 0
#define LS_HOME_ID5_DEBOUNCE_MS 20u
#define LS_HOME_ID5_ACTIVE_STATE GPIO_PIN_RESET
#define STEERING_ID5_HOME_SEEK_TARGET (-28000)
volatile uint8_t Up_Click_Armed = 0;
volatile uint32_t Up_Last_Click_Tick = 0;
volatile uint8_t Right_Click_Count = 0;
volatile uint32_t Right_Last_Click_Tick = 0;
volatile uint32_t Wheel_Lock_Until_Tick = 0;
volatile uint8_t LS_Home_Id5_Stop_Request = 0;
volatile uint8_t LS_Home_Id5_Homing_Armed = 0;
volatile uint32_t LS_Home_Id5_Last_Tick = 0;
volatile uint8_t DBG_LS_Id5_PinState = 0u;
volatile uint8_t DBG_LS_Id5_HomingArmed = 0u;
volatile uint8_t DBG_LS_Id5_Pending = 0u;
volatile uint8_t DBG_LS_Id5_StopReq = 0u;
volatile uint8_t DBG_PS4_ButtonState = 0u;
volatile uint8_t DBG_PS4_LastButton = 0u;
volatile uint8_t DBG_L1_Home_Request_Count = 0u;
volatile uint8_t DBG_LS_Id5_EXTI_Count = 0u;
volatile uint8_t DBG_LS_Id5_Latched_Count = 0u;
// Buzzer
struct
{
  uint8_t active;
  uint16_t duration;
  uint16_t counter;
} Buzzer = {0};

RobotAcceleration Wheel_Acceleration[WHEEL_MOTOR_COUNT];
RobotAcceleration Steering_Acceleration[STEERING_MOTOR_COUNT];

float prev_vx_cmd = 0.0f;
float prev_vy_cmd = 0.0f;
float prev_omg_cmd = 0.0f;

typedef enum
{
  LED_A,
  LED_B,
  LED_RUN,
  LED_FAULT
} LED_TypeDef;

#define JOYSTICK_DEADBAND 0.06f
#define SLEW_VX_PER_MS 0.008f
#define SLEW_VY_PER_MS 0.008f
#define SLEW_OMG_PER_MS 0.008f
#define CONTROL_DT_SEC 0.001f
#define WHEEL_ACCEL_DURATION_SEC 0.18f
#define STEERING_ACCEL_DURATION_SEC 0.15f
#define STEERING_REFRESH_MS 20u
#define WHEEL1_GAIN 1.00f
#define WHEEL2_GAIN 0.68f
#define WHEEL3_GAIN 1.00f
#define WHEEL4_GAIN 1.00f

#define YAW_SPEED_SCALE 1.5f
#define YAW_FIX_ROT_INPUT_DEADBAND 0.04f
#define YAW_FIX_ROT_INTENT_THRESH 0.20f
#define YAW_FIX_ROT_INTENT_HOLD_MS 40
#define YAW_FIX_ENGAGE_ERR_DEG 1.8f
#define YAW_FIX_RELEASE_ERR_DEG 0.55f
#define YAW_FIX_STILL_SPEED_THRESH 0.08f
#define YAW_FIX_STILL_MAX_CORR 0.18f
#define YAW_FIX_MOVE_MAX_CORR 0.58f
#define YAW_FIX_FAST_ERR_START_DEG 8.0f
#define YAW_FIX_FAST_ERR_FULL_DEG 40.0f
#define YAW_FIX_FAST_MAX_CORR_RATIO 0.88f
#define YAW_FIX_KS_START_DEG 5.5f
#define YAW_FIX_KS_FULL_DEG 30.0f
#define YAW_FIX_KS_MIN 0.03f
#define YAW_FIX_KS_MAX 0.14f
#define YAW_FIX_DWELL_MS 45u
#define YAW_MIN_OMEGA 1.0f
#define YAW_CORR_SLEW_PER_MS 0.006f
#define YAW_CORR_SLEW_BOOST_PER_MS 0.012f
#define YAW_CORR_DECAY_PER_MS 0.003f

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Robot_Hold_Stop(void);
void HandleButtonPress(void);
void Position_Motor_Update(void);
void Buzzer_Beep(uint16_t duration_ms);
static void Reset_Button_Sequences(void);
static void Steering_Id5_LimitStop_RequestFromIsr(void);
static void Steering_Id5_LimitHome_Process(void);
static void Steering_Id5_LimitLevel_Monitor(void);
static void Handle_R1_OpenGrip_Request(void);
static uint8_t LS_Home_Id5_IsClosed(void);
static void LS_Id5_RequestStop(void);
static void Debug_Snapshot_Update(void);
static uint8_t LS_Home_Id5_IsClosed(void)
{
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(LS_HOME_ID5_GPIO_Port, LS_HOME_ID5_Pin);
  DBG_LS_Id5_PinState = (uint8_t)pin_state;
  return (pin_state == LS_HOME_ID5_ACTIVE_STATE) ? 1u : 0u;
}

static void LS_Id5_RequestStop(void)
{
  if (LS_Home_Id5_Stop_Request == 0u)
  {
    DBG_LS_Id5_Latched_Count++;
  }

  LS_Home_Id5_Stop_Request = 1u;
  DBG_LS_Id5_StopReq = LS_Home_Id5_Stop_Request;
}

static void Debug_Snapshot_Update(void)
{
  DBG_LS_Id5_PinState = (uint8_t)HAL_GPIO_ReadPin(LS_HOME_ID5_GPIO_Port, LS_HOME_ID5_Pin);
  DBG_LS_Id5_HomingArmed = LS_Home_Id5_Homing_Armed;
  DBG_LS_Id5_Pending = LS_Home_Id5_Stop_Request;
  DBG_LS_Id5_StopReq = LS_Home_Id5_Stop_Request;
  DBG_PS4_ButtonState = (uint8_t)(((uint16_t)Button_State) & 0xFFu);
  DBG_PS4_LastButton = (uint8_t)(((uint16_t)Last_Button) & 0xFFu);
}

static void Steering_Id5_LimitLevel_Monitor(void)
{
  if ((LS_Home_Id5_Homing_Armed != 0u) && (LS_Home_Id5_IsClosed() != 0u))
  {
    LS_Id5_RequestStop();
  }
}

static void Steering_SoftHome_FromReference(void)
{
  LS_Home_Id5_Stop_Request = 0u;
  LS_Home_Id5_Homing_Armed = 0u;

  // Home ID5 = vi tri thuc te tai thoi diem bat nguon.
  Driver_Set_Zero_Position(Steering_Motors[0]);
  HAL_Delay(STEERING_HOME_REQUEST_DELAY_MS);

  // ID6 van giu quy trinh home hien tai
  Driver_Home_Request(Steering_Motors[1]);
  HAL_Delay(STEERING_HOME_REQUEST_DELAY_MS);

  Steering_Home_Target[0] = STEERING_HOME_FIXED_ID5;
  Steering_Home_Target[1] = STEERING_HOME_FIXED_ID6;
  Steering_Target[0] = Steering_Home_Target[0];
  Steering_Target[1] = Steering_Home_Target[1];
  Steering0_Preset_Index = 0;
  Reset_Button_Sequences();

  Wheel_Lock_Until_Tick = HAL_GetTick() + TOUCHPAD_WHEEL_LOCK_MS;
  prev_vx_cmd = 0.0f;
  prev_vy_cmd = 0.0f;
  prev_omg_cmd = 0.0f;

  Position_Motor_Update();
  Steering_Dirty_Mask |= (STEERING0_MASK | STEERING1_MASK);
}

static void Steering_RequestTarget(uint8_t motor_idx, int16_t target)
{
  if (motor_idx >= STEERING_MOTOR_COUNT)
  {
    return;
  }

  // Lenh tay (khong phai home target/seek target) se huy che do homing ID5.
  if ((motor_idx == 0u) &&
      (target != Steering_Home_Target[0]) &&
      (target != STEERING_ID5_HOME_SEEK_TARGET))
  {
    LS_Home_Id5_Homing_Armed = 0u;
  }

  Steering_Target[motor_idx] = target;
  Steering_Dirty_Mask |= (uint8_t)(1u << motor_idx);
}

static void Reset_Button_Sequences(void)
{
  Right_Click_Count = 0;
  Up_Click_Armed = 0;
}

static void Handle_Right_MultiPress(void)
{
  uint32_t now_tick = HAL_GetTick();

  if ((now_tick - Right_Last_Click_Tick) <= RIGHT_MULTI_PRESS_MS)
  {
    if (Right_Click_Count < 2u)
    {
      Right_Click_Count++;
    }
  }
  else
  {
    Right_Click_Count = 1;
  }

  Right_Last_Click_Tick = now_tick;
  if (Right_Click_Count <= 1u)
  {
    Steering0_Preset_Index = 0u; // muc be nhat
  }
  else
  {
    Steering0_Preset_Index = 1u; // muc be thu 2
    Right_Click_Count = 0u;
  }

  Steering_RequestTarget(0, Steering0_Presets[Steering0_Preset_Index]);

  Buzzer_Beep(50);
}

static void Handle_Up_DoublePress(void)
{
  uint32_t now_tick = HAL_GetTick();

  if (Up_Click_Armed && ((now_tick - Up_Last_Click_Tick) <= UP_DOUBLE_PRESS_MS))
  {
    Steering_RequestTarget(1, 650);
    Up_Click_Armed = 0;
    Buzzer_Beep(120);
  }
  else
  {
    Steering_RequestTarget(1, 300);
    Up_Click_Armed = 1;
    Up_Last_Click_Tick = now_tick;
    Buzzer_Beep(80);
  }
}
static void Handle_L1_Home_Request(void)
{
  LS_Home_Id5_Homing_Armed = 1u;

  // ID6 van quay ve moc home co dinh.
  Steering_RequestTarget(1, Steering_Home_Target[1]);

  LS_Home_Id5_Stop_Request = 0u;
  DBG_L1_Home_Request_Count++;

  if (LS_Home_Id5_IsClosed() != 0u)
  {
    // Cong tac da dong (0): dung ngay va chot home.
    LS_Id5_RequestStop();
  }
  else
  {
    // Chua cham cong tac: quay ID5 theo chieu nguoc setpoint kep cho den khi cham LS.
    Steering_RequestTarget(0, STEERING_ID5_HOME_SEEK_TARGET);
  }

  Steering0_Preset_Index = 0;
  Reset_Button_Sequences();
  Wheel_Lock_Until_Tick = HAL_GetTick() + TOUCHPAD_WHEEL_LOCK_MS;
  Robot_Hold_Stop();
  prev_vx_cmd = 0.0f;
  prev_vy_cmd = 0.0f;
  prev_omg_cmd = 0.0f;
  Buzzer_Beep(80);
}

static void Handle_R1_OpenGrip_Request(void)
{
  int16_t current_target = Steering_Target[0];
  uint8_t nearest_idx = 0u;
  int32_t diff = (int32_t)current_target - (int32_t)Steering0_Presets[0];
  int32_t min_abs_diff;

  if (diff < 0)
  {
    diff = -diff;
  }
  min_abs_diff = diff;

  for (uint8_t i = 1u; i < STEERING0_PRESET_COUNT; i++)
  {
    int32_t d = (int32_t)current_target - (int32_t)Steering0_Presets[i];
    if (d < 0)
    {
      d = -d;
    }

    if (d < min_abs_diff)
    {
      min_abs_diff = d;
      nearest_idx = i;
    }
  }

  // R1: map muc kep hien tai sang muc mo tuong ung.
  Steering0_Preset_Index = nearest_idx;
  Steering_RequestTarget(0, Steering0_OpenPresets[nearest_idx]);
  Reset_Button_Sequences();
  Buzzer_Beep(70);
}

static void Steering_Id5_LimitStop_RequestFromIsr(void)
{
  if (LS_Home_Id5_Homing_Armed == 0u)
  {
    return;
  }

  LS_Id5_RequestStop();
}

static void Steering_Id5_LimitHome_Process(void)
{
  if (LS_Home_Id5_Stop_Request != 0u)
  {
    int16_t home_setpoint = Steering_Home_Target[0];

    LS_Home_Id5_Stop_Request = 0u;
    LS_Home_Id5_Homing_Armed = 0u;
    DBG_LS_Id5_StopReq = LS_Home_Id5_Stop_Request;
    DBG_LS_Id5_HomingArmed = LS_Home_Id5_Homing_Armed;

    // Cong tac = 0 (dong) khi ve home: dung ngay va chot lai home cho ID5.
    Driver_Set_Zero_Position(Steering_Motors[0]);

    // Sau khi chot zero, giu lenh tai moc home (0).
    Steering_Target[0] = home_setpoint;
    Steering_Motors[0].Set_Point = home_setpoint;
    RobotAcceleration_Init(&Steering_Acceleration[0], (float)home_setpoint, STEERING_ACCEL_DURATION_SEC);
    Steering_Dirty_Mask |= STEERING0_MASK;

    Steering0_Preset_Index = 0u;
    Reset_Button_Sequences();
    Buzzer_Beep(120);
  }
}

static float NormalizeAngleDeg(float angle_deg)
{
  while (angle_deg > 180.0f)
    angle_deg -= 360.0f;
  while (angle_deg < -180.0f)
    angle_deg += 360.0f;
  return angle_deg;
}

static float ClampFloat(float value, float min_val, float max_val)
{
  if (value > max_val)
    return max_val;
  if (value < min_val)
    return min_val;
  return value;
}

static int16_t ClampToInt16(float value)
{
  if (value > 32767.0f)
    return 32767;
  if (value < -32768.0f)
    return -32768;
  return (int16_t)value;
}

static int16_t AbsInt16(int16_t value)
{
  return (value < 0) ? (int16_t)(-value) : value;
}

void Set_LED_State(LED_TypeDef led, GPIO_PinState state)
{
  (void)led;
  (void)state;
}

void Buzzer_Beep(uint16_t duration_ms)
{
  if(!Buzzer.active)
  {
    Buzzer.active = 1;
    Buzzer.duration = duration_ms;
    Buzzer.counter = 0;
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
  }
}

void Buzzer_Update(void)
{
  if (Buzzer.active)
  {
    Buzzer.counter++;
    if (Buzzer.counter >= Buzzer.duration)
    {
      HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
      Buzzer.active = 0;
      Buzzer.counter = 0;
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM5)
  {
    if (tick_1ms_pending < 2000u)
    {
      tick_1ms_pending++;
    }

    if (++Timer_OVF_Flag_2 > 10)
    {
      Timer_OVF_Flag_2 = 0;
      ps4_req_flag = 1;
    }

    Buzzer_Update();
  }
  else if (htim->Instance == TIM16)
  {
    HandleButtonPress();
  }
}

void Robot_Hold_Stop(void)
{
  for (int i = 0; i < WHEEL_MOTOR_COUNT; i++)
  {
    Wheel_Motors[i].Set_Point = 0;
    RobotAcceleration_Init(&Wheel_Acceleration[i], 0.0f, WHEEL_ACCEL_DURATION_SEC);
  }
}

void Position_Motor_Update(void)
{
  for (uint8_t i = 0; i < STEERING_MOTOR_COUNT; i++)
  {
    float steering_target = (float)Steering_Target[i];
    RobotAcceleration_SetTarget(&Steering_Acceleration[i], steering_target, STEERING_ACCEL_DURATION_SEC);
    Steering_Motors[i].Set_Point = ClampToInt16(RobotAcceleration_Update(&Steering_Acceleration[i], CONTROL_DT_SEC));
  }
}

HAL_StatusTypeDef Steering_Send_Queued(void)
{
  if (steering_send_flag != 0u)
  {
    return HAL_BUSY;
  }

  HAL_StatusTypeDef st = Driver_Send_Setpoints_U2(Steering_Motors, STEERING_MOTOR_COUNT);
  if (st == HAL_OK)
  {
    steering_send_flag = 1;
    Steering_Dirty_Mask = 0;
  }

  return st;
}

/*------------------------------------UART Tx Rx IT Handle--------------------------------------*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    wheel_send_flag = 0;
  }
  if (huart->Instance == UART5)
  {
    steering_send_flag = 0;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)    
  {
    PS4_UART_Rx_IDLE_Handle();
    Timer_Timeout_Reset(&Task_TO[0]);
  }
  if (huart->Instance == USART2)
  {
    WT901C_UART_Rx_IDLE_Hanlde(&IMU);
    Timer_Timeout_Reset(&Task_TO[1]);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == LS_HOME_ID5_Pin)
  {
    DBG_LS_Id5_EXTI_Count++;
    uint32_t now_tick = HAL_GetTick();
    if ((now_tick - LS_Home_Id5_Last_Tick) < LS_HOME_ID5_DEBOUNCE_MS)
    {
      return;
    }

    LS_Home_Id5_Last_Tick = now_tick;
    if (LS_Home_Id5_IsClosed() != 0u)
    {
      Steering_Id5_LimitStop_RequestFromIsr();
    }
  }
}
// Xu ly nut bam PS4
void HandleButtonPress(void)
{
  switch (Button_Hold_State)
  {
  case BUTTON_IDLE:
    if (Button_State != 0)
    {
      Button_Debounce_Counter++;
      if (Button_Debounce_Counter >= BUTTON_DEBOUNCE_THRESHOLD)
      {
        Last_Button = Button_State;
        Button_Hold_State = BUTTON_PRESSED;
        Button_Debounce_Counter = 0;
      }
    }
    else
    {
      Button_Debounce_Counter = 0;

      // Huy chuoi nhan neu da vuot cua so thoi gian de tranh dinh trang thai cu.
      if ((Right_Click_Count != 0u) && ((HAL_GetTick() - Right_Last_Click_Tick) > RIGHT_MULTI_PRESS_MS))
      {
        Right_Click_Count = 0;
      }
      if (Up_Click_Armed && ((HAL_GetTick() - Up_Last_Click_Tick) > UP_DOUBLE_PRESS_MS))
      {
        Up_Click_Armed = 0;
      }
    }
    break;

  case BUTTON_PRESSED:
    if (Button_State == 0)
    {
      Button_Debounce_Counter++;
      if (Button_Debounce_Counter >= BUTTON_DEBOUNCE_THRESHOLD)
      {
        Button_Hold_State = BUTTON_RELEASED;
        Button_Debounce_Counter = 0;
      }
    }
    else
    {
      Button_Debounce_Counter = 0;
    }
    break;

  case BUTTON_RELEASED:
    // Theo code mau: xu ly theo nut don le de tranh kich nham khi co combo/noise bitmask.
    if ((((uint16_t)Last_Button) & ((uint16_t)BUTTON_L1)) != 0u)
    {
      Handle_L1_Home_Request();
      Button_Hold_State = BUTTON_IDLE;
      Last_Button = BUTTON_NONE;
      break;
    }
    if ((((uint16_t)Last_Button) & ((uint16_t)BUTTON_R1)) != 0u)
    {
      Handle_R1_OpenGrip_Request();
      Button_Hold_State = BUTTON_IDLE;
      Last_Button = BUTTON_NONE;
      break;
    }

    switch (Last_Button)
    {
    case BUTTON_CROSS:
      Button_X_Active = !Button_X_Active;
      if (Button_X_Active)
      {
        Mecanum_4_Bot.fix_angle = -IMU.Yaw;
        Mecanum_4_Bot.is_yaw_fix = 1;
        PID_Init(&Mecanum_Omega_PID);
        Button_Circle_Active = 0;
        Buzzer_Beep(100);
      }
      else
      {
        Mecanum_4_Bot.is_yaw_fix = 0;
        Buzzer_Beep(50);
      }
      break;

    case BUTTON_TRIANGLE:
      Robot_Reset_Home = 1;
      Button_X_Active = 0;
      Button_Circle_Active = 0;
      Mecanum_4_Bot.is_yaw_fix = 0;
      Buzzer_Beep(150);
      break;

    case BUTTON_L1:
      Handle_L1_Home_Request();
      break;

    case BUTTON_R1:
      Handle_R1_OpenGrip_Request();
      break;

    case BUTTON_LEFT:
      Steering0_Preset_Index = (STEERING0_PRESET_COUNT - 1u); // muc lon nhat
      Steering_RequestTarget(0, Steering0_Presets[Steering0_Preset_Index]);
      Right_Click_Count = 0;
      Buzzer_Beep(50);
      break;

    case BUTTON_RIGHT:
      Handle_Right_MultiPress();
      break;

    case BUTTON_UP:
      Handle_Up_DoublePress();
      break;

    case BUTTON_DOWN:
      Steering_RequestTarget(1, -300);
      Buzzer_Beep(50);
      break;

    case BUTTON_TOUCHPAD:
      Robot_Direction = -Robot_Direction;
      Buzzer_Beep(80);
      break;

    case BUTTON_SQUARE:
      Buzzer_Beep(50);
      break;

    case BUTTON_OPTIONS:
      Buzzer_Beep(50);
      break;

    default:
      break;
    }

    Last_Button = BUTTON_NONE;
    Button_Hold_State = BUTTON_IDLE;
    Button_Debounce_Counter = 0;
    break;
  }

  // Cap nhat trang thai giu goc
  if (Button_X_Active)
    Mecanum_4_Bot.is_yaw_fix = 1;
}

// Xu ly PS4
void PS4_Process(void)
{
  if (ps4_req_flag)
  {
    ps4_req_flag = 0;
    PS4_UART_Req();
  }
}

// Tinh toan dong hoc Mecanum
void Mecanum_Calculate(void)
{
  if ((int32_t)(Wheel_Lock_Until_Tick - HAL_GetTick()) > 0)
  {
    Robot_Hold_Stop();
    prev_vx_cmd = 0.0f;
    prev_vy_cmd = 0.0f;
    prev_omg_cmd = 0.0f;
    return;
  }

  float mapped_x, mapped_y, mapped_z;
  float vx_cmd, vy_cmd, omg_cmd;
  float angle_error;
  float vx_diff, vy_diff, omg_diff;
  static uint16_t yaw_manual_rotate_counter = 0;
  static float yaw_corr_prev = 0.0f;
  static uint8_t yaw_hold_active = 0;
  static uint16_t yaw_settle_counter = 0;

  Mecanum_4_Bot.max_speed = 1.68f - (1.08f * PS4_Dat.r2_analog / 255.0f);

  if (PS4_Dat.l2_analog > 100)
  {
    Mecanum_4_Bot.max_speed *= 2.0f;
  }

  Mecanum_4_Bot.max_omega = Mecanum_4_Bot.max_speed * 0.8f * YAW_SPEED_SCALE;
  if (Mecanum_4_Bot.max_omega < YAW_MIN_OMEGA)
  {
    Mecanum_4_Bot.max_omega = YAW_MIN_OMEGA;
  }

  // Doc joystick va chuan hoa
  mapped_x = (float)PS4_Dat.l_stick_x / 128.0f;
  mapped_y = (float)PS4_Dat.l_stick_y / 128.0f;
  mapped_z = (float)PS4_Dat.r_stick_x / 128.0f;

  // Deadband
  if (mapped_x > -JOYSTICK_DEADBAND && mapped_x < JOYSTICK_DEADBAND)
    mapped_x = 0.0f;
  if (mapped_y > -JOYSTICK_DEADBAND && mapped_y < JOYSTICK_DEADBAND)
    mapped_y = 0.0f;
  if (mapped_z > -JOYSTICK_DEADBAND && mapped_z < JOYSTICK_DEADBAND)
    mapped_z = 0.0f;

  // Tinh van toc mong muon (ap dung dao chieu)
  vx_cmd = mapped_y * Mecanum_4_Bot.max_speed * Robot_Direction;
  vy_cmd = mapped_x * Mecanum_4_Bot.max_speed * Robot_Direction;
  omg_cmd = -mapped_z * Mecanum_4_Bot.max_omega;

  // Gioi han gia toc
  vx_diff = vx_cmd - prev_vx_cmd;
  vy_diff = vy_cmd - prev_vy_cmd;
  omg_diff = omg_cmd - prev_omg_cmd;

  if (vx_diff > SLEW_VX_PER_MS)
    vx_cmd = prev_vx_cmd + SLEW_VX_PER_MS;
  else if (vx_diff < -SLEW_VX_PER_MS)
    vx_cmd = prev_vx_cmd - SLEW_VX_PER_MS;

  if (vy_diff > SLEW_VY_PER_MS)
    vy_cmd = prev_vy_cmd + SLEW_VY_PER_MS;
  else if (vy_diff < -SLEW_VY_PER_MS)
    vy_cmd = prev_vy_cmd - SLEW_VY_PER_MS;

  if (omg_diff > SLEW_OMG_PER_MS)
    omg_cmd = prev_omg_cmd + SLEW_OMG_PER_MS;
  else if (omg_diff < -SLEW_OMG_PER_MS)
    omg_cmd = prev_omg_cmd - SLEW_OMG_PER_MS;

  // Luu gia tri cho lan sau
  prev_vx_cmd = vx_cmd;
  prev_vy_cmd = vy_cmd;
  prev_omg_cmd = omg_cmd;

  // Cap nhat goc IMU
  Mecanum_4_Bot.IMU_theta = -IMU.Yaw;

  // Khoa goc toi uu: co hysteresis + vung im khi dung yen de triet rung lac.
  if (Mecanum_4_Bot.is_yaw_fix)
  {
    float yaw_corr_target = 0.0f;
    float shortest_err_deg = NormalizeAngleDeg((float)(Mecanum_4_Bot.fix_angle - Mecanum_4_Bot.IMU_theta));
    float transl_speed = sqrtf((vx_cmd * vx_cmd) + (vy_cmd * vy_cmd));
    uint8_t is_still = (transl_speed < YAW_FIX_STILL_SPEED_THRESH);
    float hold_engage_deg = is_still ? YAW_FIX_ENGAGE_ERR_DEG : (YAW_FIX_ENGAGE_ERR_DEG * 0.8f);
    float hold_release_deg = is_still ? YAW_FIX_RELEASE_ERR_DEG : (YAW_FIX_RELEASE_ERR_DEG * 1.4f);
    float max_corr_base = is_still ? YAW_FIX_STILL_MAX_CORR : YAW_FIX_MOVE_MAX_CORR;
    float max_corr = max_corr_base;
    float fast_err_ratio = 0.0f;
    float corr_slew = YAW_CORR_SLEW_PER_MS;

    Mecanum_4_Bot.fix_angle = Mecanum_4_Bot.IMU_theta + shortest_err_deg;
    angle_error = fabsf(shortest_err_deg);

    if (angle_error > YAW_FIX_FAST_ERR_START_DEG)
    {
      fast_err_ratio = (angle_error - YAW_FIX_FAST_ERR_START_DEG) /
                       (YAW_FIX_FAST_ERR_FULL_DEG - YAW_FIX_FAST_ERR_START_DEG);
      fast_err_ratio = ClampFloat(fast_err_ratio, 0.0f, 1.0f);

      {
        float boosted_cap = max_corr_base + fast_err_ratio *
                                            ((Mecanum_4_Bot.max_omega * YAW_FIX_FAST_MAX_CORR_RATIO) - max_corr_base);
        if (boosted_cap > max_corr)
        {
          max_corr = boosted_cap;
        }
      }
    }

    corr_slew += (fast_err_ratio * YAW_CORR_SLEW_BOOST_PER_MS);

    if (fabsf(mapped_z) > YAW_FIX_ROT_INTENT_THRESH)
    {
      if (yaw_manual_rotate_counter < YAW_FIX_ROT_INTENT_HOLD_MS)
      {
        yaw_manual_rotate_counter++;
      }

      if (yaw_manual_rotate_counter >= YAW_FIX_ROT_INTENT_HOLD_MS)
      {
        Mecanum_4_Bot.fix_angle = -IMU.Yaw;
        yaw_hold_active = 0u;
        yaw_settle_counter = 0u;
        yaw_corr_prev = 0.0f;
        PID_Init(&Mecanum_Omega_PID);
      }
    }
    else
    {
      yaw_manual_rotate_counter = 0u;

      if (fabsf(mapped_z) < YAW_FIX_ROT_INPUT_DEADBAND)
      {
        if (!yaw_hold_active)
        {
          if (angle_error > hold_engage_deg)
          {
            yaw_hold_active = 1u;
            yaw_settle_counter = 0u;
          }
        }
        else if (angle_error < hold_release_deg)
        {
          if (yaw_settle_counter < YAW_FIX_DWELL_MS)
          {
            yaw_settle_counter++;
          }
          else
          {
            yaw_hold_active = 0u;
            PID_Init(&Mecanum_Omega_PID);
          }
        }
        else
        {
          yaw_settle_counter = 0u;
        }

        if (yaw_hold_active)
        {
          PID_Compute(&Mecanum_Omega_PID);
          extern double Mecanum_Omega_PID_Out;
          yaw_corr_target = (float)Mecanum_Omega_PID_Out;

          if (angle_error > YAW_FIX_KS_START_DEG)
          {
            float ks_ratio = (angle_error - YAW_FIX_KS_START_DEG) /
                             (YAW_FIX_KS_FULL_DEG - YAW_FIX_KS_START_DEG);
            float yaw_ks;
            ks_ratio = ClampFloat(ks_ratio, 0.0f, 1.0f);
            yaw_ks = YAW_FIX_KS_MIN + ks_ratio * (YAW_FIX_KS_MAX - YAW_FIX_KS_MIN);
            yaw_corr_target += (shortest_err_deg >= 0.0f) ? yaw_ks : -yaw_ks;
          }

          yaw_corr_target = ClampFloat(yaw_corr_target, -max_corr, max_corr);
        }
      }
      else
      {
        // Vung trung gian cua input xoay: xa d?n correction de tranh giat.
        yaw_hold_active = 0u;
        yaw_settle_counter = 0u;
        PID_Init(&Mecanum_Omega_PID);
      }
    }

    if (!yaw_hold_active && (fabsf(mapped_z) < YAW_FIX_ROT_INPUT_DEADBAND))
    {
      if (yaw_corr_prev > YAW_CORR_DECAY_PER_MS)
      {
        yaw_corr_prev -= YAW_CORR_DECAY_PER_MS;
      }
      else if (yaw_corr_prev < -YAW_CORR_DECAY_PER_MS)
      {
        yaw_corr_prev += YAW_CORR_DECAY_PER_MS;
      }
      else
      {
        yaw_corr_prev = 0.0f;
      }
    }
    else
    {
      float corr_diff = yaw_corr_target - yaw_corr_prev;
      if (corr_diff > corr_slew)
      {
        corr_diff = corr_slew;
      }
      else if (corr_diff < -corr_slew)
      {
        corr_diff = -corr_slew;
      }
      yaw_corr_prev += corr_diff;
    }

    if (is_still && (angle_error < hold_engage_deg) && (fabsf(yaw_corr_prev) < (YAW_FIX_STILL_MAX_CORR * 0.20f)))
    {
      yaw_corr_prev = 0.0f;
    }

    omg_cmd += yaw_corr_prev;
  }
  else
  {
    yaw_manual_rotate_counter = 0u;
    yaw_hold_active = 0u;
    yaw_settle_counter = 0u;
    yaw_corr_prev = 0.0f;
  }

  omg_cmd = ClampFloat(omg_cmd, -Mecanum_4_Bot.max_omega, Mecanum_4_Bot.max_omega);

  // Tinh dong hoc Mecanum
  MecanumRobot_SetMotion(&Mecanum_4_Bot, vx_cmd, vy_cmd, omg_cmd, IMU.Yaw, 0);

  float wheel_targets[WHEEL_MOTOR_COUNT] = {
      Mecanum_4_Bot.u[0] * WHEEL1_GAIN,
      -Mecanum_4_Bot.u[1] * WHEEL2_GAIN,
      -Mecanum_4_Bot.u[2] * WHEEL3_GAIN,
      -Mecanum_4_Bot.u[3] * WHEEL4_GAIN};

  for (uint8_t i = 0; i < WHEEL_MOTOR_COUNT; i++)
  {
    RobotAcceleration_SetTarget(&Wheel_Acceleration[i], wheel_targets[i], WHEEL_ACCEL_DURATION_SEC);
    Wheel_Motors[i].Set_Point = ClampToInt16(RobotAcceleration_Update(&Wheel_Acceleration[i], CONTROL_DT_SEC));
  }
}

// Cap nhat motor
void Motor_Update(void)
{
  static uint8_t prev_timeout_state = 0;
  static uint8_t first_connect_beep_done = 0;
  static uint8_t steering_refresh_counter = 0;
  Timer_Timeout_Check(Task_TO);

  if (Task_TO[0].Time_Out_Flag == 1)
  {
    Set_LED_State(LED_FAULT, GPIO_PIN_SET);
    Set_LED_State(LED_RUN, GPIO_PIN_RESET);
    Robot_Hold_Stop();
    prev_timeout_state = 1;
  }
  else
  {
    Set_LED_State(LED_FAULT, GPIO_PIN_RESET);
    Set_LED_State(LED_RUN, GPIO_PIN_SET);

    if (prev_timeout_state == 1)
    {
      Buzzer_Beep(100);
      prev_timeout_state = 0;
    }

    if (!first_connect_beep_done && (PS4_LastRxTick != 0u))
    {
      PS4_Ever_Connected = 1;
      first_connect_beep_done = 1;
      Buzzer_Beep(100);
    }
  }

  if (wheel_send_flag == 0)
  {
    if (Driver_Send_Setpoints_U1(Wheel_Motors, WHEEL_MOTOR_COUNT) == HAL_OK)
    {
      wheel_send_flag = 1;
    }
  }

  if (steering_refresh_counter < STEERING_REFRESH_MS)
  {
    steering_refresh_counter++;
  }

  if ((steering_send_flag == 0u) && ((Steering_Dirty_Mask != 0u) || (steering_refresh_counter >= STEERING_REFRESH_MS)))
  {
    if (Steering_Send_Queued() == HAL_OK)
    {
      steering_refresh_counter = 0;
    }
  }
}

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
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  // Khoi tao PS4
  PS4_Init(&huart1);

  // Khoi tao IMU 
  WT901C_Init(&IMU, &huart2);
  WT901C_Begin_Recieve(&IMU);

  // Khoi tao Robot Mecanum
  MecanumRobot_Init(&Mecanum_4_Bot, Robot_Max_Speed, Robot_Max_Omega);

  // Khoi tao Motor drivers
  Driver_PID_AML_Init_UART(&huart3, Wheel_Motors);
  Driver_PID_AML_Init_UART(&huart5, Steering_Motors);
  for (uint8_t i = 0; i < WHEEL_MOTOR_COUNT; i++)
  {
    Wheel_Motors[i].Driver_UART = &huart3;
  }
  for (uint8_t i = 0; i < STEERING_MOTOR_COUNT; i++)
  {
    Steering_Motors[i].Driver_UART = &huart5;
  }
  Assign_PID_AML_Id(&Wheel_Motors[0], 1);
  Assign_PID_AML_Id(&Wheel_Motors[1], 2);
  Assign_PID_AML_Id(&Wheel_Motors[2], 3);
  Assign_PID_AML_Id(&Wheel_Motors[3], 4);
  Assign_PID_AML_Id(&Steering_Motors[0], 5);
  Assign_PID_AML_Id(&Steering_Motors[1], 6);

  for (uint8_t i = 0; i < WHEEL_MOTOR_COUNT; i++)
  {
    RobotAcceleration_Init(&Wheel_Acceleration[i], 0.0f, WHEEL_ACCEL_DURATION_SEC);
  }

  for (uint8_t i = 0; i < STEERING_MOTOR_COUNT; i++)
  {
    RobotAcceleration_Init(&Steering_Acceleration[i], (float)Steering_Target[i], STEERING_ACCEL_DURATION_SEC);
  }

  Steering_SoftHome_FromReference();
  // Tu dong tim home ID5 ngay khi bat main (khong can bam L1).
  Handle_L1_Home_Request();
  Steering_Id5_LimitLevel_Monitor();
  Steering_Id5_LimitHome_Process();

  (void)Driver_Send_Setpoints_U1(Wheel_Motors, WHEEL_MOTOR_COUNT);
  (void)Steering_Send_Queued();

  // Khoi tao Timeout Timer
  Timeout_Begin();
  Timer_Timeout_Start(&htim5);
  HAL_TIM_Base_Start_IT(&htim16);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Debug_Snapshot_Update();
    Steering_Id5_LimitLevel_Monitor();
    Steering_Id5_LimitHome_Process();

    /*----------------------------XU LY RESET GOC IMU----------------------------*/
    if (Robot_Reset_Home)
    {
      Robot_Reset_Home = 0;
      WT901C_Reset_Angles(&IMU);
      Buzzer_Beep(120);
    }
    /*----------------------------MAIN LOOP - X\u1eeaL L\u00dd THEO FLAG----------------------------*/
    /* Ch\u1ec9 x\u1eed l\u00fd khi c\u00f3 flag t\u1eeb timer interrupt */
    if (tick_1ms_pending != 0u)
    {
      uint16_t pending_ticks;

      __disable_irq();
      pending_ticks = tick_1ms_pending;
      tick_1ms_pending = 0u;
      __enable_irq();

      while (pending_ticks-- > 0u)
      {
        /* 1. Xu ly PS4 request */
        PS4_Process();

        /* 2. Tinh toan dong hoc Mecanum */
        Mecanum_Calculate();

        /* 3. Cap nhat motor (bao gom timeout check) */
        Debug_Snapshot_Update();
        Steering_Id5_LimitLevel_Monitor();
        Steering_Id5_LimitHome_Process();
        Position_Motor_Update();
        Motor_Update();
      }
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
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











