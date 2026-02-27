#ifndef ENCODER_H
#define ENCODER_H

#include "stm32h7xx_hal.h"
#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t pulse_divider;
    uint32_t pulses_per_rev;
    float wheel_diameter;

    int32_t last_raw;
    int32_t overflow;
    uint16_t max_value;
} Encoder_HandleTypeDef;

#define OFFSET_X 150
#define OFFSET_Y 150

void Encoder_Init(Encoder_HandleTypeDef *encoder, TIM_HandleTypeDef *htim, uint32_t pulse_divider, uint32_t pulses_per_rev, float wheel_diameter);
int32_t Encoder_GetRawCount(Encoder_HandleTypeDef *encoder);
int32_t Encoder_GetEffectiveCount(Encoder_HandleTypeDef *encoder);
void Encoder_Reset(Encoder_HandleTypeDef *encoder, float *global_val);
float Encoder_GetDistance(Encoder_HandleTypeDef *encoder);

// Hàm cập nhật odometry sử dụng 2 encoder (cho trục X và Y)
// Tham số robot_angle tính bằng radian.
// void Encoder_UpdateOdometry(Encoder_HandleTypeDef *encoder_x, Encoder_HandleTypeDef *encoder_y, float robot_angle, float *global_x, float *global_y);
void Encoder_UpdateOdometry(Encoder_HandleTypeDef *encoder_x,
                           Encoder_HandleTypeDef *encoder_y,
                           float imu_angle, // Góc từ IMU (đơn vị độ)
                           float *global_x,
                           float *global_y);
#endif // ENCODER_H
