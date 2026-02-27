#ifndef SWERVE_X4_KIN_H
#define SWERVE_X4_KIN_H

#include "stdint.h"
#include "math.h"
#include "WT901C.h"
#include "PS4_ESP.h"
#include "PID.h"

#define PI 3.14159f
#define WHEEL_RADIUS 0.05f   // Bán kính bánh robot
#define D 0.2f               // Khoảng cách tâm robot đến bánh
#define Robot_Max_Speed 5.0f // Vận tốc tịnh tiến robot max (m/s)
#define Robot_Max_Omega 5.0f // Vận tốc góc robot max (rad/s)

typedef struct
{
    float vx;                 // V tịnh tiến x (m/s)
    float vy;                 // V tịnh tiến y (m/s)
    float omega;              // Vận tốc góc (rad/s)
    float max_speed;          
    float max_omega;       
    float u[4];               // Vận tốc điều khiển bánh
    float steering_angles[4]; // Góc sterring bánh (degrees)
    uint8_t Is_Yaw_Fix;       // Check bám IMU hay không
    uint8_t Is_Auto;
    uint8_t Is_Fiels_Control;
    double IMU_theta;         // Góc IMU hiện tại
    double fix_angle;         // Góc set để robot xoay so với gốc 0
    float theta;              // Góc robot so với gốc 0 (rad)
    double setpoint_x;        // Vị trí setpoint x
    double position_x;        // Vị trí hiện tại x
    double setpoint_y;        // Vị trí setpoint y
    double position_y;        // Vị trí hiện tại y
    uint8_t Play_Field;
} SRb;

typedef struct
{
    float speed;
    float angle;
} SwerveModule;

extern SRb Swerve_4X_Bot;

extern PID_TypeDef Omega_PID, Vx_PID, Vy_PID;

void Swerve_4X_Init(SRb *robot, float m_speed, float m_omega);
void Joystick_To_Velocity(SRb *robot, float max_speed, float max_omega);
void Swerve_4X_CalculateModules(SRb *robot, SwerveModule modules[4]);
void Swerve_4X_Field_Control(SRb *robot, PS4_DATA *ps4_joy, float imu_theta, uint8_t neg_heading, double feedback_pos_x, double feedback_pos_y);
void Reset_Steering_Angles(SRb *robot);
#endif // SWERVE_X4_KIN_H
