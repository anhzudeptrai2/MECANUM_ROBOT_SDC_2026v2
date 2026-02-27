/*
 * Mecanum 4-wheel kinematics (field-centric)
 *
 * Wheel layout (top view):
 *   Front
 *   [3] FL ----- FR [4]
 *        |     |
 *   [1] RL ----- RR [2]
 *   Rear
 *
 * Notes:
 * - Outputs are wheel speeds in RPM (same style as OMNI_3_FIELD_KIN).
 * - Field-centric: (vx, vy) are rotated by robot heading theta.
 */
#ifndef MECANUM_FIELD_KIN_H
#define MECANUM_FIELD_KIN_H

#include "stdint.h"
#include "math.h"
#include "PS4_ESP.h"
#include "PID.h"

#ifndef PI
#define PI 3.14159f
#endif

/* ---- Robot geometry / tuning ----
 * MECANUM_LX: half-length (center to wheel in x) [m]
 * MECANUM_LY: half-width  (center to wheel in y) [m]
 */
#define MECANUM_WHEEL_RADIUS 0.04f
#define MECANUM_LX 0.175f
#define MECANUM_LY 0.17f
#define Robot_Max_Speed 1.00f // m/s
#define Robot_Max_Omega 0.5f  // rad/s

#define MECANUM_L (MECANUM_LX + MECANUM_LY)

typedef struct
{
    float vx;    // Linear speed X (m/s)
    float vy;    // Linear speed Y (m/s)
    float omega; // Angular speed (rad/s)
    float theta; // Robot heading (rad)

    float max_speed;
    float max_omega;

    float u[4];         // Wheel speeds (RPM): FL, FR, RL, RR
    uint8_t is_yaw_fix; // 1: use omega PID to hold heading

    double fix_angle; // Setpoint angle for yaw fix
    double IMU_theta; // Current angle feedback
} MRb;

typedef MRb Mecanum_Robot;

extern MRb Mecanum_4_Bot;
extern PID_TypeDef Mecanum_Omega_PID;
extern double Mecanum_Omega_PID_Out; // PID output cho omega control

void MecanumRobot_Init(MRb *robot, float m_speed, float m_omg);
void MecanumRobot_Field_Control(MRb *robot, PS4_DATA *ps4_joy, float imu_theta, uint8_t neg_heading);
void MecanumRobot_CalculateWheelSpeeds(MRb *robot, float *u_fl, float *u_fr, float *u_rl, float *u_rr);
void MecanumRobot_SetMotion(MRb *robot, float vx, float vy, float omega, float imu_theta_deg, uint8_t neg_heading);

#endif /* MECANUM_4_FIELD_KIN_H */
