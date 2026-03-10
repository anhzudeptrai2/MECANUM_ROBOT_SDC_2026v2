#include "MECANUM_FIELD_KIN.h"

/* PID tuning for yaw-hold (omega control) */
double Mecanum_Omega_PID_Out;
static double Mecanum_Kp_Omega = 0.90f;
static double Mecanum_Ki_Omega = 0.0020f;
static double Mecanum_Kd_Omega = 0.024f;
static double Mecanum_Speed_Omega_PID = 0.78f;
#define ANGLE_DEADBAND 5.0f // Deadband 5 độ để tránh nhiễu IMU

extern PID_TypeDef Mecanum_Omega_PID;
extern MRb Mecanum_4_Bot;

typedef struct
{
    float x_axis; // (-127 -> 127)
    float y_axis; // (-127 -> 127)
    float z_axis; // (-127 -> 127)
} JoystickAxes;

static JoystickAxes Joystick;

static float rads_2_rpm(float rads_in)
{
    return (rads_in * 60.0f) / (2.0f * PI);
}

static float rpm_2_rads(float rpm_in)
{
    return (rpm_in * 2.0f * PI) / 60.0f;
}

static float normalize_angle_rad(float angle)
{
    while (angle > PI)
    {
        angle -= 2.0f * PI;
    }
    while (angle < -PI)
    {
        angle += 2.0f * PI;
    }
    return angle;
}

static float normalize_angle_deg(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

static void Joystick_To_Velocity(MRb *robot, float max_speed, float max_omega)
{
    /* Deadband on normalized joystick [-1..1] */
    const float Dead_Point = 0.06f;

    float mapped_x = (Joystick.x_axis) / 127.0f;
    float mapped_y = (Joystick.y_axis) / 127.0f;
    float mapped_z = (Joystick.z_axis) / 127.0f;

    if (fabsf(mapped_x) < Dead_Point)

        mapped_x = 0.0f;
    if (fabsf(mapped_y) < Dead_Point)
        mapped_y = 0.0f;
    if (fabsf(mapped_z) < Dead_Point)
        mapped_z = 0.0f;

    robot->vx = mapped_x * max_speed;
    robot->vy = mapped_y * max_speed;

    if (robot->is_yaw_fix)
    {
        float shortest_err_deg = normalize_angle_deg((float)(robot->fix_angle - robot->IMU_theta));
        robot->fix_angle = robot->IMU_theta + shortest_err_deg;

        if (fabsf(shortest_err_deg) < ANGLE_DEADBAND)
        {
            robot->omega = 0.0f;
            return;
        }

        PID_Compute(&Mecanum_Omega_PID);
        robot->omega = (float)Mecanum_Omega_PID_Out;
    }
    else
    {
        robot->omega = mapped_z * max_omega;
    }
}

void MecanumRobot_Init(MRb *robot, float m_speed, float m_omg)
{
    robot->x = 0.0f;
    robot->y = 0.0f;
    robot->vx = 0.0f;
    robot->vy = 0.0f;
    robot->omega = 0.0f;
    robot->theta = 0.0f;
    robot->max_speed = m_speed;
    robot->max_omega = m_omg;

    robot->u[0] = 0.0f;
    robot->u[1] = 0.0f;
    robot->u[2] = 0.0f;
    robot->u[3] = 0.0f;

    robot->is_yaw_fix = 0;
    robot->fix_angle = 0;
    robot->IMU_theta = 0;

    /* PID Init for Omega control to fix robot angle */
    PID(&Mecanum_Omega_PID,
        &robot->IMU_theta,
        &Mecanum_Omega_PID_Out,
        &robot->fix_angle,
        Mecanum_Kp_Omega,
        Mecanum_Ki_Omega,
        Mecanum_Kd_Omega,
        _PID_P_ON_E,
        _PID_CD_DIRECT); // Đổi từ REVERSE sang DIRECT

    PID_SetMode(&Mecanum_Omega_PID, _PID_MODE_AUTOMATIC);
    PID_SetSampleTime(&Mecanum_Omega_PID, 1);
    PID_SetOutputLimits(&Mecanum_Omega_PID, -Mecanum_Speed_Omega_PID, Mecanum_Speed_Omega_PID);
}

void MecanumRobot_ForwardKinematicsFromRPM(float u_fl_rpm, float u_fr_rpm, float u_rl_rpm, float u_rr_rpm,
                                           float *vx_robot, float *vy_robot, float *omega_robot)
{
    float w_fl = rpm_2_rads(u_fl_rpm);
    float w_fr = rpm_2_rads(u_fr_rpm);
    float w_rl = rpm_2_rads(u_rl_rpm);
    float w_rr = rpm_2_rads(u_rr_rpm);

    *vx_robot = (MECANUM_WHEEL_RADIUS / 4.0f) * (w_fl + w_fr + w_rl + w_rr);
    *vy_robot = (MECANUM_WHEEL_RADIUS / 4.0f) * (-w_fl + w_fr + w_rl - w_rr);
    *omega_robot = (MECANUM_WHEEL_RADIUS / (4.0f * MECANUM_L)) * (-w_fl + w_fr - w_rl + w_rr);
}

void MecanumRobot_UpdatePose(MRb *robot, float dt_s)
{
    float vx_robot;
    float vy_robot;
    float omega_robot;

    MecanumRobot_ForwardKinematicsFromRPM(robot->u[0], robot->u[1], robot->u[2], robot->u[3],
                                          &vx_robot, &vy_robot, &omega_robot);

    {
        float cos_theta = cosf(robot->theta);
        float sin_theta = sinf(robot->theta);
        float vx_field = vx_robot * cos_theta - vy_robot * sin_theta;
        float vy_field = vx_robot * sin_theta + vy_robot * cos_theta;

        robot->x += vx_field * dt_s;
        robot->y += vy_field * dt_s;
    }

    robot->theta = normalize_angle_rad(robot->theta + omega_robot * dt_s);
}

void MecanumRobot_CalculateWheelSpeeds(MRb *robot, float *u_fl, float *u_fr, float *u_rl, float *u_rr)
{
    /* Field-centric to robot-centric transformation */
    float vx_robot = robot->vx * cosf(robot->theta) + robot->vy * sinf(robot->theta);
    float vy_robot = -robot->vx * sinf(robot->theta) + robot->vy * cosf(robot->theta);

    /* Mecanum inverse kinematics (X-configuration)
     * Standard formulas from mecanum kinematics theory:
     * V_FL = Vx - Vy - omega * L
     * V_FR = Vx + Vy + omega * L
     * V_RL = Vx + Vy - omega * L
     * V_RR = Vx - Vy + omega * L
     * where L = Lx + Ly
     */
    float w_fl = (vx_robot - vy_robot - MECANUM_L * robot->omega) / MECANUM_WHEEL_RADIUS;
    float w_fr = (vx_robot + vy_robot + MECANUM_L * robot->omega) / MECANUM_WHEEL_RADIUS;
    float w_rl = (vx_robot + vy_robot - MECANUM_L * robot->omega) / MECANUM_WHEEL_RADIUS;
    float w_rr = (vx_robot - vy_robot + MECANUM_L * robot->omega) / MECANUM_WHEEL_RADIUS;

    *u_fl = rads_2_rpm(w_fl);
    *u_fr = rads_2_rpm(w_fr);
    *u_rl = rads_2_rpm(w_rl);
    *u_rr = rads_2_rpm(w_rr);
}

void MecanumRobot_SetMotion(MRb *robot, float vx, float vy, float omega, float imu_theta_deg, uint8_t neg_heading)
{
    robot->vx = vx;
    robot->vy = vy;
    robot->omega = omega;

    if (neg_heading)
    {
        robot->theta = (180.0f + imu_theta_deg) * (PI / 180.0f);
    }
    else
    {
        robot->theta = imu_theta_deg * (PI / 180.0f);
    }

    if (robot->theta > 2.0f * PI)
    {
        robot->theta -= 2.0f * PI;
    }
    else if (robot->theta < 0.0f)
    {
        robot->theta += 2.0f * PI;
    }

    robot->theta = -robot->theta;

    MecanumRobot_CalculateWheelSpeeds(robot, &robot->u[0], &robot->u[1], &robot->u[2], &robot->u[3]);
}

void MecanumRobot_Field_Control(MRb *robot, PS4_DATA *ps4_joy, float imu_theta, uint8_t neg_heading)
{
    robot->IMU_theta = -imu_theta;

    /* Convention used by mecanum inverse kinematics below:
     * vx: forward (+)
     * vy: left (+)
     * omega: CCW (+)
     */
    Joystick.x_axis = -ps4_joy->l_stick_y; /* forward/back on left stick */
    Joystick.y_axis = ps4_joy->l_stick_x;  /* strafe on left stick */
    Joystick.z_axis = -ps4_joy->r_stick_x; /* yaw on right stick */

    Joystick_To_Velocity(robot, robot->max_speed, robot->max_omega);

    if (neg_heading)
    {
        robot->theta = (180.0f + imu_theta) * (PI / 180.0f);
    }
    else
    {
        robot->theta = imu_theta * (PI / 180.0f);
    }

    if (robot->theta > 2.0f * PI)
    {
        robot->theta -= 2.0f * PI;
    }
    else if (robot->theta < 0.0f)
    {
        robot->theta += 2.0f * PI;
    }

    robot->theta = -robot->theta;

    MecanumRobot_CalculateWheelSpeeds(robot, &robot->u[0], &robot->u[1], &robot->u[2], &robot->u[3]);
}
