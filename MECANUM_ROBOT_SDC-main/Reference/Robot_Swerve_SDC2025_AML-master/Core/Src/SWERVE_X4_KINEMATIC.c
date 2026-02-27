#include "SWERVE_X4_KINEMATIC.h"

// PID cho omega
double Omega_PID_Out;
double Kp_Omega = 0.1f, Ki_Omega = 0.00001f, Kd_Omega = 0.0003f;
double Speed_Omega_PID = 0.8f;
// PID Vx
double Vx_PID_Out;
double Kp_Vx = 0.008f, Ki_Vx = 0.0f, Kd_Vx = 0.0000001f;
double Speed_Vx_PID = 1.8f;
// PID Vy
double Vy_PID_Out;
double Kp_Vy = 0.008f, Ki_Vy = 0.0f, Kd_Vy = 0.0000001f;
double Speed_Vy_PID = 1.8f;
struct
{
    float x_axis; // (-1.0 -> 1.0)
    float y_axis; // (-1.0 -> 1.0)
    float z_axis; // (-1.0 -> 1.0)
} Joystick;

float Rad_2_rpm(float rad_per_sec)
{
    return rad_per_sec * (60.0f / (2.0f * PI));
}

// Hàm trợ giúp: Tính hiệu lệch góc (delta) theo góc liên tục
static inline float angleDifference(float target, float current)
{
    float diff = target - current;
    while (diff > 180.0f)
        diff -= 360.0f;
    while (diff < -180.0f)
        diff += 360.0f;
    return diff;
}
/*
 * Hàm OptimizeSteeringAngle:
 * Dựa trên góc phản hồi thực tế (current_angle) từ steering, tính hiệu lệch góc giữa
 * cách biểu diễn ban đầu và cách biểu diễn với cộng thêm 180°, chọn cách cho hiệu chỉnh nhỏ nhất.
 * Nếu sự khác biệt giữa 2 cách chỉ nhỏ hơn một ngưỡng (flip_threshold), giữ nguyên cách ban đầu.
 */
void OptimizeSteeringAngle(SwerveModule *module, float current_angle)
{
    // Góc mục tiêu tính từ động học (không ép về khoảng [-180, 180])
    float target_angle = module->angle;
    // Tính hiệu lệch giữa góc mục tiêu và góc hiện tại (giá trị liên tục)
    float delta1 = angleDifference(target_angle, current_angle);

    // Option 2: cộng thêm 180° và đảo chiều tốc độ
    float alt_angle = target_angle + 180.0f;
    float delta2 = angleDifference(alt_angle, current_angle);

    // Đặt ngưỡng chuyển đổi để tránh flip khi sự chênh lệch chỉ nhỏ
    const float flip_threshold = 5.0f; // đơn vị: độ

    // Nếu hiệu chỉnh theo cách thứ 2 cải thiện rõ rệt và khác nhau hơn ngưỡng, thì flip
    if ((fabsf(fabsf(delta2) - fabsf(delta1)) > flip_threshold) && (fabsf(delta2) < fabsf(delta1)))
    {
        module->angle = current_angle + delta2;
        module->speed = -module->speed;
    }
    else
    {
        module->angle = current_angle + delta1;
    }
}

/*
 * Hàm OptimizeSteeringAngle_Pure:
 * Áp dụng cho trường hợp chỉ xoay (omega khác 0, vx, vy gần 0).
 * Tương tự, sử dụng góc phản hồi (current_angle) để tính hiệu chỉnh và quyết định flip.
 */
void OptimizeSteeringAngle_Pure(SwerveModule *module, float current_angle)
{
    float target_angle = module->angle;
    float delta1 = angleDifference(target_angle, current_angle);

    float alt_angle = target_angle + 180.0f;
    float delta2 = angleDifference(alt_angle, current_angle);

    const float flip_threshold = 5.0f;

    if ((fabsf(fabsf(delta2) - fabsf(delta1)) > flip_threshold) && (fabsf(delta2) < fabsf(delta1)))
    {
        module->angle = current_angle + delta2;
        module->speed = -module->speed;
    }
    else
    {
        module->angle = current_angle + delta1;
    }
}

/*
 * Hàm Swerve_4X_Init khởi tạo robot, cấu hình PID cho omega và thiết lập các góc mặc định của module.
 * Cấu hình mặc định theo dạng omni 4 bánh:
 *    - Module 0 và Module 2: +45°
 *    - Module 1 và Module 3: –45°
 */
void Swerve_4X_Init(SRb *robot, float max_speed, float max_omega)
{
    robot->vx = 0.0f;
    robot->vy = 0.0f;
    robot->omega = 0.0f;
    robot->theta = 0.0f;
    robot->max_speed = max_speed;
    robot->max_omega = max_omega;
    robot->Is_Yaw_Fix = 0;
    robot->fix_angle = 0;
    robot->Is_Fiels_Control = 1;
    robot->Play_Field = 0;
    // Khởi tạo PID cho omega
    PID(&Omega_PID, &robot->IMU_theta, &Omega_PID_Out, &robot->fix_angle,
        Kp_Omega, Ki_Omega, Kd_Omega, _PID_P_ON_E, _PID_CD_DIRECT);
    PID_SetMode(&Omega_PID, _PID_MODE_AUTOMATIC);
    PID_SetSampleTime(&Omega_PID, 1);
    PID_SetOutputLimits(&Omega_PID, -Speed_Omega_PID, Speed_Omega_PID);
    // Khởi tạo PID cho vx
    PID(&Vx_PID, &robot->position_x, &Vx_PID_Out, &robot->setpoint_x,
        Kp_Vx, Ki_Vx, Kd_Vx, _PID_P_ON_E, _PID_CD_REVERSE);
    PID_SetMode(&Vx_PID, _PID_MODE_AUTOMATIC);
    PID_SetSampleTime(&Vx_PID, 1);
    PID_SetOutputLimits(&Vx_PID, -Speed_Vx_PID, Speed_Vx_PID);
    // Khởi tạo PID cho vy
    PID(&Vy_PID, &robot->position_y, &Vy_PID_Out, &robot->setpoint_y,
        Kp_Vy, Ki_Vy, Kd_Vy, _PID_P_ON_E, _PID_CD_REVERSE);
    PID_SetMode(&Vy_PID, _PID_MODE_AUTOMATIC);
    PID_SetSampleTime(&Vy_PID, 1);
    PID_SetOutputLimits(&Vy_PID, -Speed_Vy_PID, Speed_Vy_PID);

    // Thiết lập góc mặc định cho các module theo cấu hình omni 4 bánh
    robot->steering_angles[0] = 45.0f;  // Front-left
    robot->steering_angles[1] = 135.0f; // Rear-left
    robot->steering_angles[2] = 45.0f;  // Rear-right
    robot->steering_angles[3] = 135.0f; // Front-right
    for (int i = 0; i < 4; i++)
    {
        robot->u[i] = 0.0f;
    }
}

/*
 * Hàm chuyển đổi tín hiệu joystick sang vận tốc robot.
 */
/*
 * Hàm chuyển đổi tín hiệu joystick sang vận tốc robot.
 * Nếu một trong 2 trục (vx, vy) có giá trị nhỏ hơn đáng kể so với trục kia,
 * ta đặt giá trị của trục đó bằng 0 nhằm giảm nhiễu và xử lý trường hợp điều khiển lệch.
 */
void Joystick_To_Velocity(SRb *robot, float max_speed, float max_omega)
{
    if (!robot->Is_Auto)
    {
        float mapped_x = (float)Joystick.x_axis / 128.0f; // trực tiếp từ l_stick_x
        float mapped_y = (float)Joystick.y_axis / 128.0f; // trực tiếp từ l_stick_y
        float mapped_z = (float)Joystick.z_axis / 128.0f; // trực tiếp từ r_stick_x

        // Áp dụng ngưỡng lọc nếu cần (như cũ)
        float abs_x = fabsf(mapped_x);
        float abs_y = fabsf(mapped_y);
        float threshold_ratio = 0.15f;
        if (abs_y > 0 && abs_x < threshold_ratio * abs_y)
            mapped_x = 0;
        if (abs_x > 0 && abs_y < threshold_ratio * abs_x)
            mapped_y = 0;

        // Điều chỉnh vx, vy và ω theo ý nghĩa điều khiển:
        // - Khi l_stick_y âm (kéo lên) => -mapped_y dương -> tiến
        // - Khi l_stick_x dương => robot dịch sang phải
        robot->vx = -mapped_y * max_speed;
        robot->vy = mapped_x * max_speed;
        // Nếu khi kéo cần quay phải (mapped_z dương) mà robot quay ngược, đảo dấu:
        robot->omega = -mapped_z * max_omega;
    }
    else
    {
        PID_Compute(&Omega_PID);
        robot->omega = Omega_PID_Out;
        PID_Compute(&Vx_PID);
        robot->vy = Vx_PID_Out;
        PID_Compute(&Vy_PID);
        robot->vx = Vy_PID_Out;
    }

    if (robot->Is_Yaw_Fix)
    {
        PID_Compute(&Omega_PID);
        robot->omega = Omega_PID_Out;
    }
}

/*
 * Hàm Swerve_4X_CalculateModules:
 * Tính động học ngược cho 4 module.
 * Lưu ý: Ở các trường hợp, thay vì ép góc về [-180, 180] cứng nhắc, ta dùng giá trị
 * robot->steering_angles[i] làm góc phản hồi thực tế (continuous angle) để đảm bảo tính liên tục.
 */
void Swerve_4X_CalculateModules(SRb *robot, SwerveModule modules[4])
{
    float wheel_positions[4][2] = {
        {D, -D},  // Module 0: Front-left
        {-D, -D}, // Module 1: Back-left
        {-D, D},  // Module 2: Back-right
        {D, D}    // Module 3: Front-right
    };

    // Trường hợp robot đứng yên hoàn toàn
    if (fabsf(robot->vx) < 0.001f && fabsf(robot->vy) < 0.001f && fabsf(robot->omega) < 0.001f)
    {
        float default_chassis_angles[4] = {45.0f, 135.0f, 45.0f, 135.0f};
        for (int i = 0; i < 4; i++)
        {
            modules[i].speed = 0.0f;
            modules[i].angle = default_chassis_angles[i];
            // Dùng giá trị góc phản hồi thực tế từ robot->steering_angles[i]
            OptimizeSteeringAngle(&modules[i], robot->steering_angles[i]);
            robot->steering_angles[i] = modules[i].angle;
            robot->u[i] = 0.0f;
        }
        return;
    }

    // Trường hợp robot chỉ có omega (xoay thuần túy)
    if (fabsf(robot->vx) < 0.001f && fabsf(robot->vy) < 0.001f && fabsf(robot->omega) >= 0.001f)
    {
        for (int i = 0; i < 4; i++)
        {
            float wx = -robot->omega * wheel_positions[i][1];
            float wy = robot->omega * wheel_positions[i][0];

            modules[i].speed = sqrtf(wx * wx + wy * wy) / WHEEL_RADIUS;
            modules[i].speed = Rad_2_rpm(modules[i].speed);
            modules[i].angle = atan2f(wy, wx) * (180.0f / PI);
            // Sử dụng góc phản hồi thực tế từ robot->steering_angles[i]
            OptimizeSteeringAngle_Pure(&modules[i], robot->steering_angles[i]);

            robot->steering_angles[i] = modules[i].angle;
            robot->u[i] = modules[i].speed;
        }
        return;
    }

    // Các trường hợp còn lại: có vận tốc truyền động (vx, vy)
    float vx_robot = robot->vx * cosf(robot->theta) + robot->vy * sinf(robot->theta);
    float vy_robot = -robot->vx * sinf(robot->theta) + robot->vy * cosf(robot->theta);
    for (int i = 0; i < 4; i++)
    {
        float wx = vx_robot - robot->omega * wheel_positions[i][1];
        float wy = vy_robot + robot->omega * wheel_positions[i][0];

        modules[i].speed = sqrtf(wx * wx + wy * wy) / WHEEL_RADIUS;
        modules[i].speed = Rad_2_rpm(modules[i].speed);
        modules[i].angle = atan2f(wy, wx) * (180.0f / PI);

        // Sử dụng giá trị góc phản hồi thực tế từ robot->steering_angles[i]
        OptimizeSteeringAngle(&modules[i], robot->steering_angles[i]);

        robot->steering_angles[i] = modules[i].angle;
        robot->u[i] = modules[i].speed;
    }
}
/*
 * Hàm Swerve_4X_Field_Control xử lý tín hiệu điều khiển từ thiết bị (ví dụ PS4),
 * cập nhật trạng thái robot (vận tốc, góc theta, IMU,...) và tính toán động học ngược cho các module.
 */
void Swerve_4X_Field_Control(SRb *robot, PS4_DATA *ps4_joy, float imu_theta, uint8_t neg_heading, double feedback_pos_x, double feedback_pos_y)
{
    robot->position_x = feedback_pos_x;
    robot->position_y = feedback_pos_y;
    robot->IMU_theta = imu_theta;
    // Gán trực tiếp theo ánh xạ mong muốn
    if (robot->Play_Field)
    {
        Joystick.x_axis = ps4_joy->l_stick_y;  // Không đảo dấu
        Joystick.y_axis = -ps4_joy->l_stick_x; // đảo dấu
        Joystick.z_axis = ps4_joy->r_stick_x;  // Không đảo dấu
    }
    else
    {
        Joystick.x_axis = -ps4_joy->l_stick_y;  // Không đảo dấu
        Joystick.y_axis = ps4_joy->l_stick_x; // Không đảo dấu
        Joystick.z_axis = ps4_joy->r_stick_x;  // Không đảo dấu
    }

    // Gọi hàm chuyển đổi với mapping mới
    Joystick_To_Velocity(robot, robot->max_speed, robot->max_omega);

    if (neg_heading)
        robot->theta = (180.0f + imu_theta) * (PI / 180.0f);
    else
        robot->theta = imu_theta * (PI / 180.0f);

    robot->theta = fmodf(robot->theta, 2 * PI);
    if (robot->theta < 0)
        robot->theta += 2 * PI;

    SwerveModule modules[4];
    Swerve_4X_CalculateModules(robot, modules);
}

void Reset_Steering_Angles(SRb *robot)
{
    // Đặt lại góc ban đầu cho robot
    robot->steering_angles[0] = 45.0f;  // Front-left
    robot->steering_angles[1] = 135.0f; // Rear-left
    robot->steering_angles[2] = 45.0f;  // Rear-right
    robot->steering_angles[3] = 135.0f; // Front-right
    // Nếu có các giá trị điều khiển module, reset chúng về 0
    for (int i = 0; i < 4; i++)
    {
        robot->u[i] = 0.0f;
    }
}
