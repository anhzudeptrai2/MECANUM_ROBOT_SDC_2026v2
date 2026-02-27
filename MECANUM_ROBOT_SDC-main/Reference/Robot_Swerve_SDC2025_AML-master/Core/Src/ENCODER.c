#include "encoder.h"

void Encoder_Init(Encoder_HandleTypeDef *encoder, TIM_HandleTypeDef *htim,
                  uint32_t pulse_divider, uint32_t pulses_per_rev, float wheel_diameter)
{
    encoder->htim = htim;
    encoder->pulse_divider = pulse_divider;
    encoder->pulses_per_rev = pulses_per_rev;
    encoder->wheel_diameter = wheel_diameter;
    encoder->max_value = encoder->htim->Instance->ARR;
    uint32_t ticks_per_rev = encoder->max_value + 1;
    encoder->last_raw = 0;
    encoder->overflow = 0;

    if (HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL) != HAL_OK)
    {
        // Xử lý lỗi nếu cần
    }
    __HAL_TIM_SET_COUNTER(encoder->htim, 0);
}

int32_t Encoder_GetRawCount(Encoder_HandleTypeDef *encoder)
{
    uint32_t ticks_per_rev = encoder->max_value + 1;
    int32_t current_raw = (int32_t)__HAL_TIM_GET_COUNTER(encoder->htim);
    int32_t diff = current_raw - encoder->last_raw;
    int32_t half_range = ticks_per_rev / 2;
    if (diff > (int32_t)half_range)
    {
        encoder->overflow--;
    }
    else if (diff < -(int32_t)half_range)
    {
        encoder->overflow++;
    }

    encoder->last_raw = current_raw;
    return current_raw + encoder->overflow * ticks_per_rev;
}

int32_t Encoder_GetEffectiveCount(Encoder_HandleTypeDef *encoder)
{
    int32_t extended_raw = Encoder_GetRawCount(encoder);
    return extended_raw / encoder->pulse_divider;
}

float Encoder_GetDistance(Encoder_HandleTypeDef *encoder)
{
    int32_t effective_count = Encoder_GetEffectiveCount(encoder);
    float mm_per_count = (M_PI * encoder->wheel_diameter * encoder->pulse_divider) / encoder->pulses_per_rev;
    return effective_count * mm_per_count;
}
// static int32_t prev_enc_x = 0, prev_enc_y = 0; // Lưu số đếm của lần cập nhật trước
// static float prev_robot_angle = 0.0f;
// void Encoder_UpdateOdometry(Encoder_HandleTypeDef *encoder_x,
//                             Encoder_HandleTypeDef *encoder_y,
//                             float imu_angle, // Góc từ IMU (đơn vị độ)
//                             float *global_x,
//                             float *global_y)
// {
//     // Chuyển góc IMU từ độ sang radian
//     float robot_angle = imu_angle * M_PI / 180.0f;
//     prev_robot_angle = robot_angle;
//     float dtheta = robot_angle - prev_robot_angle; // Thay đổi góc (radian)

//     // Lấy giá trị raw từ encoder
//     int32_t raw_x = Encoder_GetRawCount(encoder_x);
//     int32_t raw_y = Encoder_GetRawCount(encoder_y);

//     // Tính delta số đếm từ encoder
//     int32_t delta_x_counts = raw_x - prev_enc_x;
//     int32_t delta_y_counts = raw_y - prev_enc_y;

//     // Quy đổi số đếm sang khoảng cách (mm)
//     float dist_per_count_x = (M_PI * encoder_x->wheel_diameter * encoder_x->pulse_divider) / encoder_x->pulses_per_rev;
//     float dist_per_count_y = (M_PI * encoder_y->wheel_diameter * encoder_y->pulse_divider) / encoder_y->pulses_per_rev;

//     float local_dx = delta_x_counts * dist_per_count_x;
//     float local_dy = delta_y_counts * dist_per_count_y;

//     // Offset của sensor so với tâm robot (mm)
//     float offset_x = 50.0f; // theo trục x
//     float offset_y = 250.0f; // theo trục y

//     // Tính thành phần chuyển động do quay của sensor
//     // Khi robot quay một góc dtheta, sensor dịch chuyển thêm:
//     float dx_rot = -offset_y * dtheta; // chuyển động do quay trên trục x
//     float dy_rot = offset_x * dtheta;  // chuyển động do quay trên trục y

//     // Hiệu chỉnh số đo của encoder: loại bỏ chuyển động do quay
//     float corrected_local_dx = local_dx - dx_rot;
//     float corrected_local_dy = local_dy - dy_rot;

//     // Khi robot quay tại chỗ (không dịch chuyển), các số đo từ encoder do chuyển động tịnh tiến sẽ gần như bằng 0 sau hiệu chỉnh.
//     // Nếu robot chỉ quay tại chỗ, corrected_local_dx và corrected_local_dy sẽ rất nhỏ (hoặc = 0).

//     // Biến đổi chuyển động từ hệ tọa độ của robot sang hệ tọa độ toàn cục
//     float global_dx = corrected_local_dx * cosf(robot_angle) - corrected_local_dy * sinf(robot_angle);
//     float global_dy = corrected_local_dx * sinf(robot_angle) + corrected_local_dy * cosf(robot_angle);

//     // Cập nhật vị trí toàn cục
//     *global_x += global_dx;
//     *global_y += global_dy;

//     // Cập nhật giá trị cho lần cập nhật sau
//     prev_enc_x = raw_x;
//     prev_enc_y = raw_y;
//     prev_robot_angle = robot_angle;
// }
static int32_t prev_enc_x = 0, prev_enc_y = 0, prev_robot_angle = 0.0f;
// Hàm cập nhật odometry: tính khoảng cách di chuyển theo hệ tọa độ toàn cục
// sử dụng giá trị của 2 encoder (encoder_x, encoder_y) và góc robot (robot_angle, đơn vị radian).
void Encoder_UpdateOdometry(Encoder_HandleTypeDef *encoder_x, Encoder_HandleTypeDef *encoder_y, float robot_angle, float *global_x, float *global_y)
{
    robot_angle = robot_angle / 180.0f * M_PI;
    // Đọc giá trị raw từ encoder
    int32_t raw_x = Encoder_GetRawCount(encoder_x);
    int32_t raw_y = Encoder_GetRawCount(encoder_y);

    // Tính delta counts
    int32_t delta_x_counts = raw_x - prev_enc_x;
    int32_t delta_y_counts = raw_y - prev_enc_y;

    // Chuyển đổi từ số đếm sang khoảng cách (mm hoặc đơn vị bạn sử dụng)
    float dist_per_count_x = (M_PI * encoder_x->wheel_diameter * encoder_x->pulse_divider) / encoder_x->pulses_per_rev;
    float dist_per_count_y = (M_PI * encoder_y->wheel_diameter * encoder_y->pulse_divider) / encoder_y->pulses_per_rev;

    float local_dx = delta_x_counts * dist_per_count_x;
    float local_dy = delta_y_counts * dist_per_count_y;

    // Biến đổi từ hệ tọa độ robot sang hệ tọa độ toàn cục bằng ma trận quay
    float global_dx = local_dx * cosf(robot_angle) - local_dy * sinf(robot_angle);
    float global_dy = local_dx * sinf(robot_angle) + local_dy * cosf(robot_angle);

    // Cập nhật vị trí toàn cục
    *global_x += global_dx;
    *global_y += global_dy;

    // Lưu lại giá trị encoder cho lần cập nhật tiếp theo
    prev_enc_x = raw_x;
    prev_enc_y = raw_y;
}

void Encoder_Reset(Encoder_HandleTypeDef *encoder, float *global_val)
{
    __HAL_TIM_SET_COUNTER(encoder->htim, 0);
    encoder->last_raw = 0;
    encoder->overflow = 0;
    prev_enc_x = 0;
    prev_enc_y = 0;
    *global_val = 0.0f;
}