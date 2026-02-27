#ifndef ROBOT_ACCELERATION_H
#define ROBOT_ACCELERATION_H

// =========================
// Cấu trúc dữ liệu RobotAcceleration
// =========================
typedef struct
{
    float currentVelocity; // Vận tốc hiện tại
    float targetVelocity;  // Vận tốc mục tiêu
    float previousTarget;  // Lưu vận tốc mục tiêu trước đó (hỗ trợ thay đổi động)
    float duration;        // Thời gian chuyển đổi (n giây)
    float tau;             // Hằng số thời gian
    float timeElapsed;     // Thời gian đã trôi qua
} RobotAcceleration;

// =========================
// Hàm khởi tạo
// =========================
void RobotAcceleration_Init(RobotAcceleration *robot, float initialTarget, float duration);

// =========================
// Cập nhật vận tốc mục tiêu
// =========================
void RobotAcceleration_SetTarget(RobotAcceleration *robot, float newTargetVelocity, float newDuration);

// =========================
// Tính toán vận tốc hiện tại
// =========================
float RobotAcceleration_Update(RobotAcceleration *robot, float dt);

#endif // ROBOT_ACCELERATION_H
