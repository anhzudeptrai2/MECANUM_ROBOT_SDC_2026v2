#include "ROBOT_ACCELERATION.h"
#include <math.h> // Để sử dụng hàm exp()

// =========================
// Khởi tạo cấu trúc RobotAcceleration
// =========================
void RobotAcceleration_Init(RobotAcceleration *robot, float initialTarget, float duration)
{
    robot->currentVelocity = 0.0f;         // Vận tốc ban đầu là 0
    robot->targetVelocity = initialTarget; // Gán vận tốc mục tiêu ban đầu
    robot->previousTarget = initialTarget; // Khởi tạo giá trị mục tiêu trước đó
    robot->duration = duration;            // Thời gian chuyển đổi
    robot->tau = duration / 5.0f;          // Hằng số thời gian tau = duration / 5
    robot->timeElapsed = 0.0f;             // Thời gian ban đầu = 0
}

// =========================
// Cập nhật vận tốc mục tiêu và thời gian
// =========================
void RobotAcceleration_SetTarget(RobotAcceleration *robot, float newTargetVelocity, float newDuration)
{
    robot->previousTarget = robot->currentVelocity; // Lưu lại vận tốc hiện tại làm mục tiêu trước đó
    robot->targetVelocity = newTargetVelocity;      // Cập nhật vận tốc mục tiêu mới
    robot->duration = newDuration;                  // Cập nhật thời gian chuyển đổi mới
    robot->tau = newDuration / 5.0f;                // Tính lại hằng số thời gian tau
    robot->timeElapsed = 0.0f;                      // Reset thời gian đã trôi qua
}

// =========================
// Tính toán vận tốc hiện tại theo thời gian thực
// =========================
float RobotAcceleration_Update(RobotAcceleration *robot, float dt)
{
    // Cập nhật thời gian đã trôi qua
    robot->timeElapsed += dt;

    // Giới hạn thời gian đã trôi qua trong khoảng [0, duration]
    if (robot->timeElapsed > robot->duration)
    {
        robot->timeElapsed = robot->duration;
    }

    // Tính vận tốc theo công thức gia tốc bậc 1
    float velocityRange = robot->targetVelocity - robot->previousTarget;
    robot->currentVelocity = robot->previousTarget + velocityRange * (1 - exp(-robot->timeElapsed / robot->tau));

    return robot->currentVelocity;
}
