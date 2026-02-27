#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <stdlib.h>
#include <math.h>

// Định nghĩa cấu trúc waypoint chứa tọa độ và thời gian (giây)
typedef struct
{
    float x;
    float y;
    float theta;
    float time; // thời gian tính từ lúc bắt đầu (giây)
} Waypoint;

// Cấu trúc chứa các tham số của motion profile
typedef struct
{
    Waypoint start;
    Waypoint target;
    float a_max;    // gia tốc tối đa (mm/s^2)
    float v_max;    // tốc độ tối đa (mm/s)
    float distance; // khoảng cách từ start đến target (mm)
    float t_acc;    // thời gian gia tốc (s)
    float t_const;  // thời gian tốc độ không đổi (s)
    float T;        // tổng thời gian chuyển động (s)
    int triangular; // 1 nếu profile tam giác (không có giai đoạn tốc độ không đổi), 0 nếu trapezoidal
} MotionProfile;

// Khởi tạo motion profile dựa trên điểm bắt đầu, điểm kết thúc, gia tốc và tốc độ tối đa
void Motion_Profile_Init(MotionProfile *mp, Waypoint start, Waypoint target, float a_max, float v_max);

// Tính setpoint (vị trí, góc, thời gian) tại thời gian t dựa trên motion profile đã khởi tạo
Waypoint Motion_Profile_Compute(MotionProfile *mp, float t);

// Kiểm tra xem đã chạy hết chuyển động (t >= T) hay chưa
int Motion_Profile_Finished(MotionProfile *mp, float t);

#endif // WAYPOINTS_H
