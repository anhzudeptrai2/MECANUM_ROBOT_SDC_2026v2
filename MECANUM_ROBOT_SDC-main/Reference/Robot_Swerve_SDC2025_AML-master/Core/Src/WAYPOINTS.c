#include "WAYPOINTS.h"

void Motion_Profile_Init(MotionProfile *mp, Waypoint start, Waypoint target, float a_max, float v_max)
{
    mp->start = start;
    mp->target = target;
    mp->a_max = a_max;
    mp->v_max = v_max;

    // Tính khoảng cách giữa start và target (giả sử đơn vị mm)
    float dx = target.x - start.x;
    float dy = target.y - start.y;
    mp->distance = sqrtf(dx * dx + dy * dy);

    // Tính thời gian gia tốc
    mp->t_acc = v_max / a_max;
    float d_acc = 0.5f * a_max * mp->t_acc * mp->t_acc;

    // Kiểm tra xem có đủ khoảng cách để đạt tốc độ tối đa hay không
    if (2 * d_acc >= mp->distance)
    {
        // Nếu khoảng cách quá ngắn, sử dụng profile tam giác
        mp->triangular = 1;
        mp->t_acc = sqrtf(mp->distance / a_max);
        mp->t_const = 0.0f;
        mp->T = 2 * mp->t_acc;
    }
    else
    {
        // Nếu đủ, profile hình thang có giai đoạn tốc độ không đổi
        mp->triangular = 0;
        mp->t_const = (mp->distance - 2 * d_acc) / v_max;
        mp->T = 2 * mp->t_acc + mp->t_const;
    }
}

Waypoint Motion_Profile_Compute(MotionProfile *mp, float t)
{
    Waypoint sp;

    // Clamp thời gian t vào khoảng [0, T]
    if (t < 0)
        t = 0;
    if (t > mp->T)
        t = mp->T;

    float d; // khoảng cách đã đi tại thời gian t
    if (t <= mp->t_acc)
    {
        // Giai đoạn gia tốc
        d = 0.5f * mp->a_max * t * t;
    }
    else if (!mp->triangular && t <= (mp->t_acc + mp->t_const))
    {
        // Giai đoạn tốc độ không đổi
        float d_acc = 0.5f * mp->a_max * mp->t_acc * mp->t_acc;
        d = d_acc + mp->v_max * (t - mp->t_acc);
    }
    else
    {
        // Giai đoạn giảm tốc
        float T = mp->T;
        d = mp->distance - 0.5f * mp->a_max * (T - t) * (T - t);
    }

    // Tính tỷ lệ đã di chuyển so với tổng khoảng cách
    float fraction = (mp->distance > 0) ? (d / mp->distance) : 1.0f;

    // Nội suy vị trí và góc theta
    sp.x = mp->start.x + fraction * (mp->target.x - mp->start.x);
    sp.y = mp->start.y + fraction * (mp->target.y - mp->start.y);
    sp.theta = mp->start.theta + fraction * (mp->target.theta - mp->start.theta);
    sp.time = t;

    return sp;
}

int Motion_Profile_Finished(MotionProfile *mp, float t)
{
    return t >= mp->T;
}