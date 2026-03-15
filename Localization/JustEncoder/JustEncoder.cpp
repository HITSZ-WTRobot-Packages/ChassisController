/**
 * @file    JustEncoder.cpp
 * @author  syhanjin
 * @date    2026-03-07
 */
#include "JustEncoder.hpp"
#include <cmath>

#define DEG2RAD(__DEG__) ((__DEG__) * (float)3.14159265358979323846f / 180.0f)

namespace chassis::loc
{
void JustEncoder::update(const float dt)
{
    // 速度反馈
    const auto velocity     = forwardGetVelocity();
    const auto [vx, vy, wz] = velocity;
    // 速度积分
    const float dx = 0.5f * (vx + velocity_.in_body.vx) * dt;
    const float dy = 0.5f * (vy + velocity_.in_body.vy) * dt;
    // 计算平均 yaw，作为积分方向
    const float& yaw_prev    = posture_.in_world.yaw;
    const float  yaw         = yaw_prev + 0.5f * (wz + velocity_.in_body.wz) * dt; // 用平均速度积分
    const float  ave_yaw_rad = DEG2RAD((yaw + yaw_prev) * 0.5f);
    // 积分
    posture_.in_world.x += dx * cosf(ave_yaw_rad) - dy * sinf(ave_yaw_rad);
    posture_.in_world.y += dx * sinf(ave_yaw_rad) + dy * cosf(ave_yaw_rad);
    posture_.in_world.yaw = yaw;
    // 更新速度反馈
    velocity_.in_body  = velocity;
    velocity_.in_world = BodyVelocity2WorldVelocity(velocity);
}
} // namespace chassis::loc