/**
 * @file    IChassis.cpp
 * @author  syhanjin
 * @date    2026-01-31
 */
#include "IChassis.hpp"
#include <cassert>
#include <cmath>

namespace chassis
{
IChassis::Velocity IChassis::WorldVelocity2BodyVelocity(const Velocity& velocity_in_world) const
{
    const float _sin_yaw = sinf(DEG2RAD(-posture_.in_world.yaw)),
                _cos_yaw = cosf(DEG2RAD(-posture_.in_world.yaw));

    const Velocity velocity_in_body = {
        .vx = velocity_in_world.vx * _cos_yaw - velocity_in_world.vy * _sin_yaw,
        .vy = velocity_in_world.vx * _sin_yaw + velocity_in_world.vy * _cos_yaw,
        .wz = velocity_in_world.wz
    };

    return velocity_in_body;
}

IChassis::Velocity IChassis::BodyVelocity2WorldVelocity(const Velocity& velocity_in_body) const
{
    const float sin_yaw = sinf(DEG2RAD(posture_.in_world.yaw)),
                cos_yaw = cosf(DEG2RAD(posture_.in_world.yaw));

    const Velocity velocity_in_world = {
        .vx = velocity_in_body.vx * cos_yaw - velocity_in_body.vy * sin_yaw,
        .vy = velocity_in_body.vx * sin_yaw + velocity_in_body.vy * cos_yaw,
        .wz = velocity_in_body.wz,
    };

    return velocity_in_world;
}

IChassis::Posture IChassis::WorldPosture2BodyPosture(const Posture& posture_in_world) const
{
    const float _sin_yaw = sinf(DEG2RAD(-posture_.in_world.yaw)),
                _cos_yaw = cosf(DEG2RAD(-posture_.in_world.yaw));

    const float tx = posture_in_world.x - posture_.in_world.x;
    const float ty = posture_in_world.y - posture_.in_world.y;

    const Posture posture_in_body = {
        .x   = tx * _cos_yaw - ty * _sin_yaw,
        .y   = tx * _sin_yaw + ty * _cos_yaw,
        .yaw = posture_in_world.yaw - posture_.in_world.yaw,
    };

    return posture_in_body;
}

IChassis::Posture IChassis::BodyPosture2WorldPosture(const Posture& posture_in_body) const
{
    const float sin_yaw            = sinf(DEG2RAD(posture_.in_world.yaw)),
                cos_yaw            = cosf(DEG2RAD(posture_.in_world.yaw));
    const Posture posture_in_world = {
        .x   = posture_in_body.x * cos_yaw - posture_in_body.y * sin_yaw + posture_.in_world.x,
        .y   = posture_in_body.x * sin_yaw + posture_in_body.y * cos_yaw + posture_.in_world.y,
        .yaw = posture_in_body.yaw + posture_.in_world.yaw,
    };

    return posture_in_world;
}

/**
 * 更新底盘反馈
 * @param dt 更新间隔 (unit: s)
 */
void IChassis::feedbackUpdate(const float dt)
{
    if (!enabled() || dt <= 0)
        return;

    if (isOpsEnabled())
    {
        assert(feedback_.x && feedback_.y && feedback_.yaw);
        // 直接读取 OPS
        posture_.in_world.x   = *feedback_.x;
        posture_.in_world.y   = *feedback_.y;
        posture_.in_world.yaw = *feedback_.yaw;
        return;
    }
    // 通过里程计或者运动学解算计算位置
    // 通过速度积分还是直接读取底盘
    const bool use_vel = kinematicsType() == WheeledKinematicsType::VelocityIntegrated;
    // 速度反馈
    const float vx = feedback_.vx != nullptr ? *feedback_.vx : forwardGetVx();
    const float vy = feedback_.vy != nullptr ? *feedback_.vy : forwardGetVy();
    const float wz = feedback_.wz != nullptr ? *feedback_.wz : forwardGetWz();
    // 计算里程增量
    float dx, dy;
    if (feedback_.sx || !use_vel)
    {
        // use measured or estimated position
        const float sx    = feedback_.sx ? *feedback_.sx : forwardGetX();
        dx                = sx - last_feedback_.sx;
        last_feedback_.sx = sx;
    }
    else
    {
        // fallback: integrate velocity
        dx = 0.5f * (vx + last_feedback_.vx) * dt;
        last_feedback_.sx += dx;
    }
    if (feedback_.sy || !use_vel)
    {
        // use measured or estimated position
        const float sy    = feedback_.sy ? *feedback_.sy : forwardGetY();
        dy                = sy - last_feedback_.sy;
        last_feedback_.sy = sy;
    }
    else
    {
        // fallback: integrate velocity
        dy = 0.5f * (vy + last_feedback_.vy) * dt;
        last_feedback_.sy += dy;
    }
    // 计算平均 yaw，作为积分方向
    const float& yaw_prev = last_feedback_.yaw;

    const float yaw = feedback_.yaw ? *feedback_.yaw                              // 直接读取陀螺仪
                      : use_vel ? yaw_prev + 0.5f * (wz + last_feedback_.wz) * dt // 用平均速度积分
                                : forwardGetYaw(); // 直接读取底盘解算

    const float ave_yaw              = (yaw + yaw_prev) * 0.5f;
    const float ave_yaw_in_world_rad = DEG2RAD(ave_yaw - world_.posture.yaw);
    last_feedback_.yaw               = yaw;

    // 积分
    posture_.in_world.x += dx * cosf(ave_yaw_in_world_rad) - dy * sinf(ave_yaw_in_world_rad);
    posture_.in_world.y += dx * sinf(ave_yaw_in_world_rad) + dy * cosf(ave_yaw_in_world_rad);
    posture_.in_world.yaw = yaw - world_.posture.yaw;
    // 更新速度反馈
    last_feedback_.vx = vx, last_feedback_.vy = vy, last_feedback_.wz = wz;

    // 更新底盘速度
    velocity_.in_body.vx = vx;
    velocity_.in_body.vy = vy;
    velocity_.in_body.wz = wz;
    velocity_.in_world   = BodyVelocity2WorldVelocity(velocity_.in_body);
}

void IChassis::applySetWorldFromCurrent()
{
    // 启用 OPS 的情况下 world 由 OPS 管理
    if (isOpsEnabled())
        return;
    world_.posture.x += posture_.in_world.x;
    world_.posture.y += posture_.in_world.y;
    world_.posture.yaw += posture_.in_world.yaw;
    posture_.in_world.x   = 0.0f;
    posture_.in_world.y   = 0.0f;
    posture_.in_world.yaw = 0.0f;
    velocity_.in_world    = velocity_.in_body;
}

IChassis::IChassis(const Config& cfg) : feedback_(cfg.feedback_source) {}
} // namespace chassis