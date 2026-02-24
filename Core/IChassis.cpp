/**
 * @file    IChassis.cpp
 * @author  syhanjin
 * @date    2026-01-31
 */
#include "IChassis.hpp"
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

void IChassis::feedbackUpdate()
{
    if (!enabled())
        return;
    update_posture();
    update_velocity_feedback();
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

void IChassis::update_posture()
{
    if (isOpsEnabled())
    {
        // 直接读取 OPS
        posture_.in_world.x   = *feedback_.x;
        posture_.in_world.y   = *feedback_.y;
        posture_.in_world.yaw = *feedback_.yaw;
    }
    else
    {
        // 通过里程计或者运动学解算计算位置
        const float sx  = feedback_.sx != nullptr ? *feedback_.sx : forwardGetX();
        const float sy  = feedback_.sy != nullptr ? *feedback_.sy : forwardGetY();
        const float yaw = feedback_.yaw != nullptr ? *feedback_.yaw : forwardGetYaw();

        const float dx                   = sx - last_feedback_.sx;
        const float dy                   = sy - last_feedback_.sy;
        const float ave_yaw              = (yaw + last_feedback_.yaw) * 0.5f;
        const float ave_yaw_in_world_rad = DEG2RAD(ave_yaw - world_.posture.yaw);

        last_feedback_.sx  = sx;
        last_feedback_.sy  = sy;
        last_feedback_.yaw = yaw;

        posture_.in_world.x += dx * cosf(ave_yaw_in_world_rad) - dy * sinf(ave_yaw_in_world_rad);
        posture_.in_world.y += dx * sinf(ave_yaw_in_world_rad) + dy * cosf(ave_yaw_in_world_rad);
        posture_.in_world.yaw = yaw - world_.posture.yaw;
    }
}
void IChassis::update_velocity_feedback()
{
    velocity_.in_body.vx = feedback_.vx != nullptr ? *feedback_.vx : forwardGetVx();
    velocity_.in_body.vy = feedback_.vy != nullptr ? *feedback_.vy : forwardGetVy();
    velocity_.in_body.wz = feedback_.wz != nullptr ? *feedback_.wz : forwardGetWz();
    velocity_.in_world   = BodyVelocity2WorldVelocity(velocity_.in_body);
}
} // namespace chassis