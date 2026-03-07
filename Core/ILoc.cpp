/**
 * @file    ILoc.cpp
 * @author  syhanjin
 * @date    2026-03-07
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#include "ILoc.hpp"
#include <cmath>

#define DEG2RAD(__DEG__) ((__DEG__) * (float) 3.14159265358979323846f / 180.0f)

namespace chassis_loc
{

ILoc::Velocity ILoc::WorldVelocity2BodyVelocity(const Velocity& velocity_in_world) const
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

ILoc::Velocity ILoc::BodyVelocity2WorldVelocity(const Velocity& velocity_in_body) const
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

ILoc::Posture ILoc::WorldPosture2BodyPosture(const Posture& posture_in_world) const
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

ILoc::Posture ILoc::BodyPosture2WorldPosture(const Posture& posture_in_body) const
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

} // namespace chassis_loc