/**
 * @file    IChassisLoc.hpp
 * @author  syhanjin
 * @date    2026-03-07
 */
#pragma once
#include "IChassisDef.hpp"
#include "IChassisMotion.hpp"

#include <cmath>

#ifndef DEG2RAD
#    define DEG2RAD(__DEG__) ((__DEG__) * (float)3.14159265358979323846f / 180.0f)
#endif

namespace chassis::loc
{

class IChassisLoc
{
public:
    explicit IChassisLoc(motion::IChassisMotion& motion) : motion_(&motion) {}

    virtual ~IChassisLoc() = default;

    [[nodiscard]] virtual const Velocity& velocityInBody() const  = 0;
    [[nodiscard]] virtual const Velocity& velocityInWorld() const = 0;

    [[nodiscard]] virtual const Posture& postureInWorld() const = 0;

    [[nodiscard]] Velocity WorldVelocity2BodyVelocity(const Velocity& velocity_in_world) const
    {
        const float _sin_yaw = sinf(DEG2RAD(-postureInWorld().yaw)),
                    _cos_yaw = cosf(DEG2RAD(-postureInWorld().yaw));

        const Velocity velocity_in_body = {
            .vx = velocity_in_world.vx * _cos_yaw - velocity_in_world.vy * _sin_yaw,
            .vy = velocity_in_world.vx * _sin_yaw + velocity_in_world.vy * _cos_yaw,
            .wz = velocity_in_world.wz
        };

        return velocity_in_body;
    }

    [[nodiscard]] Velocity BodyVelocity2WorldVelocity(const Velocity& velocity_in_body) const
    {
        const float sin_yaw = sinf(DEG2RAD(postureInWorld().yaw)),
                    cos_yaw = cosf(DEG2RAD(postureInWorld().yaw));

        const Velocity velocity_in_world = {
            .vx = velocity_in_body.vx * cos_yaw - velocity_in_body.vy * sin_yaw,
            .vy = velocity_in_body.vx * sin_yaw + velocity_in_body.vy * cos_yaw,
            .wz = velocity_in_body.wz,
        };

        return velocity_in_world;
    }

    [[nodiscard]] Posture WorldPosture2BodyPosture(const Posture& posture_in_world) const
    {
        const float _sin_yaw = sinf(DEG2RAD(-postureInWorld().yaw)),
                    _cos_yaw = cosf(DEG2RAD(-postureInWorld().yaw));

        const float tx = posture_in_world.x - postureInWorld().x;
        const float ty = posture_in_world.y - postureInWorld().y;

        const Posture posture_in_body = {
            .x   = tx * _cos_yaw - ty * _sin_yaw,
            .y   = tx * _sin_yaw + ty * _cos_yaw,
            .yaw = posture_in_world.yaw - postureInWorld().yaw,
        };

        return posture_in_body;
    }

    [[nodiscard]] Posture BodyPosture2WorldPosture(const Posture& posture_in_body) const
    {
        const float sin_yaw            = sinf(DEG2RAD(postureInWorld().yaw)),
                    cos_yaw            = cosf(DEG2RAD(postureInWorld().yaw));
        const Posture posture_in_world = {
            .x   = posture_in_body.x * cos_yaw - posture_in_body.y * sin_yaw + postureInWorld().x,
            .y   = posture_in_body.x * sin_yaw + posture_in_body.y * cos_yaw + postureInWorld().y,
            .yaw = posture_in_body.yaw + postureInWorld().yaw,
        };

        return posture_in_world;
    }

protected:
    motion::IChassisMotion* motion_{ nullptr };

    [[nodiscard]] auto forwardGetVelocity() const { return motion_->forwardGetVelocity(); }
};

} // namespace chassis::loc
