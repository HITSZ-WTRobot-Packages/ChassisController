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

    [[nodiscard]] virtual Velocity velocityInBody() const  = 0;
    [[nodiscard]] virtual Velocity velocityInWorld() const = 0;

    [[nodiscard]] virtual Posture postureInWorld() const = 0;

    [[nodiscard]] Velocity WorldVelocity2BodyVelocity(const Velocity& velocity_in_world) const
    {
        return rotateVelocity(velocity_in_world, -postureInWorld().yaw);
    }

    [[nodiscard]] Velocity BodyVelocity2WorldVelocity(const Velocity& velocity_in_body) const
    {
        return rotateVelocity(velocity_in_body, postureInWorld().yaw);
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

    [[nodiscard]] static Velocity rotateVelocity(const Velocity& inp, const float theta)
    {
        const float sin_yaw = sinf(DEG2RAD(theta)), cos_yaw = cosf(DEG2RAD(theta));

        const Velocity out = {
            .vx = inp.vx * cos_yaw - inp.vy * sin_yaw,
            .vy = inp.vx * sin_yaw + inp.vy * cos_yaw,
            .wz = inp.wz,
        };

        return out;
    }
};

} // namespace chassis::loc
