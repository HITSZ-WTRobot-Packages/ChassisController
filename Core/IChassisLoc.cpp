/**
 * @file    IChassisLoc.cpp
 * @author  syhanjin
 * @date    2026-11-17
 */
#include "IChassisLoc.hpp"

namespace chassis::loc
{

namespace
{
void rotate_SO2(float& rx, float& ry, const float x, const float y, const float theta)
{
    const float sin_yaw = sinf(DEG2RAD(theta)), cos_yaw = cosf(DEG2RAD(theta));
    rx = x * cos_yaw - y * sin_yaw;
    ry = x * sin_yaw + y * cos_yaw;
}
} // namespace

Velocity IChassisLoc::WorldVelocity2BodyVelocity(const Velocity& velocity_in_world) const
{
    return rotateVelocity(velocity_in_world, -postureInWorld().yaw);
}
Velocity IChassisLoc::BodyVelocity2WorldVelocity(const Velocity& velocity_in_body) const
{
    return rotateVelocity(velocity_in_body, postureInWorld().yaw);
}
Acceleration IChassisLoc::WorldAcceleration2BodyAcceleration(
        const Acceleration& acceleration_in_world) const
{
    return rotateAcceleration(acceleration_in_world, -postureInWorld().yaw);
}
Acceleration IChassisLoc::BodyAcceleration2WorldAcceleration(
        const Acceleration& acceleration_in_body) const
{
    return rotateAcceleration(acceleration_in_body, postureInWorld().yaw);
}

Posture IChassisLoc::WorldPosture2BodyPosture(const Posture& posture_in_world) const
{
    return WorldPosture2RelativePosture(postureInWorld(), posture_in_world);
}
Posture IChassisLoc::BodyPosture2WorldPosture(const Posture& posture_in_body) const
{
    return RelativePosture2WorldPosture(postureInWorld(), posture_in_body);
}
Posture IChassisLoc::RelativePosture2WorldPosture(const Posture& base_in_world,
                                                  const Posture& posture_in_base)
{
    const float sin_yaw = sinf(DEG2RAD(base_in_world.yaw));
    const float cos_yaw = cosf(DEG2RAD(base_in_world.yaw));

    return {
        .x   = posture_in_base.x * cos_yaw - posture_in_base.y * sin_yaw + base_in_world.x,
        .y   = posture_in_base.x * sin_yaw + posture_in_base.y * cos_yaw + base_in_world.y,
        .yaw = posture_in_base.yaw + base_in_world.yaw,
    };
}
Posture IChassisLoc::WorldPosture2RelativePosture(const Posture& base_in_world,
                                                  const Posture& posture_in_world)
{
    const float sin_yaw = sinf(DEG2RAD(-base_in_world.yaw));
    const float cos_yaw = cosf(DEG2RAD(-base_in_world.yaw));

    const float tx = posture_in_world.x - base_in_world.x;
    const float ty = posture_in_world.y - base_in_world.y;

    return {
        .x   = tx * cos_yaw - ty * sin_yaw,
        .y   = tx * sin_yaw + ty * cos_yaw,
        .yaw = posture_in_world.yaw - base_in_world.yaw,
    };
}

Posture IChassisLoc::CurrentPostureRelativeTo(const Posture& base_in_world) const
{
    return WorldPosture2RelativePosture(base_in_world, postureInWorld());
}

std::pair<Velocity, Acceleration> IChassisLoc::VelocityAndAccelerationInBody(
        const Velocity& v_in_w, const Acceleration& a_in_w) const
{
    const Posture base = postureInWorld();
    return std::make_pair<Velocity, Acceleration>(rotateVelocity(v_in_w, -base.yaw),
                                                  rotateAcceleration(a_in_w, -base.yaw));
}

Velocity IChassisLoc::rotateVelocity(const Velocity& inp, const float theta)
{
    Velocity out{};
    rotate_SO2(out.vx, out.vy, inp.vx, inp.vy, theta);
    out.wz = inp.wz;
    return out;
}
Acceleration IChassisLoc::rotateAcceleration(const Acceleration& inp, float theta)
{
    Acceleration out{};
    rotate_SO2(out.ax, out.ay, inp.ax, inp.ay, theta);
    out.alpha = inp.alpha;
    return out;
}

} // namespace chassis::loc