/**
 * @file    Omni4.cpp
 * @author  ported by GitHub Copilot
 * @date    2026-03-08
 */
#include "Omni4.hpp"

#include <cmath>

#define RPS2RPM(__RPS__) ((__RPS__) * 60.0f / (2.0f * 3.14159265358979323846f))
#define DEG2RAD(__DEG__) ((__DEG__) * (float) 3.14159265358979323846f / 180.0f)
#define RPM2DPS(__RPM__) ((__RPM__) / 60.0f * 360.0f)

namespace chassis::motion
{
namespace
{
constexpr float kInvSqrt2 = 0.7071067811865475f;

constexpr size_t idx(const Omni4::WheelType w)
{
    return static_cast<size_t>(w);
}
} // namespace

Omni4::Omni4(const Config& cfg) : wheel_radius_(cfg.wheel_radius * 1e-3f)
{
    const float half_x = cfg.wheel_distance_x * 1e-3f * 0.5f;
    const float half_y = cfg.wheel_distance_y * 1e-3f * 0.5f;
    half_diag_         = std::hypot(half_x, half_y);

    wheel_[idx(WheelType::FrontRight)] = cfg.wheel_front_right;
    wheel_[idx(WheelType::FrontLeft)]  = cfg.wheel_front_left;
    wheel_[idx(WheelType::RearLeft)]   = cfg.wheel_rear_left;
    wheel_[idx(WheelType::RearRight)]  = cfg.wheel_rear_right;
}

bool Omni4::enable()
{
    bool enabled = true;
    for (const auto& w : wheel_)
        enabled &= w->enable();

    if (!enabled)
    {
        for (const auto& w : wheel_)
            w->disable();
    }
    else
    {
        // Reset angle once before odometry integration.
        for (const auto& w : wheel_)
            w->getMotor()->resetAngle();
    }

    enabled_ = enabled;
    return enabled;
}

void Omni4::disable()
{
    for (const auto& w : wheel_)
        w->disable();
    enabled_ = false;
}

void Omni4::applyVelocity(const Velocity& velocity)
{
    const float wz_rad = DEG2RAD(velocity.wz);

    const float v_fr = kInvSqrt2 * (velocity.vx + velocity.vy) + half_diag_ * wz_rad;
    const float v_fl = kInvSqrt2 * (-velocity.vx + velocity.vy) + half_diag_ * wz_rad;
    const float v_rl = kInvSqrt2 * (-velocity.vx - velocity.vy) + half_diag_ * wz_rad;
    const float v_rr = kInvSqrt2 * (velocity.vx - velocity.vy) + half_diag_ * wz_rad;

    wheel_[idx(WheelType::FrontRight)]->setRef(RPS2RPM(v_fr / wheel_radius_));
    wheel_[idx(WheelType::FrontLeft)]->setRef(RPS2RPM(v_fl / wheel_radius_));
    wheel_[idx(WheelType::RearLeft)]->setRef(RPS2RPM(v_rl / wheel_radius_));
    wheel_[idx(WheelType::RearRight)]->setRef(RPS2RPM(v_rr / wheel_radius_));
}

void Omni4::update()
{
    if (!enabled())
        return;
    for (const auto& w : wheel_)
        w->update();
}

Velocity Omni4::forwardGetVelocity()
{
    Velocity vel{};

    const float w_fr = wheel_[idx(WheelType::FrontRight)]->getMotor()->getVelocity();
    const float w_fl = wheel_[idx(WheelType::FrontLeft)]->getMotor()->getVelocity();
    const float w_rl = wheel_[idx(WheelType::RearLeft)]->getMotor()->getVelocity();
    const float w_rr = wheel_[idx(WheelType::RearRight)]->getMotor()->getVelocity();

    vel.vx = RPM2DPS(wheel_radius_ * 0.25f * 1.4142135623730951f *
                     DEG2RAD(w_fr - w_fl - w_rl + w_rr));
    vel.vy = RPM2DPS(wheel_radius_ * 0.25f * 1.4142135623730951f *
                     DEG2RAD(w_fr + w_fl - w_rl - w_rr));

    if (half_diag_ > 1e-6f)
    {
        vel.wz = RPM2DPS(-(wheel_radius_ / (4.0f * half_diag_)) *
                         DEG2RAD(w_fr + w_fl + w_rl + w_rr));
    }
    else
    {
        vel.wz = 0.0f;
    }

    return vel;
}

} // namespace chassis::motion
