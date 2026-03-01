/**
 * @file    Steering4.cpp
 * @author  syhanjin
 * @date    2026-02-28
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#include "Steering4.hpp"

#include <cmath>

#define RAD2DEG(__RAD__) ((__RAD__) / 3.14159265358979323846f * 180)

namespace chassis
{
Steering4::Steering4(Config cfg, const IChassis::Config& base_cfg) :
    IChassis(base_cfg), enable_calib_(cfg.enable_calibration),
    wheel_radius_(1e-3f * cfg.radius), // mm to m
    half_distance_x(0.5f * cfg.distance_x), half_distance_y(0.5f * cfg.distance_y),
    inv_l2_(4.0f / (cfg.distance_x * cfg.distance_x + cfg.distance_y * cfg.distance_y)),
    spd2rpm_(1.0f / (wheel_radius_ * 3.14159265358979323846f * 2) * 60.0f), wheel_{
        steering::SteeringWheel(cfg.wheel_front_right.cfg,
                                cfg.enable_calibration,
                                cfg.wheel_front_right.calib_cfg),
        steering::SteeringWheel(cfg.wheel_front_left.cfg,
                                cfg.enable_calibration,
                                cfg.wheel_front_left.calib_cfg),
        steering::SteeringWheel(cfg.wheel_rear_left.cfg,
                                cfg.enable_calibration,
                                cfg.wheel_rear_left.calib_cfg),
        steering::SteeringWheel(cfg.wheel_rear_right.cfg,
                                cfg.enable_calibration,
                                cfg.wheel_rear_right.calib_cfg),
    }
{
}

void Steering4::applyVelocity(const Velocity& velocity)
{
    if (enable_calib_ && !calibrated_)
        // 需要校准但未校准，无法设置速度
        return;
    for (size_t i = 0; i < static_cast<size_t>(WheelType::Max); ++i)
    {
        const auto [xi, yi]   = getWheelPosition(static_cast<WheelType>(i));
        const float vxi       = velocity.vx - velocity.wz * yi;
        const float vyi       = velocity.vy + velocity.wz * xi;
        const float speed_rpm = spd2rpm_ * std::hypot(vxi, vyi);
        if (fabsf(speed_rpm) < 1e-6f)
        {
            // 速度为零，无须转向
            wheel_[i].setTargetVelocity({
                    .angle = wheel_[i].getSteerAngle(),
                    .speed = 0,
            });
        }
        else
        {
            const float angle = RAD2DEG(atan2f(vyi, vxi));
            wheel_[i].setTargetVelocity({ angle, speed_rpm });
        }
    }
}
void Steering4::velocityControllerUpdate()
{
    if (enable_calib_ && !calibrated_)
    {
        // check calibration state
        bool calibrated = true;
        for (auto& w : wheel_)
            calibrated &= w.isCalibrated();
        calibrated_ = calibrated;
    }
    else
    {
        // 更新反馈速度
        float vx = 0, vy = 0, wz = 0;
        for (size_t i = 0; i < static_cast<size_t>(WheelType::Max); ++i)
        {
            const auto [xi, yi]         = getWheelPosition(static_cast<WheelType>(i));
            const float steer_angle     = wheel_[i].getSteerAngle();
            const float driver_speed    = wheel_[i].getDriveSpeed() / spd2rpm_;
            const float steer_angle_rad = DEG2RAD(steer_angle);
            const float sin_theta       = sinf(steer_angle_rad);
            const float cos_theta       = cosf(steer_angle_rad);
            vx += driver_speed * cos_theta;
            vy += driver_speed * sin_theta;
            wz += -yi * cos_theta + xi * sin_theta;
        }
        velocity_.vx = 0.25f * vx;
        velocity_.vy = 0.25f * vy;
        velocity_.wz = inv_l2_ * wz;
    }

    for (auto& w : wheel_)
        wheel_->update();
}

Steering4::WheelPosition Steering4::getWheelPosition(WheelType wheel) const
{
    constexpr WheelPosition WHEEL_POS[static_cast<size_t>(WheelType::Max)] = {
        { 1, -1 }, { 1, 1 }, { -1, 1 }, { -1, -1 }
    };
    const auto [kx, ky] = WHEEL_POS[static_cast<size_t>(wheel)];
    return {
        kx * half_distance_x,
        ky * half_distance_y,
    };
}
} // namespace chassis