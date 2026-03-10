/**
 * @file    Special_Steering4.cpp
 * @author  rediduck
 * @brief   这个底盘构型比较特殊，而且底盘是可以移动的，详情可联系rediduck（39328428）
 * @date    2026-03-09
 */
#include "Special_Steering4.hpp"

#include <cmath>

#define RAD2DEG(__RAD__) ((__RAD__) / 3.14159265358979323846f * 180)
#define DEG2RAD(__DEG__) ((__DEG__) * 3.14159265358979323846f / 180)

namespace chassis
{

Special_Steering4::Special_Steering4(chassis_loc::ILoc& loc, const Config& cfg) :
    IChassis(loc), enable_calib_(cfg.enable_calibration), wheel_radius_(1e-3f * cfg.radius),
    distance_x_(1e-3f * cfg.distance_x), distance_y_(1e-3f * cfg.distance_y),
    half_distance_x_(0.5e-3f * cfg.distance_x), half_distance_y_(0.5e-3f * cfg.distance_y),
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

void Special_Steering4::applyVelocity(const Velocity& velocity)
{
    if (enable_calib_ && !calibrated_)
        return;
    for (size_t i = 0; i < static_cast<size_t>(WheelType::Max); ++i)
    {
        const auto [xi, yi]   = getWheelPosition(static_cast<WheelType>(i));
        // ILoc velocity interface uses deg/s; convert to rad/s for kinematic computation.
        const float wz_rad    = DEG2RAD(velocity.wz);
        const float vxi       = velocity.vx + wz_rad * yi;
        const float vyi       = velocity.vy + wz_rad * xi;
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

void Special_Steering4::velocityControllerUpdate()
{
    if (enable_calib_ && !calibrated_)
    {
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
            const float steer_angle     = wheel_[i].getSteerAngle();
            const float driver_speed    = wheel_[i].getDriveSpeed() / spd2rpm_;
            const float steer_angle_rad = DEG2RAD(steer_angle);
            const float sin_theta       = sinf(steer_angle_rad);
            const float cos_theta       = cosf(steer_angle_rad);
            vx += driver_speed * cos_theta;
            vy += driver_speed * sin_theta;
            if (0 == i)
            {
                wz += vy / distance_x_;
            }
            if (2 == i)
            {
                wz -= vy / distance_x_;
            }
        }
        velocity_.vx = 0.25f * vx;
        velocity_.vy = 0.25f * vy;
        velocity_.wz = RAD2DEG(wz);
    }

    for (auto& w : wheel_)
        w.update();
}

Special_Steering4::WheelPosition Special_Steering4::getWheelPosition(const WheelType wheel) const
{
    constexpr WheelPosition WHEEL_POS[static_cast<size_t>(WheelType::Max)] = {
        { 0, 1 }, { -1, 1 }, { 0, -1 }, { 1, -1 }
    };
    const auto [kx, ky] = WHEEL_POS[static_cast<size_t>(wheel)];
    return {
        kx * (half_distance_y_ + ky * y_shift_),
        (ky + ky) * half_distance_x_,
    };
}

} // namespace chassis
