/**
 * @file    Steering4.cpp
 * @author  syhanjin
 * @date    2026-02-28
 * @brief   四舵轮底盘运动学实现。
 */
#include "Steering4.hpp"

#include <cmath>

#define RAD2DEG(__RAD__) ((__RAD__) / 3.14159265358979323846f * 180)
#define DEG2RAD(__DEG__) ((__DEG__) * 3.14159265358979323846f / 180)

namespace chassis::motion
{
Steering4::Steering4(const Config& cfg) :
    enable_calib_(cfg.enable_calibration), //
    wheel_radius_(1e-3f * cfg.radius),     // mm to m
    half_distance_x(0.5e-3f * cfg.distance_x), half_distance_y(0.5e-3f * cfg.distance_y),
    inv_l2_(4.0f / ((1e-3f * cfg.distance_x) * (1e-3f * cfg.distance_x) +
                    (1e-3f * cfg.distance_y) * (1e-3f * cfg.distance_y))),
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
    // inv_l2_ / spd2rpm_ 都是为运行期减少重复计算准备的几何常量。
}

void Steering4::applyVelocity(const Velocity& velocity)
{
    if (!isReady())
        // 需要校准但未校准，无法设置速度
        return;
    for (size_t i = 0; i < static_cast<size_t>(WheelType::Max); ++i)
    {
        const auto [xi, yi]   = getWheelPosition(static_cast<WheelType>(i));
        const float wz_rad    = DEG2RAD(velocity.wz);
        // 刚体平面运动中，轮心速度 = 底盘平移速度 + 角速度带来的切向速度。
        const float vxi       = velocity.vx - wz_rad * yi;
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
            // atan2 给出轮子应该朝向的平面角度，具体是否翻轮由 SteeringWheel 再优化。
            const float angle = RAD2DEG(atan2f(vyi, vxi));
            wheel_[i].setTargetVelocity({ angle, speed_rpm });
        }
    }
}
void Steering4::update()
{
    if (enable_calib_ && !calibrated_)
    {
        // 校准阶段只判断是否已经全部完成，不输出反馈速度，避免上层误以为底盘可用。
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
            // 把每个轮子的线速度按舵向角投影回车体 x/y，再由几何关系反解角速度。
            vx += driver_speed * cos_theta;
            vy += driver_speed * sin_theta;
            wz += driver_speed * (-yi * cos_theta + xi * sin_theta);
        }
        velocity_.vx = 0.25f * vx;
        velocity_.vy = 0.25f * vy;
        velocity_.wz = RAD2DEG(inv_l2_ * wz);
    }

    for (auto& w : wheel_)
        w.update();
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
} // namespace chassis::motion
