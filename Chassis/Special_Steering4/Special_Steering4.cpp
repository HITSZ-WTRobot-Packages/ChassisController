/**
 * @file    Special_Steering4.cpp
 * @author  rediduck
 * @brief   这个底盘构型比较特殊，而且底盘是可以移动的，详情可联系rediduck（39328428）
 * @date    2026-03-09
 */
#include "Special_Steering4.hpp"

#include <cmath>

#define RAD2DEG(__RAD__) ((__RAD__) / 3.14159265358979323846f * 180.0f)
#define DEG2RAD(__DEG__) ((__DEG__) * 3.14159265358979323846f / 180.0f)

namespace chassis::motion
{

Special_Steering4::Special_Steering4(const Config& cfg) :
    enable_calib_(cfg.enable_calibration), wheel_radius_(1e-3f * cfg.radius),
    distance_x_(1e-3f * cfg.distance_x), distance_y_(1e-3f * cfg.distance_y),
    half_distance_x_(0.5e-3f * cfg.distance_x), half_distance_y_(0.5e-3f * cfg.distance_y),
    inv_l2_(2.0f * ((1e-3f * cfg.distance_x / 2.0f) * (1e-3f * cfg.distance_x / 2.0f) +
                    (1e-3f * cfg.distance_y / 2.0f + 1e-3f * y_shift_) *
                            (1e-3f * cfg.distance_y / 2.0f + 1e-3f * y_shift_))),
    spd2rpm_(1.0f / (wheel_radius_ * 3.14159265358979323846f * 2) * 60.0f),
    wheel_{
            steering::Special_SteeringWheel(cfg.wheel_front_right.cfg,
                                            cfg.enable_calibration,
                                            cfg.wheel_front_right.calib_cfg),
            steering::Special_SteeringWheel(cfg.wheel_front_left.cfg,
                                            cfg.enable_calibration,
                                            cfg.wheel_front_left.calib_cfg),
            steering::Special_SteeringWheel(cfg.wheel_rear_left.cfg,
                                            cfg.enable_calibration,
                                            cfg.wheel_rear_left.calib_cfg),
            steering::Special_SteeringWheel(cfg.wheel_rear_right.cfg,
                                            cfg.enable_calibration,
                                            cfg.wheel_rear_right.calib_cfg),
    }
{
}

float angle_test[4] = {0}; // namespace chassis::motion
float speed_vx[4]   = {0};
float speed_vy[4]   = {0};
float speed_wz[4]   = {0};

void Special_Steering4::applyVelocity(const Velocity& velocity)
{
    if (enable_calib_ && !calibrated_)
        return;
    for (size_t i = 0; i < static_cast<size_t>(WheelType::Max); ++i)
    {
        const auto [xi, yi] = getWheelPosition(static_cast<WheelType>(i));
        // ILoc velocity interface uses deg/s; convert to rad/s for kinematic computation.
        const float wz_rad = DEG2RAD(velocity.wz);
        const float vxi    = velocity.vx + wz_rad * yi;
        const float vyi    = velocity.vy + wz_rad * xi;

        speed_vx[i] = vxi;
        speed_vy[i] = vyi;
        speed_wz[i] = wz_rad;

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

            angle_test[i] = angle;

            wheel_[i].setTargetVelocity({angle, speed_rpm});
        }
    }
}

void Special_Steering4::update()
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
        float vx = 0, vy = 0, wz = 0, vxi = 0, vyi = 0;
        for (size_t i = 0; i < static_cast<size_t>(WheelType::Max); ++i)
        {
            const auto [xi, yi]         = getWheelPosition(static_cast<WheelType>(i));
            const float steer_angle     = wheel_[i].getSteerAngle();
            const float driver_speed    = wheel_[i].getDriveSpeed() / spd2rpm_;
            const float steer_angle_rad = DEG2RAD(steer_angle);
            const float sin_theta       = sinf(steer_angle_rad);
            const float cos_theta       = cosf(steer_angle_rad);
            vxi                         = driver_speed * cos_theta;
            vyi                         = driver_speed * sin_theta;
            vx += vxi;
            vy += vyi;
            wz += vxi * yi + vyi * xi;
        }
        velocity_.vx = 0.25f * vx;
        velocity_.vy = 0.25f * vy;
        velocity_.wz = RAD2DEG(wz / inv_l2_);
    }

    for (auto& w : wheel_)
        w.update();
}

Special_Steering4::WheelPosition Special_Steering4::getWheelPosition(const WheelType wheel) const
{
    constexpr WheelPosition WHEEL_POS[static_cast<size_t>(WheelType::Max)] = {{1, 0},
                                                                              {0, -1},
                                                                              {-1, 0},
                                                                              {0, 1}};
    const auto [kx, ky] = WHEEL_POS[static_cast<size_t>(wheel)];
    return {
            kx * half_distance_x_,
            ky * (half_distance_y_ + y_shift_),
    };
}

} // namespace chassis::motion
