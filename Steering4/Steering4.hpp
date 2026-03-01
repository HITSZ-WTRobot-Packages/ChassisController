/**
 * @file    Steering4.hpp
 * @author  syhanjin
 * @date    2026-02-28
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#pragma once
#include "IChassis.hpp"
#include "SteeringWheel.hpp"

namespace chassis
{

class Steering4 : public IChassis
{
public:
    enum class WheelType : size_t
    {
        FrontRight = 0U, ///< 右前轮
        FrontLeft  = 1U, ///< 左前轮
        RearLeft   = 2U, ///< 左后轮
        RearRight  = 3U, ///< 右后轮
        Max
    };
    struct Config
    {
        bool enable_calibration = false; // 是否启用校准功能

        float radius;     // 驱动轮半径 (unit: mm)
        float distance_x; // 前后轮距 (unit: mm)
        float distance_y; // 左右轮距 (unit: mm)

        struct Wheel
        {
            steering::SteeringWheel::Config            cfg;
            steering::SteeringWheel::CalibrationConfig calib_cfg;
        };
        Wheel wheel_front_right; ///< 右前方
        Wheel wheel_front_left;  ///< 左前方
        Wheel wheel_rear_left;   ///< 左后方
        Wheel wheel_rear_right;  ///< 右后方
    };

    Steering4(Config cfg, const IChassis::Config& base_cfg);
    [[nodiscard]] bool enable() override
    {
        if (enabled_)
            return true;
        bool enabled = true;
        for (auto& w : wheel_)
            enabled &= w.enable();
        if (!enabled)
        {
            disable();
            return false;
        }
        enabled_ = true;
        return true;
    }

    void disable() override
    {
        for (auto& w : wheel_)
            w.disable();
        enabled_ = false;
    }

    void startCalibration()
    {
        for (auto& w : wheel_)
            wheel_->startCalibration();
    }

protected:
    void applyVelocity(const Velocity& velocity) override;
    void velocityControllerUpdate() override;

    float forwardGetVx() override
    {
        return velocity_.vx;
    }
    float forwardGetVy() override
    {
        return velocity_.vy;
    }
    float forwardGetWz() override
    {
        return velocity_.wz;
    }

    [[nodiscard]] WheeledKinematicsType kinematicsType() const override
    {
        return WheeledKinematicsType::VelocityIntegrated;
    }
    float forwardGetX() override   = 0;
    float forwardGetY() override   = 0;
    float forwardGetYaw() override = 0;

private:
    bool enabled_{ false };
    bool enable_calib_;
    bool calibrated_{ false };

    float wheel_radius_;
    float half_distance_x;
    float half_distance_y;
    float inv_l2_;
    float spd2rpm_;

    struct
    {
        float vx, vy, wz;
    } velocity_{}; // 反馈速度

    steering::SteeringWheel wheel_[];

    struct WheelPosition
    {
        float x, y;
    };

    WheelPosition getWheelPosition(WheelType wheel) const;
};

} // namespace chassis
