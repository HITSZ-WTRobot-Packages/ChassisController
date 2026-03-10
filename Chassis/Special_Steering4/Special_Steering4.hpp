/**
 * @file    Special_Steering4.hpp
 * @author  syhanjin
 * @date    2026-03-09
 */
#pragma once

#include "IChassis.hpp"
#include "SteeringWheel.hpp"

namespace chassis
{

class Special_Steering4 : public IChassis
{
public:
    enum class WheelType : size_t
    {
        Front = 0U, // 前轮
        Left  = 1U, // 左轮
        Rear  = 2U, // 后轮
        Right = 3U, // 右轮
        Max
    };

    struct Config
    {
        bool enable_calibration = false;

        float radius;     // unit: mm
        float distance_x; // unit: mm
        float distance_y; // unit: mm

        struct Wheel
        {
            steering::SteeringWheel::Config            cfg;
            steering::SteeringWheel::CalibrationConfig calib_cfg;
        };

        Wheel wheel_front_right;
        Wheel wheel_front_left;
        Wheel wheel_rear_left;
        Wheel wheel_rear_right;
    };

    struct KinematicParam
    {
        float distance_x; // unit: mm
        float distance_y; // unit: mm
        float y_shift;    // unit: mm
    };

    Special_Steering4(chassis_loc::ILoc& loc, const Config& cfg);

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

    [[nodiscard]] bool enabled() const override
    {
        return enabled_;
    }

    void startCalibration()
    {
        for (auto& w : wheel_)
            w.startCalibration();
    }

    [[nodiscard]] Velocity forwardGetVelocity() override
    {
        return velocity_;
    }

    // Runtime parameter input is mm; internally stored in meters.
    void setYShift(float y_shift)
    {
        y_shift_ = 1e-3f * y_shift;
    }

    [[nodiscard]] float yShift() const
    {
        return 1e3f * y_shift_;
    }

    [[nodiscard]] KinematicParam kinematicParam() const
    {
        return {
            1e3f * distance_x_,
            1e3f * distance_y_,
            1e3f * y_shift_,
        };
    }

protected:
    void applyVelocity(const Velocity& velocity) override;
    void velocityControllerUpdate() override;

private:
    struct WheelPosition
    {
        float x;
        float y;
    };

    [[nodiscard]] WheelPosition getWheelPosition(WheelType wheel) const;

    bool enabled_{ false };
    bool enable_calib_;
    bool calibrated_{ false };

    float wheel_radius_;
    float distance_x_;
    float distance_y_;
    float half_distance_x_;
    float half_distance_y_;
    float y_shift_{ 0.0f };
    float spd2rpm_;

    Velocity velocity_{};

    steering::SteeringWheel wheel_[static_cast<size_t>(WheelType::Max)];
};

} // namespace chassis
