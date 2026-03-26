/**
 * @file    Special_Steering4.hpp
 * @author  rediduck
 * @date    2026-03-26
 */
#pragma once

#include "IChassisMotion.hpp"
#include "Special_SteeringWheel.hpp"

namespace chassis::motion
{

class Special_Steering4 : public IChassisMotion
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
            steering::Special_SteeringWheel::Config            cfg;
            steering::Special_SteeringWheel::CalibrationConfig calib_cfg;
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

    explicit Special_Steering4(const Config& cfg);
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

    [[nodiscard]] bool enabled() const override { return enabled_; }

    void startCalibration()
    {
        for (auto& w : wheel_)
            w.startCalibration();
    }

    [[nodiscard]] Velocity forwardGetVelocity() override { return velocity_; }

    [[nodiscard]] bool isReady() const override { return !enable_calib_ || calibrated_; }

    // Runtime parameter input is mm; internally stored in meters.
    void setYShift(float y_shift) { y_shift_ = 1e-3f * y_shift; }

    [[nodiscard]] float yShift() const { return 1e3f * y_shift_; }

    [[nodiscard]] KinematicParam kinematicParam() const
    {
        return {
            1e3f * distance_x_,
            1e3f * distance_y_,
            1e3f * y_shift_,
        };
    }

    void update() override;

protected:
    void applyVelocity(const Velocity& velocity) override;

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
    float inv_l2_;
    float spd2rpm_;

    Velocity velocity_{};

    steering::Special_SteeringWheel wheel_[static_cast<size_t>(WheelType::Max)];
};

} // namespace chassis::motion
