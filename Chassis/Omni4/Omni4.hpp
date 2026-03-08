/**
 * @file    Omni4.hpp
 * @author  ported by GitHub Copilot
 * @date    2026-03-08
 */
#ifndef OMNI4_HPP
#define OMNI4_HPP

#include "IChassis.hpp"
#include "motor_vel_controller.hpp"

#include <cstddef>

namespace chassis
{

class Omni4 : public IChassis
{
public:
    enum class WheelType : size_t
    {
        FrontRight = 0U,
        FrontLeft,
        RearLeft,
        RearRight,
        Max
    };

    struct Config
    {
        float wheel_radius;     ///< 轮子半径 (unit: mm)
        float wheel_distance_x; ///< 前后轮距 (unit: mm), x 轴指向车体前方
        float wheel_distance_y; ///< 左右轮距 (unit: mm), y 轴指向车体左侧

        controllers::MotorVelController* wheel_front_right;
        controllers::MotorVelController* wheel_front_left;
        controllers::MotorVelController* wheel_rear_left;
        controllers::MotorVelController* wheel_rear_right;
    };

    Omni4(chassis_loc::ILoc& loc, const Config& cfg);

    bool enable() override;
    void disable() override;

    [[nodiscard]] bool enabled() const override
    {
        return enabled_;
    }

    Velocity forwardGetVelocity() override;

protected:
    void applyVelocity(const Velocity& velocity) override;
    void velocityControllerUpdate() override;

private:
    bool  enabled_{ false };
    float wheel_radius_; ///< unit: m
    float half_diag_;    ///< unit: m

    controllers::MotorVelController* wheel_[static_cast<size_t>(WheelType::Max)]{};
};

} // namespace chassis

#endif // OMNI4_HPP
