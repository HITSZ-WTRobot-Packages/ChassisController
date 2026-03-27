/**
 * @file    Mecanum4.hpp
 * @author  syhanjin
 * @date    2026-01-31
 */
#ifndef MECANUM4_HPP
#define MECANUM4_HPP
#include "IChassisMotion.hpp"
#include <cstddef>
#include "motor_vel_controller.hpp"

namespace chassis::motion
{

class Mecanum4 : public IChassisMotion
{
public:
    enum class WheelType : size_t
    {
        FrontRight = 0U, ///< 右前轮
        FrontLeft,       ///< 左前轮
        RearLeft,        ///< 左后轮
        RearRight,       ///< 右后轮
        Max
    };
    enum class ChassisType
    {
        XType = 0U, ///< X 型布局
        OType,      ///< O 型布局
    };

    struct Config
    {
        float       wheel_radius;     ///< 轮子半径 (unit: mm)
        float       wheel_distance_x; ///< 前后轮距 (unit: mm), x 轴指向车体前方
        float       wheel_distance_y; ///< 左右轮距 (unit: mm), y 轴指向车体左侧
        ChassisType chassis_type;     ///< 底盘构型

        controllers::MotorVelController* wheel_front_right; ///< 右前方
        controllers::MotorVelController* wheel_front_left;  ///< 左前方
        controllers::MotorVelController* wheel_rear_left;   ///< 左后方
        controllers::MotorVelController* wheel_rear_right;  ///< 右后方
    };

    explicit Mecanum4(const Config& driver_cfg);

    bool enable() override;
    void disable() override;

    [[nodiscard]] bool enabled() const override { return enabled_; }

    Velocity forwardGetVelocity() override;

    void update();

    [[nodiscard]] bool isReady() const override { return true; }

protected:
    void applyVelocity(const Velocity& velocity) override;

private:
    bool        enabled_{ false };
    ChassisType type_;         ///< 底盘构型
    float       wheel_radius_; ///< 轮子半径 (unit: m)
    float       k_omega_;      ///< O 型：半前后 + 半左右；X 型：半左右 - 半前后 (unit: m)

    controllers::MotorVelController* wheel_[static_cast<size_t>(WheelType::Max)]{};
};

} // namespace chassis::motion

#endif // MECANUM4_HPP
