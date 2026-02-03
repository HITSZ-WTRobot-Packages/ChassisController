/**
 * @file    Mecanum4.hpp
 * @author  syhanjin
 * @date    2026-01-31
 */
#ifndef MECANUM4_HPP
#define MECANUM4_HPP
#include "IChassis.hpp"
#include <cstddef>
#include "motor_vel_controller.hpp"

/**
 * rad/s to round/min
 * @param __RPS__ rad/s
 */
#define RPS2RPM(__RPS__) ((__RPS__) * 60.0f / (2 * 3.14159265358979323846f))

#define DEG2RAD(__DEG__) ((__DEG__) * (float) 3.14159265358979323846f / 180.0f)

#define RPM2DPS(__RPM__) ((__RPM__) / 60.0f * 360.0f)

namespace chassis
{

class Mecanum4 final : public IChassis
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
        float       wheel_distance_x; ///< 左右轮距 (unit: mm)
        float       wheel_distance_y; ///< 前后轮距 (unit: mm)
        ChassisType chassis_type;     ///< 底盘构型

        controllers::MotorVelController* wheel_front_right; ///< 右前方
        controllers::MotorVelController* wheel_front_left;  ///< 左前方
        controllers::MotorVelController* wheel_rear_left;   ///< 左后方
        controllers::MotorVelController* wheel_rear_right;  ///< 右后方
    };

    Mecanum4(const Config& driver_cfg, const IChassis::Config& base_cfg);
    bool postEnable() override;
    void postDisable() override;
    bool enabled() const override
    {
        return enabled_;
    }

protected:
    void  applyVelocity(const Velocity& velocity) override;
    void  velocityControllerUpdate() override;
    float forwardGetX() override;
    float forwardGetY() override;
    float forwardGetYaw() override;
    float forwardGetVx() override;
    float forwardGetVy() override;
    float forwardGetWz() override;

private:
    bool        enabled_{ false };
    ChassisType type_;         ///< 底盘构型
    float       wheel_radius_; ///< 轮子半径 (unit: m)
    float       k_omega_;      ///< O 型：半宽 + 半高；X 型：半宽 - 半高 (unit: m)

    controllers::MotorVelController* wheel_[static_cast<size_t>(WheelType::Max)]{};
};

} // namespace chassis

#endif // MECANUM4_HPP
