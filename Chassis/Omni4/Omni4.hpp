/**
 * @file    Omni4.hpp
 * @author  ported by GitHub Copilot
 * @date    2026-03-08
 * @brief   四轮全向轮底盘运动学实现。
 */
#ifndef OMNI4_HPP
#define OMNI4_HPP

#include "IChassisMotion.hpp"
#include "motor_vel_controller.hpp"

#include <cstddef>

namespace chassis::motion
{

/**
 * 四轮全向轮底盘。
 *
 * 对外语义与其他 Motion 实现保持一致：上层只需要给底盘速度，不需要直接理解单轮运动学。
 */
class Omni4 : public IChassisMotion
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

    struct Config
    {
        float wheel_radius;     ///< 轮子半径 (unit: mm)
        float wheel_distance_x; ///< 前后轮距 (unit: mm), x 轴指向车体前方
        float wheel_distance_y; ///< 左右轮距 (unit: mm), y 轴指向车体左侧

        controllers::MotorVelController* wheel_front_right; ///< 右前方
        controllers::MotorVelController* wheel_front_left;  ///< 左前方
        controllers::MotorVelController* wheel_rear_left;   ///< 左后方
        controllers::MotorVelController* wheel_rear_right;  ///< 右后方
    };

    explicit Omni4(const Config& cfg);

    bool enable() override;
    void disable() override;

    [[nodiscard]] bool enabled() const override { return enabled_; }

    /// 由四个全向轮的反馈速度反解底盘车体系速度。
    Velocity forwardGetVelocity() override;

    /// 周期调用四个轮子的速度控制器。
    void update();

    /// 当前实现没有额外校准流程，使能后即可认为 ready。
    [[nodiscard]] bool isReady() const override { return true; }

protected:
    /// 把底盘速度换算为四个轮子的目标转速。
    void applyVelocity(const Velocity& velocity) override;

private:
    bool  enabled_{ false };
    float wheel_radius_; ///< 轮半径 (unit: m)
    float half_diag_;    ///< 轮心到底盘中心的等效半对角线长度 (unit: m)

    controllers::MotorVelController* wheel_[static_cast<size_t>(WheelType::Max)]{};
};

} // namespace chassis::motion

#endif // OMNI4_HPP
