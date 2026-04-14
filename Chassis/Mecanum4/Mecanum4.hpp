/**
 * @file    Mecanum4.hpp
 * @author  syhanjin
 * @date    2026-01-31
 * @brief   四轮麦克纳姆底盘运动学实现。
 */
#ifndef MECANUM4_HPP
#define MECANUM4_HPP
#include "IChassisMotion.hpp"
#include <cstddef>
#include "motor_vel_controller.hpp"

namespace chassis::motion
{

/**
 * 四轮麦克纳姆底盘。
 *
 * 该类只关心统一底盘速度和四个轮速控制器之间的换算，不负责轨迹规划或定位融合。
 */
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
        ChassisType chassis_type;     ///< 底盘构型，决定正逆解算符号关系

        controllers::MotorVelController* wheel_front_right; ///< 右前方
        controllers::MotorVelController* wheel_front_left;  ///< 左前方
        controllers::MotorVelController* wheel_rear_left;   ///< 左后方
        controllers::MotorVelController* wheel_rear_right;  ///< 右后方
    };

    explicit Mecanum4(const Config& driver_cfg);

    bool enable() override;
    void disable() override;

    [[nodiscard]] bool enabled() const override { return enabled_; }

    /// 由四个轮子的反馈速度反解出底盘车体系速度。
    Velocity forwardGetVelocity() override;

    /// 周期调用底层轮速控制器更新。
    void update();

    /// 当前实现不存在额外就绪阶段，使能后即可作为完整底盘使用。
    [[nodiscard]] bool isReady() const override { return true; }

protected:
    /// 把底盘车体系速度换算成四个轮子的目标转速。
    void applyVelocity(const Velocity& velocity) override;

private:
    bool        enabled_{ false };
    ChassisType type_;         ///< 底盘构型
    float       wheel_radius_; ///< 轮子半径 (unit: m)
    float       k_omega_;      ///< 角速度项对应的等效力臂 (unit: m)

    controllers::MotorVelController* wheel_[static_cast<size_t>(WheelType::Max)]{};
};

} // namespace chassis::motion

#endif // MECANUM4_HPP
