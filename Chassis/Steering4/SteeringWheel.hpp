/**
 * @file    SteeringWheel.hpp
 * @author  syhanjin
 * @date    2026-02-28
 * @brief   单个舵轮模块的控制与校准逻辑。
 */
#pragma once
#include "motor_pos_controller.hpp"
#include "motor_vel_controller.hpp"
#include "gpio_driver.h"

namespace chassis::steering
{

/**
 * 单个舵轮。
 *
 * 每个轮组同时拥有：
 * - 一个驱动轮速度控制器
 * - 一个舵向位置控制器
 *
 * 如果启用校准，还会额外使用一个舵向速度控制器和光电门来建立零点。
 */
class SteeringWheel
{
public:
    struct Velocity
    {
        float angle; ///< 舵向角度，行进正前方为零点，逆时针为正    (unit: deg)
        float speed; ///< 轮向速度，angle = 0 时向前运动的方向为正 (unit: rpm)
    };
    enum class CalibState
    {
        Idle,        ///< 尚未开始校准
        SeekGate,    ///< 高速寻找光电门，先确认已经找到光电门区域
        FineCapture, ///< 低速精确捕获零点，固定使用某一个边缘作为零点参考
        Done,        ///< 校准完成
    };

    struct CalibrationConfig
    {
        controllers::MotorVelController* steer_motor = nullptr;        ///< 校准阶段使用的舵向速度环
        GPIO_t                           photogate   = {};             ///< 光电门 GPIO；若依赖 EXTI 回调，需在 CubeMX 中配置为双边沿触发
        GPIO_PinState photogate_active_state         = GPIO_PIN_RESET; ///< 光电门有效电平，用于判断当前是否处于挡光区域
    };

    struct Config
    {
        controllers::MotorVelController* drive_motor; ///< 轮向电机速度环控制器
        controllers::MotorPosController* steer_motor; ///< 舵向电机位置环控制器
        float steer_offset; ///< 舵向偏置；如果有光电门，则表示光电门所在角度；如果用的云台电机，则是云台电机零点位置
    };

    explicit SteeringWheel(const Config& cfg, bool enable_calib, const CalibrationConfig&);

    /// 启动零点校准流程。只有在已使能且还未开始校准时才会生效。
    void startCalibration();
    /// 设置该轮组的目标舵向角和轮向速度。
    void setTargetVelocity(const Velocity& vel);

    /// 读取去除零点偏置后的舵向角。
    [[nodiscard]] float getSteerAngle() const
    {
        return toSteerAngle(cfg_.steer_motor->getMotor()->getAngle());
    }

    /// 读取轮向电机速度反馈。
    [[nodiscard]] float getDriveSpeed() const
    {
        return cfg_.drive_motor->getMotor()->getVelocity();
    }

    /// 同时使能舵向位置环和轮向速度环，任意一个失败都会回滚。
    [[nodiscard]] bool enable() const
    {
        const auto steer_enabled = cfg_.steer_motor->enable();
        const auto drive_enabled = cfg_.drive_motor->enable();
        if (steer_enabled && drive_enabled)
            return true;
        if (steer_enabled && !drive_enabled)
            cfg_.steer_motor->disable();
        if (drive_enabled && !steer_enabled)
            cfg_.drive_motor->disable();
        return false;
    }

    /// 关闭正常运行使用的两个控制器。
    void disable() const
    {
        cfg_.steer_motor->disable();
        cfg_.drive_motor->disable();
    }

    /// 仅表示正常运行的两个控制器已经使能，不代表校准已经完成。
    [[nodiscard]] bool enabled() const
    {
        return cfg_.drive_motor->enabled() && cfg_.steer_motor->enabled();
    }

    /// 只有校准状态机进入 Done，才认为该轮组已经建立零点。
    [[nodiscard]] bool isCalibrated() const { return calib_state_ == CalibState::Done; }

    /// 周期更新舵向位置环、轮向速度环，以及校准阶段用到的速度环。
    void update() const;

private:
    Config cfg_;

    bool              enable_calib_;
    CalibrationConfig calib_cfg_;
    CalibState        calib_state_{ CalibState::Idle };

    Velocity target_vel_{}; ///< 最近一次设置后的目标速度

    static void PhotogateCallback(const GPIO_t* gpio, uint32_t counter, void* data)
    {
        static_cast<SteeringWheel*>(data)->photogateTrigger();
    }

    /// 处理光电门触发后的校准状态流转。
    /// 当前实现默认底层 EXTI 已配置为双边沿触发，这样挡光区的两个边缘都会回调进来。
    /// 当前逻辑故意使用两次触发：第一次说明进入了光电门区域，第二次才固定到某个边缘，
    /// 避免零点有时落在进入边缘、有时落在离开边缘。
    void photogateTrigger();

    /// 把对外可理解的舵向角转换成底层电机角度。
    [[nodiscard]] float toMotorAngle(const float steer_angle) const
    {
        return steer_angle + cfg_.steer_offset;
    }

    /// 把底层电机角度转换成去偏置后的舵向角。
    [[nodiscard]] float toSteerAngle(const float motor_angle) const
    {
        return motor_angle - cfg_.steer_offset;
    }

    /// 在保持等效运动的前提下，选择舵向变化更小的目标解。
    [[nodiscard]] Velocity toBestVelocity(Velocity velocity) const;
};

} // namespace chassis::steering
