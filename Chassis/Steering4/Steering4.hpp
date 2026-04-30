/**
 * @file    Steering4.hpp
 * @author  syhanjin
 * @date    2026-02-28
 * @brief   四舵轮底盘运动学实现。
 */
#pragma once
#include "IChassisMotion.hpp"
#include "SteeringWheel.hpp"

namespace chassis::motion
{

/**
 * 四舵轮底盘。
 *
 * 和普通全向轮不同，四舵轮在真正 ready 之前可能还要经历轮组校准阶段，
 * 所以上层必须同时关注 `enabled()` 和 `isReady()`。
 */
class Steering4 : public IChassisMotion
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
        bool enable_calibration = false; ///< 是否启用启动校准流程

        float radius;     ///< 驱动轮半径 (unit: mm)
        float distance_x; ///< 前后轮距 (unit: mm), x 轴指向车体前方
        float distance_y; ///< 左右轮距 (unit: mm), y 轴指向车体左侧

        struct Wheel
        {
            steering::SteeringWheel::Config            cfg;       ///< 正常运行配置
            steering::SteeringWheel::CalibrationConfig calib_cfg; ///< 校准配置
        };
        Wheel wheel_front_right; ///< 右前方
        Wheel wheel_front_left;  ///< 左前方
        Wheel wheel_rear_left;   ///< 左后方
        Wheel wheel_rear_right;  ///< 右后方
    };

    explicit Steering4(const Config& cfg);
    /// 依次使能四个轮组；任意一个失败都会整体回滚。
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

    /// 关闭四个轮组。
    void disable() override
    {
        for (auto& w : wheel_)
            w.disable();
        enabled_ = false;
    }

    [[nodiscard]] bool enabled() const override { return enabled_; }

    /**
     * 启动四个轮组的校准流程。
     *
     * 仅在 enable 成功之后才有意义。启用校准时，只有全部轮组完成校准后
     * `isReady()` 才会变为 true，在此之前控制器下发的底盘速度会被忽略。
     */
    void startCalibration()
    {
        for (auto& w : wheel_)
            w.startCalibration();
    }

    /// 返回四舵轮根据当前轮速和舵向反解出的底盘反馈速度。
    Velocity forwardGetVelocity() override { return velocity_; }

    /// 启用校准时，ready 表示四个轮组都已经建立舵向零点。
    [[nodiscard]] bool isReady() const override { return !enable_calib_ || calibrated_; }

    /// 周期刷新校准状态、轮组控制器以及反馈速度。
    void update();

protected:
    void applyVelocity(const Velocity& velocity) override;

private:
    bool enabled_{ false };
    bool enable_calib_;        ///< 是否启用启动校准
    bool calibrated_{ false }; ///< 四个轮组是否都已完成校准

    float wheel_radius_;   ///< 驱动轮半径 (unit: m)
    float half_distance_x; ///< 半前后轮距 (unit: m)
    float half_distance_y; ///< 半左右轮距 (unit: m)
    float inv_l2_;         ///< 角速度反解时使用的几何项倒数
    float spd2rpm_;        ///< 线速度与轮速 rpm 的换算系数

    Velocity velocity_{}; ///< 当前由四个轮组反解得到的反馈速度

    steering::SteeringWheel wheel_[static_cast<size_t>(WheelType::Max)];

    struct WheelPosition
    {
        float x, y;
    };

    /// 根据轮序返回该轮相对于底盘中心的位置。
    [[nodiscard]] WheelPosition getWheelPosition(WheelType wheel) const;
};

} // namespace chassis::motion
