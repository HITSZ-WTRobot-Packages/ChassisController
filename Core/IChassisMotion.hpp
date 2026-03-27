/**
 * @file    IChassisMotion.hpp
 * @author  syhanjin
 * @date    2026-01-31
 */
#pragma once
#include "IChassisDef.hpp"

namespace chassis::controller
{
class IChassisController;
}

namespace chassis::motion
{
/**
 * 底盘动作系统，仅用于底盘正逆解算，和控制轮子
 */
class IChassisMotion
{
public:
    virtual ~IChassisMotion()            = default;
    [[nodiscard]] virtual bool enable()  = 0;
    virtual void               disable() = 0;
    [[nodiscard]] virtual bool enabled() const { return false; }

    /**
     * 是否就位
     * 自检/变形过程中处于 not-ready 状态
     */
    [[nodiscard]] virtual bool isReady() const { return false; }

    virtual Velocity forwardGetVelocity() = 0;

protected:
    explicit IChassisMotion() {}

    // 限制只能由控制器调用
    virtual void applyVelocity(const Velocity& velocity) = 0;

    friend class controller::IChassisController;
};

} // namespace chassis::motion
