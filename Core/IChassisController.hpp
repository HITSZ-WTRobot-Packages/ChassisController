/**
 * @file    IChassisController.hpp
 * @author  syhanjin
 * @date    2026-03-15
 * @brief   底盘控制层统一接口。
 */
#pragma once
#include "IChassisLoc.hpp"
#include "IChassisMotion.hpp"
namespace chassis::controller
{
/**
 * 业务层直接持有的控制器基类。
 *
 * Controller 同时依赖 Motion 和 Loc：
 * - Motion 负责真正把速度命令下发到轮组
 * - Loc 负责反馈底盘当前状态
 * - Controller 基类负责控制权申请/释放，保证同一时刻只有一个控制器真正驱动底盘
 *
 * 基类保留 acquireControl / releaseControl / enable / disable / stop 这些最基本控制语义，
 * 其他控制能力由子类扩展。
 * 不同 Controller 也可能拆出多个独立更新入口，因此基类不统一要求单一 update()。
 */
class IChassisController
{
public:
    virtual ~IChassisController() { IChassisController::releaseControl(); }

    // 公共接口转发，方便上层通过 Controller 直接访问常用状态。
    [[nodiscard]] const auto& motion() const { return *motion_; }
    [[nodiscard]] const auto& loc() const { return *loc_; }
    [[nodiscard]] auto        velocityInBody() const { return loc_->velocityInBody(); }
    [[nodiscard]] auto        velocityInWorld() const { return loc_->velocityInWorld(); }
    [[nodiscard]] auto        postureInWorld() const { return loc_->postureInWorld(); }

    /**
     * 申请底盘控制权并进入 stop 状态。
     *
     * `acquireControl()` 只做控制权交接，不负责底盘执行器上电。
     * 成功获取控制权后会立即 `stop()`，保证新接手的控制器从安全状态开始工作。
     *
     * @return 是否成功获取控制权
     */
    virtual bool acquireControl()
    {
        if (motion_ == nullptr || !motion_->tryAcquireController(this))
            return false;

        stop();
        return true;
    }

    /**
     * 释放底盘控制权，但不关闭底层执行器。
     */
    virtual void releaseControl()
    {
        if (motion_ != nullptr)
            motion_->releaseController(this);
    }

    /**
     * 使能底盘并申请控制权
     *
     * 这里的 Controller 应被理解为“更高一层的控制器封装”：
     * 调用它的 enable() 会继续把使能动作传递给下层 Motion，而 Motion
     * 又会继续把使能传递给更底层的轮组 / 电机控制器。
     *
     * 因此在正常接入里，业务层通常不需要再重复手动使能下层控制器。
     *
     * @return 是否成功使能底盘并获取控制权
     */
    virtual bool enable()
    {
        if (motion_ == nullptr)
            return false;

        const auto* owner = motion_->currentController();
        if (owner != nullptr && owner != this)
            return false;

        if (!motion_->enable())
            return false;

        return acquireControl();
    }

    /// 直接关闭底盘执行器，同时释放当前控制器对底盘的控制权。
    virtual void disable()
    {
        releaseControl();
        if (motion_ != nullptr)
            motion_->disable();
    }

    /// 当前 Controller 是否持有底盘控制权。
    [[nodiscard]] bool hasControl() const
    {
        return motion_ != nullptr && motion_->currentController() == this;
    }

    /// 只有持有控制权，且底层 Motion 已使能并 ready 时，控制器才认为自己可工作。
    [[nodiscard]] bool enabled() const
    {
        return hasControl() && motion_->enabled() && motion_->isReady();
    }

    /**
     * 静止底盘
     *
     * stop 状态应当是控制 motion 锁定在当前位置，即位置环锁定状态
     */
    virtual void stop() = 0;

protected:
    /// Controller 在构造时就必须绑定现成的 Motion 和 Loc。
    IChassisController(motion::IChassisMotion& chassis_motion, loc::IChassisLoc& chassis_loc) :
        motion_(&chassis_motion), loc_(&chassis_loc)
    {
    }

    motion::IChassisMotion* motion_; ///< 真正执行底盘速度命令的运动学层
    loc::IChassisLoc*       loc_;    ///< 提供底盘状态反馈的定位层

    // 保护接口转发，只允许当前持有控制权的控制器把速度命令传给 Motion。
    void applyVelocity(const Velocity& velocity)
    {
        if (hasControl())
            motion_->applyVelocity(velocity);
    }
};
} // namespace chassis::controller
