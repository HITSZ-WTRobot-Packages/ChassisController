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
 *
 * 基类只保留 enable / disable / stop 这几个最基本控制语义，其他控制能力由子类扩展。
 * 不同 Controller 也可能拆出多个独立更新入口，因此基类不统一要求单一 update()。
 */
class IChassisController
{
public:
    virtual ~IChassisController() = default;

    // 公共接口转发，方便上层通过 Controller 直接访问常用状态。
    [[nodiscard]] const auto& motion() const { return *motion_; }
    [[nodiscard]] const auto& loc() const { return *loc_; }
    [[nodiscard]] auto        velocityInBody() const { return loc_->velocityInBody(); }
    [[nodiscard]] auto        velocityInWorld() const { return loc_->velocityInWorld(); }
    [[nodiscard]] auto        postureInWorld() const { return loc_->postureInWorld(); }

    /**
     * 使能底盘并进入 stop 状态
     *
     * 这里的 Controller 应被理解为“更高一层的控制器封装”：
     * 调用它的 enable() 会继续把使能动作传递给下层 Motion，而 Motion
     * 又会继续把使能传递给更底层的轮组 / 电机控制器。
     *
     * 因此在正常接入里，业务层通常不需要再重复手动使能下层控制器。
     *
     * @return 是否成功 enable
     */
    virtual bool enable()
    {
        stop();
        return motion_->enable();
    }

    /// 直接关闭底盘执行器。
    void disable() { motion_->disable(); }

    /// 只有 Motion 已使能且 ready 时，控制器才认为自己可工作。
    [[nodiscard]] bool enabled() const { return motion_->enabled() && motion_->isReady(); }

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

    // 保护接口转发，只允许子类控制器把速度命令传给 Motion。
    void applyVelocity(const Velocity& velocity) { motion_->applyVelocity(velocity); }
};
} // namespace chassis::controller
