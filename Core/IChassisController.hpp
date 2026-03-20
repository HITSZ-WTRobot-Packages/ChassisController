/**
 * @file    IChassisController.hpp
 * @author  syhanjin
 * @date    2026-03-15
 */
#pragma once
#include "IChassisLoc.hpp"
#include "IChassisMotion.hpp"
namespace chassis::controller
{
class IChassisController
{
public:
    virtual ~IChassisController() = default;

    // 公共接口转发
    [[nodiscard]] const auto& motion() const { return *motion_; }
    [[nodiscard]] const auto& loc() const { return *loc_; }
    [[nodiscard]] auto        velocityInBody() const { return loc_->velocityInBody(); }
    [[nodiscard]] auto        velocityInWorld() const { return loc_->velocityInWorld(); }
    [[nodiscard]] auto        postureInWorld() const { return loc_->postureInWorld(); }

    /**
     * 使能底盘并进入 stop 状态
     * @return 是否成功 enable
     */
    virtual bool enable()
    {
        stop();
        return motion_->enable();
    }

    void disable() { motion_->disable(); }

    [[nodiscard]] bool enabled() const { return motion_->enabled() && motion_->isReady(); }

    /**
     * 静止底盘
     *
     * stop 状态应当是控制 motion 锁定在当前位置，即位置环锁定状态
     */
    virtual void stop() = 0;

protected:
    IChassisController(motion::IChassisMotion& chassis_motion, loc::IChassisLoc& chassis_loc) :
        motion_(&chassis_motion), loc_(&chassis_loc)
    {
    }

    motion::IChassisMotion* motion_;
    loc::IChassisLoc*       loc_;

    // 保护接口转发
    void applyVelocity(const Velocity& velocity) { motion_->applyVelocity(velocity); }
};
} // namespace chassis::controller