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
    // 公共接口转发
    [[nodiscard]] const auto& motion() const { return *motion_; }
    [[nodiscard]] const auto& loc() const { return *loc_; }
    [[nodiscard]] const auto& velocityInBody() const { return loc_->velocityInBody(); }
    [[nodiscard]] const auto& velocityInWorld() const { return loc_->velocityInWorld(); }
    [[nodiscard]] const auto& postureInWorld() const { return loc_->postureInWorld(); }

    [[nodiscard]] bool enable() { return motion_->enable(); }
    void               disable() { motion_->disable(); }
    [[nodiscard]] bool enabled() const { return motion_->enabled(); }

protected:
    IChassisController(motion::IChassisMotion& chassis_motion, loc::IChassisLoc& chassis_loc) :
        motion_(&chassis_motion), loc_(&chassis_loc)
    {
    }

    motion::IChassisMotion* motion_;
    loc::IChassisLoc*       loc_;

    // 保护接口转发
    void applyVelocity(const Velocity& velocity) { motion_->applyVelocity(velocity); }
    void velocityControllerUpdate() { motion_->velocityControllerUpdate(); }
};
} // namespace chassis::controller