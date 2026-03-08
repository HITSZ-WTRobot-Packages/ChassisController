/**
 * @file    IChassis.hpp
 * @author  syhanjin
 * @date    2026-01-31
 */
#pragma once
#include "ILoc.hpp"
namespace chassis
{

class IChassis
{
public:
    using Velocity = chassis_loc::ILoc::Velocity;
    using Posture  = chassis_loc::ILoc::Posture;

    virtual ~IChassis()                  = default;
    [[nodiscard]] virtual bool enable()  = 0;
    virtual void               disable() = 0;
    [[nodiscard]] virtual bool enabled() const
    {
        return false;
    }

    [[nodiscard]] const chassis_loc::ILoc& loc() const
    {
        return loc_;
    }

    [[nodiscard]] const auto& posture() const
    {
        return loc().posture();
    }
    [[nodiscard]] const auto& velocity() const
    {
        return loc().velocity();
    }

    [[nodiscard]] Velocity WorldVelocity2BodyVelocity(const Velocity& velocity_in_world) const
    {
        return loc_.WorldVelocity2BodyVelocity(velocity_in_world);
    }
    [[nodiscard]] Velocity BodyVelocity2WorldVelocity(const Velocity& velocity_in_body) const
    {
        return loc_.BodyVelocity2WorldVelocity(velocity_in_body);
    }
    [[nodiscard]] Posture WorldPosture2BodyPosture(const Posture& posture_in_world) const
    {
        return loc_.WorldPosture2BodyPosture(posture_in_world);
    }
    [[nodiscard]] Posture BodyPosture2WorldPosture(const Posture& posture_in_body) const
    {
        return loc_.BodyPosture2WorldPosture(posture_in_body);
    }

    virtual Velocity forwardGetVelocity() = 0;

protected:
    friend chassis_loc::ILoc;
    explicit IChassis(chassis_loc::ILoc& loc) : loc_(loc)
    {
        loc_.bind_chassis(this);
    }

    virtual void applyVelocity(const Velocity& velocity) = 0;
    virtual void velocityControllerUpdate()              = 0;

private:
    chassis_loc::ILoc& loc_;
};

} // namespace chassis
