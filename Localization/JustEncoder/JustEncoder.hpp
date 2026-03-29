/**
 * @file    JustEncoder.hpp
 * @author  syhanjin
 * @date    2026-03-07
 */
#pragma once

#include "IChassisLoc.hpp"

namespace chassis::loc
{

class JustEncoder final : public IChassisLoc
{
public:
    using IChassisLoc::IChassisLoc;

    void update(float dt);

    [[nodiscard]] Velocity velocityInBody() const override { return velocity_.in_body; }
    [[nodiscard]] Velocity velocityInWorld() const override { return velocity_.in_world; }
    [[nodiscard]] Posture  postureInWorld() const override { return posture_.in_world; }

private:
    struct
    {
        Posture in_world;
    } posture_{};

    struct
    {
        Velocity in_world;
        Velocity in_body;
    } velocity_{};
};

} // namespace chassis::loc
