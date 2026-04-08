/**
 * @file    JustEncoder.hpp
 * @author  syhanjin
 * @date    2026-03-07
 * @brief   仅依赖底盘速度反馈的简单积分定位。
 */
#pragma once

#include "IChassisLoc.hpp"

namespace chassis::loc
{

/**
 * 纯编码器/纯运动学积分定位。
 *
 * 它不引入外部绝对观测，只把 Motion 反馈速度按时间积分成世界系位姿，
 * 适合做最轻量的定位后端，或者作为更复杂定位系统的基线方案。
 */
class JustEncoder final : public IChassisLoc
{
public:
    using IChassisLoc::IChassisLoc;

    /**
     * 仅依赖 Motion 反馈速度做积分定位。
     * @param dt 周期时长，单位为秒
     */
    void update(float dt);

    [[nodiscard]] Velocity velocityInBody() const override { return velocity_.in_body; }
    [[nodiscard]] Velocity velocityInWorld() const override { return velocity_.in_world; }
    [[nodiscard]] Posture  postureInWorld() const override { return posture_.in_world; }

private:
    struct
    {
        Posture in_world; ///< 当前积分得到的世界系位姿
    } posture_{};

    struct
    {
        Velocity in_world; ///< 当前世界系速度
        Velocity in_body;  ///< 当前车体系速度
    } velocity_{};
};

} // namespace chassis::loc
