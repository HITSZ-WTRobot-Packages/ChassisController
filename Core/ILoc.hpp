/**
 * @file    ILoc.hpp
 * @author  syhanjin
 * @date    2026-03-07
 */
#pragma once

namespace chassis
{
class IChassis;
}

namespace chassis_loc
{

class ILoc
{
public:
    struct Velocity
    {
        float vx; ///< 指向车体前方 (unit: m/s)
        float vy; ///< 指向车体左侧 (unit: m/s)
        float wz; ///< 向上（逆时针）为正 (unit: deg/s)
    };

    struct Posture
    {
        float x;   ///< 指向车体前方 (unit: m)
        float y;   ///< 指向车体左侧 (unit: m)
        float yaw; ///< 向上（逆时针）为正 (unit: deg)
    };

    explicit ILoc() = default;
    virtual ~ILoc() = default;

    [[nodiscard]] const auto& velocity() const
    {
        return velocity_;
    }

    [[nodiscard]] const auto& posture() const
    {
        return posture_;
    }

    [[nodiscard]] Velocity WorldVelocity2BodyVelocity(const Velocity& velocity_in_world) const;
    [[nodiscard]] Velocity BodyVelocity2WorldVelocity(const Velocity& velocity_in_body) const;
    [[nodiscard]] Posture  WorldPosture2BodyPosture(const Posture& posture_in_world) const;
    [[nodiscard]] Posture  BodyPosture2WorldPosture(const Posture& posture_in_body) const;

    void bind_chassis(chassis::IChassis* chassis)
    {
        chassis_ = chassis;
    }

protected:
    chassis::IChassis* chassis_{ nullptr };

    struct
    {
        Velocity in_body{};
        Velocity in_world{};
    } velocity_{};

    struct
    {
        Posture in_world{};
    } posture_{};
};

} // namespace chassis_loc
