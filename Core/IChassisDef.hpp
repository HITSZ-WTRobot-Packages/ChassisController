/**
 * @file    IChassisDef.hpp
 * @author  syhanjin
 * @date    2026-03-15
 */
#pragma once
namespace chassis
{
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
} // namespace chassis