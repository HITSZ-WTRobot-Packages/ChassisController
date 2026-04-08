/**
 * @file    IChassisDef.hpp
 * @author  syhanjin
 * @date    2026-03-15
 * @brief   ChassisController 对外统一使用的基础运动学数据结构。
 */
#pragma once
namespace chassis
{
/**
 * 底盘速度。
 *
 * 这里约定的是“车体坐标系常用正方向”：
 * - x 轴指向车头前方
 * - y 轴指向车体左侧
 * - z 轴向上，逆时针旋转为正
 */
struct Velocity
{
    float vx; ///< 指向车体前方 (unit: m/s)
    float vy; ///< 指向车体左侧 (unit: m/s)
    float wz; ///< 向上（逆时针）为正 (unit: deg/s)
};

/**
 * 底盘位姿。
 *
 * `x/y` 所处的世界坐标系由具体定位后端决定，但单位和旋向约定在整个模块内保持一致。
 */
struct Posture
{
    float x;   ///< 指向车体前方 (unit: m)
    float y;   ///< 指向车体左侧 (unit: m)
    float yaw; ///< 向上（逆时针）为正 (unit: deg)
};
} // namespace chassis
