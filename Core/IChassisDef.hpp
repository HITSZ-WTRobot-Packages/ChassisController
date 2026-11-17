/**
 * @file    IChassisDef.hpp
 * @author  syhanjin
 * @date    2026-03-15
 * @brief   ChassisController 对外统一使用的基础运动学数据结构。
 */
#pragma once
#include <cmath>
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

    static constexpr Velocity zero() { return { 0, 0, 0 }; }
};

/**
 * 底盘位姿。
 *
 * `x/y` 所处的世界坐标系由具体定位后端决定，但单位和旋向约定在整个模块内保持一致。
 * yaw 使用 deg，范围为 [-180, 180]；它表示朝向而非累计转角。
 * 构造及加减运算自动归一化 yaw；构造后直接写入公开字段时仍须遵守该范围。
 */
struct Posture
{
    float x;   ///< 指向车体前方 (unit: m)
    float y;   ///< 指向车体左侧 (unit: m)
    float yaw; ///< 向上（逆时针）为正，范围 [-180, 180] (unit: deg)

    /// 默认构造为零位姿。
    constexpr Posture() : Posture(0.0f, 0.0f, 0.0f) {}

    /// 使用 m、m、deg 构造位姿，yaw 自动归一化到 [-180, 180]。
    constexpr Posture(const float x, const float y, const float yaw) :
        x(x), y(y), yaw(smallestAngle(yaw))
    {
    }

    static constexpr Posture zero() { return { 0, 0, 0 }; }

    /// 将角度折返到 [-180, 180] deg；恰好半圈时保留输入的正负方向。
    static constexpr float smallestAngle(float yaw)
    {
        if (yaw >= -180.0f && yaw <= 180.0f)
            return yaw;
        // 合法 Posture 的加减只需折返一次，也保留这些运算的常量求值能力。
        while (yaw > 180.0f)
            yaw -= 360.0f;
        while (yaw < -180.0f)
            yaw += 360.0f;
        return yaw;
    }

    constexpr Posture operator+(const Posture& rhs) const
    {
        return { x + rhs.x, y + rhs.y, yaw + rhs.yaw };
    }

    constexpr Posture operator-(const Posture& rhs) const
    {
        return { x - rhs.x, y - rhs.y, yaw - rhs.yaw };
    }

    /// 返回 current - target 的劣弧有向角差，范围 [-180, 180] deg。
    /// 恰好相差 180 deg 时保留原差值的正负方向。
    static constexpr float yawError(const float current, const float target)
    {
        return smallestAngle(current - target);
    }
};

/**
 * 平面物理加速度及角加速度。
 *
 * ax/ay 是加速度向量在指定坐标系中的表达，不是旋转车体系速度分量的直接导数。
 */
struct Acceleration
{
    float ax;    ///< x 方向加速度 (unit: m/s^2)
    float ay;    ///< y 方向加速度 (unit: m/s^2)
    float alpha; ///< 逆时针角加速度 (unit: deg/s^2)

    static constexpr Acceleration zero() { return { 0, 0, 0 }; }
};
} // namespace chassis
