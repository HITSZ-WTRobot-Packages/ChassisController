/**
 * @file    IChassisLoc.hpp
 * @author  syhanjin
 * @date    2026-03-07
 * @brief   底盘定位层统一接口与坐标变换工具。
 */
#pragma once
#include "IChassisDef.hpp"
#include "IChassisMotion.hpp"

#include <cmath>

#ifndef DEG2RAD
#    define DEG2RAD(__DEG__) ((__DEG__) * (float)3.14159265358979323846f / 180.0f)
#endif

namespace chassis::loc
{

/**
 * 定位层基类。
 *
 * 它对外统一暴露：
 * - 世界系位姿
 * - 车体系速度
 * - 世界系速度
 *
 * 同时封装了一组常用坐标变换，避免每个控制器或业务模块都重复写一遍旋转公式。
 *
 * 在接入层里，某些 Loc 后端可能要等到校准完成或首次外部结果到达后才真正构造，
 * 因此业务代码里常会把 Loc 指针先置空，并在周期调用前先检查是否已经构造完成。
 *
 * 注意：本基类同样不统一规定 update() 形式。不同 Loc 后端可能需要
 * `update(dt)`、`update()`、`updateLidar(...)` 等不同入口，具体调用方式由
 * 接入方根据具体实现自行安排。
 */
class IChassisLoc
{
public:
    /// Loc 虽然可能还依赖外部传感器，但一定依赖一个 Motion 作为速度输入来源。
    explicit IChassisLoc(motion::IChassisMotion& motion) : motion_(&motion) {}

    virtual ~IChassisLoc() = default;

    /// 返回车体坐标系速度。
    [[nodiscard]] virtual Velocity velocityInBody() const  = 0;
    /// 返回世界坐标系速度。
    [[nodiscard]] virtual Velocity velocityInWorld() const = 0;

    /// 返回世界坐标系位姿。
    [[nodiscard]] virtual Posture postureInWorld() const = 0;

    /// 使用当前朝向，把世界系速度转换为车体系速度。
    [[nodiscard]] Velocity WorldVelocity2BodyVelocity(const Velocity& velocity_in_world) const
    {
        return rotateVelocity(velocity_in_world, -postureInWorld().yaw);
    }

    /// 使用当前朝向，把车体系速度转换为世界系速度。
    [[nodiscard]] Velocity BodyVelocity2WorldVelocity(const Velocity& velocity_in_body) const
    {
        return rotateVelocity(velocity_in_body, postureInWorld().yaw);
    }

    /// 把世界系位姿换算成“以当前车体为参考”的相对位姿。
    [[nodiscard]] Posture WorldPosture2BodyPosture(const Posture& posture_in_world) const
    {
        const float _sin_yaw = sinf(DEG2RAD(-postureInWorld().yaw)),
                    _cos_yaw = cosf(DEG2RAD(-postureInWorld().yaw));

        const float tx = posture_in_world.x - postureInWorld().x;
        const float ty = posture_in_world.y - postureInWorld().y;

        const Posture posture_in_body = {
            .x   = tx * _cos_yaw - ty * _sin_yaw,
            .y   = tx * _sin_yaw + ty * _cos_yaw,
            .yaw = posture_in_world.yaw - postureInWorld().yaw,
        };

        return posture_in_body;
    }

    /// 把“相对当前车体”的位姿换算回世界系。
    [[nodiscard]] Posture BodyPosture2WorldPosture(const Posture& posture_in_body) const
    {
        const float sin_yaw            = sinf(DEG2RAD(postureInWorld().yaw)),
                    cos_yaw            = cosf(DEG2RAD(postureInWorld().yaw));
        const Posture posture_in_world = {
            .x   = posture_in_body.x * cos_yaw - posture_in_body.y * sin_yaw + postureInWorld().x,
            .y   = posture_in_body.x * sin_yaw + posture_in_body.y * cos_yaw + postureInWorld().y,
            .yaw = posture_in_body.yaw + postureInWorld().yaw,
        };

        return posture_in_world;
    }

    /// 已知某个基准位姿在世界系中的位置，把“相对基准”的位姿展开到世界系。
    [[nodiscard]] static Posture RelativePosture2WorldPosture(const Posture& base_in_world,
                                                              const Posture& posture_in_base)
    {
        const float sin_yaw = sinf(DEG2RAD(base_in_world.yaw));
        const float cos_yaw = cosf(DEG2RAD(base_in_world.yaw));

        return {
            .x   = posture_in_base.x * cos_yaw - posture_in_base.y * sin_yaw + base_in_world.x,
            .y   = posture_in_base.x * sin_yaw + posture_in_base.y * cos_yaw + base_in_world.y,
            .yaw = posture_in_base.yaw + base_in_world.yaw,
        };
    }

    /// 已知某个基准位姿在世界系中的位置，把世界系位姿改写成“相对基准”的位姿。
    [[nodiscard]] static Posture WorldPosture2RelativePosture(const Posture& base_in_world,
                                                              const Posture& posture_in_world)
    {
        const float sin_yaw = sinf(DEG2RAD(-base_in_world.yaw));
        const float cos_yaw = cosf(DEG2RAD(-base_in_world.yaw));

        const float tx = posture_in_world.x - base_in_world.x;
        const float ty = posture_in_world.y - base_in_world.y;

        return {
            .x   = tx * cos_yaw - ty * sin_yaw,
            .y   = tx * sin_yaw + ty * cos_yaw,
            .yaw = posture_in_world.yaw - base_in_world.yaw,
        };
    }

    /// 便捷函数：计算“当前位置相对于某个世界系基准位姿”的相对位姿。
    [[nodiscard]] Posture CurrentPostureRelativeTo(const Posture& base_in_world) const
    {
        return WorldPosture2RelativePosture(base_in_world, postureInWorld());
    }

protected:
    motion::IChassisMotion* motion_{ nullptr }; ///< Loc 读取速度反馈所依赖的 Motion

    /// 受保护转发：读取 Motion 给出的车体系反馈速度。
    [[nodiscard]] auto forwardGetVelocity() const { return motion_->forwardGetVelocity(); }

    /// 二维平面旋转速度向量。角度单位仍是度，内部会转换成弧度。
    [[nodiscard]] static Velocity rotateVelocity(const Velocity& inp, const float theta)
    {
        const float sin_yaw = sinf(DEG2RAD(theta)), cos_yaw = cosf(DEG2RAD(theta));

        const Velocity out = {
            .vx = inp.vx * cos_yaw - inp.vy * sin_yaw,
            .vy = inp.vx * sin_yaw + inp.vy * cos_yaw,
            .wz = inp.wz,
        };

        return out;
    }
};

} // namespace chassis::loc
