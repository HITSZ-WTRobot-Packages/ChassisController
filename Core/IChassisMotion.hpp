/**
 * @file    IChassisMotion.hpp
 * @author  syhanjin
 * @date    2026-01-31
 * @brief   底盘运动学层统一接口。
 */
#pragma once
#include "IChassisDef.hpp"

namespace chassis::controller
{
class IChassisController;
}

namespace chassis::motion
{
/**
 * 底盘动作系统，仅负责：
 * - 把底层轮组组织成统一的“底盘速度输入 / 反馈”模型
 * - 执行正逆解算
 * - 管理底层执行器的使能状态
 *
 * 它不直接暴露给业务层发送速度命令；真正对外的控制入口应是 Controller。
 *
 * 这里的 Motion 只抽象“全向底盘平面运动”本身。若某些特殊底盘还带有升降、
 * 伸缩等与平面运动独立的自由度，这些自由度不应直接并入本接口，而应在其他
 * 模块中独立抽象后，再与本仓库提供的底盘运动能力组合。
 *
 * 注意：本基类故意不定义统一的 update() 约束。不同 Motion 实现可能需要
 * 不同参数、不同频率、甚至不同名字的更新入口，调用者应按具体实现自行调度。
 */
class IChassisMotion
{
public:
    virtual ~IChassisMotion()            = default;
    /// 使能底层执行器，返回是否成功进入可工作状态。
    [[nodiscard]] virtual bool enable()  = 0;
    /// 关闭底层执行器。
    virtual void               disable() = 0;
    /// 查询底层执行器是否已经处于使能状态。
    [[nodiscard]] virtual bool enabled() const = 0;

    /**
     * 是否就位
     *
     * `enabled()` 只表示执行器已经上电/使能；`isReady()` 进一步表示该 Motion
     * 已经可以被当成完整底盘使用。像舵轮校准、机构变形、自检这类过程，
     * 都可能出现“enabled 但 not-ready”。
     */
    [[nodiscard]] virtual bool isReady() const = 0;

    /// 正向解算得到当前底盘在车体坐标系下的反馈速度。
    virtual Velocity forwardGetVelocity() = 0;

    /**
     * 尝试让某个 Controller 获取该 Motion 的控制权。
     *
     * 同一时刻只允许一个 Controller 真正向 Motion 下发速度命令，
     * 用来避免多个控制器并发写目标。
     */
    virtual bool tryAcquireController(controller::IChassisController* ctrl)
    {
        if (controller_ == nullptr)
        {
            controller_ = ctrl;
            return true;
        }
        return controller_ == ctrl; // re-acquire allowed
    }

    /// 释放控制权；只有当前持有者自己可以释放。
    virtual void releaseController(controller::IChassisController* ctrl)
    {
        if (controller_ == ctrl)
            controller_ = nullptr;
    }

    /// 当前持有该 Motion 控制权的 Controller。
    [[nodiscard]] controller::IChassisController* currentController() const
    {
        return controller_;
    }

protected:
    explicit IChassisMotion() {}

    // 限制只能由控制器调用，避免业务层绕开控制模式直接下发底盘速度。
    virtual void applyVelocity(const Velocity& velocity) = 0;

    friend class controller::IChassisController;

private:
    controller::IChassisController* controller_{ nullptr }; ///< 当前持有控制权的控制器
};

} // namespace chassis::motion
