/**
 * @file    Master.hpp
 * @author  syhanjin
 * @date    2026-02-24
 * @brief   Brief description of the file
 */
#pragma once
#include "IChassis.hpp"
#include "isr_lock.h"

#include <algorithm>
#include <cmath>
#include <type_traits>

namespace chassis::controller
{

template <typename Chassis> class Master : public Chassis
{
    static_assert(std::is_base_of_v<IChassis, Chassis>);

public:
    using AxisLimit = velocity_profile::SCurveProfile::Config;
    using Velocity  = IChassis::Velocity;
    using Posture   = IChassis::Posture;

    struct Config
    {
        struct
        {
            MITPD::Config vx; ///< x 速度 PD 控制器
            MITPD::Config vy; ///< y 速度 PD 控制器
            MITPD::Config wz; ///< 角速度 PD 控制器
        } posture_error_pd_cfg;

        struct
        {
            AxisLimit x, y, yaw;
        } limit{};
    };

    enum class CtrlMode
    {
        Stopped,
        Velocity,
        Posture,
    };

    Master(Chassis&& chassis, const Config& cfg) :
        Chassis(std::move(chassis)), lock_(osMutexNew(nullptr)),                       //
        limit_x_{ cfg.limit.x }, limit_y_{ cfg.limit.y }, limit_yaw_{ cfg.limit.yaw }, //
        posture_trajectory_{ .pd    = { MITPD(cfg.posture_error_pd_cfg.vx),
                                        MITPD(cfg.posture_error_pd_cfg.vy),
                                        MITPD(cfg.posture_error_pd_cfg.wz) },
                             .curve = { velocity_profile::SCurveProfile(cfg.limit.x, 0, 0, 0, 0),
                                        velocity_profile::SCurveProfile(cfg.limit.y, 0, 0, 0, 0),
                                        velocity_profile::SCurveProfile(cfg.limit.yaw, 0, 0, 0, 0) }

        }
    {
    }

    bool setTargetPostureInWorld(const Posture& absolute_target);
    bool setTargetPostureInBody(const Posture& relative_target);

    [[nodiscard]] bool isTrajectoryFinished() const;
    void               waitTrajectoryFinish() const;

    void setVelocityInWorld(const Velocity& world_velocity, bool target_in_world);
    void setVelocityInBody(const Velocity& body_velocity, bool target_in_world);

    void stop();

    void profileUpdate(float dt);
    void errorUpdate();
    void controllerUpdate();

    bool enable()
    {
        stop();
        return Chassis::enable();
    }

    void setWorldFromCurrent()
    {
        if (this->isOpsEnabled())
            return;
        osMutexAcquire(lock_, osWaitForever);
        const auto saved = isr_lock();
        this->applySetWorldFromCurrent();
        isr_unlock(saved);
        osMutexRelease(lock_);
    }

private:
    osMutexId_t lock_;

    CtrlMode ctrl_mode_{ CtrlMode::Stopped }; ///< 当前控制模式

    AxisLimit limit_x_;
    AxisLimit limit_y_;
    AxisLimit limit_yaw_;

    struct
    {
        volatile bool target_in_world; ///< 速度是否相对于世界坐标系不变
        Velocity      in_world;        ///< 世界坐标系下速度
        Velocity      in_body;         ///< 车体坐标系下速度
    } velocity_ref_;

    struct
    {
        float now{};        ///< 当前执行时间
        float total_time{}; ///< 总执行时间

        struct
        {
            MITPD vx; ///< x 速度 PD 控制器
            MITPD vy; ///< y 速度 PD 控制器
            MITPD wz; ///< 角速度 PD 控制器
        } pd;

        struct
        {
            velocity_profile::SCurveProfile x;
            velocity_profile::SCurveProfile y;
            velocity_profile::SCurveProfile yaw;
        } curve;

        Posture  p_ref_curr_{};
        Velocity v_ref_curr_{};
    } posture_trajectory_;

private:
    void update_velocity_control();
    void apply_position_velocity();
};
template <typename Chassis>
bool Master<Chassis>::setTargetPostureInWorld(const Posture& absolute_target)
{
    osMutexAcquire(lock_, osWaitForever);

    // copy 当前位置和速度
    const auto [x, y, yaw]  = this->posture().in_world;
    const auto [vx, vy, wz] = this->velocity().in_world;

    float ax = 0, ay = 0, ayaw = 0;
    if (ctrl_mode_ == CtrlMode::Posture)
    {
        ax   = posture_trajectory_.curve.x.CalcA(posture_trajectory_.now);
        ay   = posture_trajectory_.curve.y.CalcA(posture_trajectory_.now);
        ayaw = posture_trajectory_.curve.yaw.CalcA(posture_trajectory_.now);
    }
    // 初始化 S 型曲线
    // 衔接当前位置，速度，如果之前是位置控制还会衔接加速度
    const velocity_profile::SCurveProfile //
            curve_x(limit_x_, x, vx, ax, absolute_target.x),
            curve_y(limit_y_, y, vy, ay, absolute_target.y),
            curve_yaw(limit_yaw_, yaw, wz, ayaw, absolute_target.yaw);

    if (!curve_x.success() || !curve_y.success() || !curve_yaw.success())
        return false;

    float total_time = std::fmaxf(curve_x.getTotalTime(),
                                  std::fmaxf(curve_y.getTotalTime(), curve_yaw.getTotalTime()));

    const uint32_t saved = isr_lock(); // 写入过程加中断锁

    posture_trajectory_.now        = 0;
    posture_trajectory_.total_time = total_time;

    posture_trajectory_.curve.x   = curve_x;
    posture_trajectory_.curve.y   = curve_y;
    posture_trajectory_.curve.yaw = curve_yaw;

    ctrl_mode_ = CtrlMode::Posture;

    isr_unlock(saved);

    osMutexRelease(lock_);
    return true;
}
template <typename Chassis>
bool Master<Chassis>::setTargetPostureInBody(const Posture& relative_target)
{
    osMutexAcquire(lock_, osWaitForever);
    const auto absolute_target = this->BodyPosture2WorldPosture(relative_target);
    osMutexRelease(lock_);

    return setTargetPostureInWorld(absolute_target);
}
template <typename Chassis> bool Master<Chassis>::isTrajectoryFinished() const
{
    return posture_trajectory_.now >= posture_trajectory_.total_time;
}

template <typename Chassis> void Master<Chassis>::waitTrajectoryFinish() const
{
    while (!isTrajectoryFinished())
        osDelay(1);
}

template <typename Chassis>
void Master<Chassis>::setVelocityInWorld(const Velocity& world_velocity, bool target_in_world)
{
    osMutexAcquire(lock_, osWaitForever);
    const auto [vx, vy, wz] = this->WorldVelocity2BodyVelocity(world_velocity);

    const uint32_t saved = isr_lock(); // 写入过程加中断锁

    velocity_ref_.target_in_world = target_in_world;
    velocity_ref_.in_world.vx     = world_velocity.vx;
    velocity_ref_.in_world.vy     = world_velocity.vy;
    velocity_ref_.in_world.wz     = world_velocity.wz;
    velocity_ref_.in_body.vx      = vx;
    velocity_ref_.in_body.vy      = vy;
    velocity_ref_.in_body.wz      = wz;

    ctrl_mode_ = CtrlMode::Velocity;

    isr_unlock(saved);

    osMutexRelease(lock_);
}
template <typename Chassis>
void Master<Chassis>::setVelocityInBody(const Velocity& body_velocity, bool target_in_world)
{
    osMutexAcquire(lock_, osWaitForever);
    const auto [vx, vy, wz] = this->BodyVelocity2WorldVelocity(body_velocity);

    const uint32_t saved = isr_lock(); // 写入过程加中断锁

    velocity_ref_.target_in_world = target_in_world;
    velocity_ref_.in_body.vx      = body_velocity.vx;
    velocity_ref_.in_body.vy      = body_velocity.vy;
    velocity_ref_.in_body.wz      = body_velocity.wz;
    velocity_ref_.in_world.vx     = vx;
    velocity_ref_.in_world.vy     = vy;
    velocity_ref_.in_world.wz     = wz;

    ctrl_mode_ = CtrlMode::Velocity;

    isr_unlock(saved);
    osMutexRelease(lock_);
}
template <typename Chassis> void Master<Chassis>::stop()
{
    osMutexAcquire(lock_, osWaitForever);
    const uint32_t saved = isr_lock();

    ctrl_mode_ = CtrlMode::Stopped;

    posture_trajectory_.p_ref_curr_ = this->posture().in_world;
    posture_trajectory_.v_ref_curr_ = { 0, 0, 0 };

    isr_unlock(saved);
    osMutexRelease(lock_);
}
template <typename Chassis> void Master<Chassis>::profileUpdate(float dt)
{
    if (!this->enabled() || ctrl_mode_ != CtrlMode::Posture)
        return;

    // 推进曲线
    const float now               = this->posture_trajectory_.now + dt;
    this->posture_trajectory_.now = now;

    // 计算前馈速度
    this->posture_trajectory_.v_ref_curr_ = { .vx = posture_trajectory_.curve.x.CalcV(now),
                                              .vy = posture_trajectory_.curve.y.CalcV(now),
                                              .wz = posture_trajectory_.curve.yaw.CalcV(now) };

    // 计算当前目标
    this->posture_trajectory_.p_ref_curr_ = { .x   = posture_trajectory_.curve.x.CalcX(now),
                                              .y   = posture_trajectory_.curve.y.CalcX(now),
                                              .yaw = posture_trajectory_.curve.yaw.CalcX(now) };

    apply_position_velocity();
}
template <typename Chassis> void Master<Chassis>::errorUpdate()
{
    if (!this->enabled() || !(ctrl_mode_ == CtrlMode::Posture || ctrl_mode_ == CtrlMode::Stopped))
        return;

    // 使用 pd 控制器跟随当前目标
    posture_trajectory_.pd.vx.calc(posture_trajectory_.p_ref_curr_.x,
                                   this->posture().in_world.x,
                                   posture_trajectory_.v_ref_curr_.vx,
                                   this->velocity().in_world.vx);
    posture_trajectory_.pd.vy.calc(posture_trajectory_.p_ref_curr_.y,
                                   this->posture().in_world.y,
                                   posture_trajectory_.v_ref_curr_.vy,
                                   this->velocity().in_world.vy);
    posture_trajectory_.pd.wz.calc(posture_trajectory_.p_ref_curr_.yaw,
                                   this->posture().in_world.yaw,
                                   posture_trajectory_.v_ref_curr_.wz,
                                   this->velocity().in_world.wz);
    apply_position_velocity();
}
template <typename Chassis> void Master<Chassis>::controllerUpdate()
{
    if (!this->enabled())
        return;

    if (ctrl_mode_ == CtrlMode::Velocity)
        update_velocity_control();
    this->velocityControllerUpdate();
}
template <typename Chassis> void Master<Chassis>::update_velocity_control()
{
    if (velocity_ref_.target_in_world)
    { // 如果基于世界坐标计算速度，则需要转为车身坐标系，并应用到底盘驱动器
        velocity_ref_.in_body = this->WorldVelocity2BodyVelocity(velocity_ref_.in_world);
        // 进行修正 1e-3f 为更新间隔，此处发现前馈并没有什么精度优化，暂时不做前馈
        // const float              beta     = DEG2RAD(0.5f * velocity_.in_body.wz * 1e-3f);
        // const float              cot_beta = 1.0f / tanf(beta);
        // const Chassis_Velocity_t temp_velocity = {
        //     .vx = beta * (velocity_.in_body.vx * cot_beta +
        //     velocity_.in_body.vy), .vy = beta * (velocity_.in_body.vy * cot_beta
        //     - velocity_.in_body.vx), .wz = velocity_.in_body.wz
        // };
        // ChassisDriver_ApplyVelocity(&driver_,
        //                             temp_velocity.vx,
        //                             temp_velocity.vy,
        //                             temp_velocity.wz);
    }
    else
    {
        // 直接应用速度
    }
    this->applyVelocity(velocity_ref_.in_body);
}
template <typename Chassis> void Master<Chassis>::apply_position_velocity()
{
    // 叠加前馈和 pd 输出
    const Velocity velocity_in_world = {
        posture_trajectory_.v_ref_curr_.vx + posture_trajectory_.pd.vx.getOutput(),
        posture_trajectory_.v_ref_curr_.vy + posture_trajectory_.pd.vy.getOutput(),
        posture_trajectory_.v_ref_curr_.wz + posture_trajectory_.pd.wz.getOutput(),
    };

    // 将世界坐标系速度转换为底盘坐标系速度
    const Velocity body_velocity = this->WorldVelocity2BodyVelocity(velocity_in_world);

    // 应用速度
    this->applyVelocity(body_velocity);
}

} // namespace chassis::controller
