/**
 * @file    Master.cpp
 * @author  syhanjin
 * @date    2026-11-17
 */

#include "Master.hpp"

namespace chassis::controller
{
Master::Master(motion::IChassisMotion& motion, loc::IChassisLoc& loc, const Config& cfg) :
    IChassisController(motion, loc), lock_(osMutexNew(nullptr)), limit_(cfg.limit),
    tracking_threshold_(cfg.tracking_threshold),
    auto_nearest_yaw_target_(cfg.auto_nearest_yaw_target),
    posture_trajectory_{ .pd    = { PD(cfg.posture_error_pd_cfg.vx),
                                    PD(cfg.posture_error_pd_cfg.vy),
                                    PD(cfg.posture_error_pd_cfg.wz) },
                         .curve = { velocity_profile::SCurveProfile(cfg.limit.x, 0, 0, 0, 0),
                                    velocity_profile::SCurveProfile(cfg.limit.y, 0, 0, 0, 0),
                                    velocity_profile::SCurveProfile(cfg.limit.yaw, 0, 0, 0, 0) }

    }
{
}
bool Master::setTargetPostureInWorld(const Posture&           absolute_target,
                                     const TrajectoryLinkMode link_mode,
                                     const TrajectoryLimit&   limit)
{
    osMutexAcquire(lock_, osWaitForever);

    const auto [limit_x, limit_y, limit_yaw] = limit;

    constexpr auto clamp_vel_acc = [&](float& v, float& a, const AxisLimit& l)
    {
        float amin = -l.max_acc, amax = l.max_acc;
        if (v < -l.max_spd)
        {
            v    = -l.max_spd;
            amin = 0;
        }
        else if (v > l.max_spd)
        {
            v    = l.max_spd;
            amax = 0;
        }
        a = std::clamp(a, amin, amax);
    };

    Velocity v{};
    Posture  p{};
    float    ax = 0, ay = 0, ayaw = 0;

    // 如果选择衔接当前状态 或 之前不是位置控制（没有曲线可以衔接）
    if (link_mode == TrajectoryLinkMode::CurrentState || ctrl_mode_ != CtrlMode::Posture)
    {
        // 衔接之前的状态

        // copy 当前位置和速度
        p = postureInWorld();
        v = velocityInWorld();
    }
    else
    {
        // 否则衔接之前的曲线，用于可能的多段路径规划
        p.x   = posture_trajectory_.curve.x.CalcX(posture_trajectory_.now);
        p.y   = posture_trajectory_.curve.y.CalcX(posture_trajectory_.now);
        p.yaw = posture_trajectory_.curve.yaw.CalcX(posture_trajectory_.now);

        v.vx = posture_trajectory_.curve.x.CalcV(posture_trajectory_.now);
        v.vy = posture_trajectory_.curve.y.CalcV(posture_trajectory_.now);
        v.wz = posture_trajectory_.curve.yaw.CalcV(posture_trajectory_.now);

        ax   = posture_trajectory_.curve.x.CalcA(posture_trajectory_.now);
        ay   = posture_trajectory_.curve.y.CalcA(posture_trajectory_.now);
        ayaw = posture_trajectory_.curve.yaw.CalcA(posture_trajectory_.now);
    }
    auto [x, y, yaw]  = p;
    auto [vx, vy, wz] = v;

    // 初始化 S 型曲线
    // 此处需要保证不超过限制，避免产生规划失败的问题
    clamp_vel_acc(vx, ax, limit_x);
    clamp_vel_acc(vy, ay, limit_y);
    clamp_vel_acc(wz, ayaw, limit_yaw);

    Posture target = absolute_target;
    if (auto_nearest_yaw_target_)
        target.yaw = nearestEquivalentYaw(yaw, absolute_target.yaw);

    const velocity_profile::SCurveProfile //
            curve_x(limit_x, x, vx, ax, target.x),
            curve_y(limit_y, y, vy, ay, target.y), curve_yaw(limit_yaw, yaw, wz, ayaw, target.yaw);

    if (!curve_x.success() || !curve_y.success() || !curve_yaw.success())
    {
        last_scurve_failure_       = {};
        last_scurve_failure_.valid = true;
        if (!curve_x.success())
        {
            last_scurve_failure_.axis_mask |= 1U << 0U;
            last_scurve_failure_.axis[0] = curve_x.failureInfo();
        }
        if (!curve_y.success())
        {
            last_scurve_failure_.axis_mask |= 1U << 1U;
            last_scurve_failure_.axis[1] = curve_y.failureInfo();
        }
        if (!curve_yaw.success())
        {
            last_scurve_failure_.axis_mask |= 1U << 2U;
            last_scurve_failure_.axis[2] = curve_yaw.failureInfo();
        }
        osMutexRelease(lock_);
        return false;
    }

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
bool Master::setTargetPostureInWorld(const Posture& absolute_target)
{
    return setTargetPostureInWorld(absolute_target, defaultTrajectoryLinkMode, limit_);
}
bool Master::setTargetPostureInWorld(const Posture&           absolute_target,
                                     const TrajectoryLinkMode link_mode)
{
    return setTargetPostureInWorld(absolute_target, link_mode, limit_);
}
bool Master::setTargetPostureInWorld(const Posture& absolute_target, const TrajectoryLimit& limit)
{
    return setTargetPostureInWorld(absolute_target, defaultTrajectoryLinkMode, limit);
}
bool Master::setTargetPostureInBody(const Posture&           relative_target,
                                    const TrajectoryLinkMode link_mode,
                                    const TrajectoryLimit&   limit)
{
    osMutexAcquire(lock_, osWaitForever);
    const auto absolute_target = this->loc().BodyPosture2WorldPosture(relative_target);
    osMutexRelease(lock_);

    return setTargetPostureInWorld(absolute_target, link_mode, limit);
}
bool Master::setTargetPostureInBody(const Posture& relative_target)
{
    return setTargetPostureInBody(relative_target, defaultTrajectoryLinkMode, limit_);
}
bool Master::setTargetPostureInBody(const Posture&           relative_target,
                                    const TrajectoryLinkMode link_mode)
{
    return setTargetPostureInBody(relative_target, link_mode, limit_);
}
bool Master::setTargetPostureInBody(const Posture& relative_target, const TrajectoryLimit& limit)
{
    return setTargetPostureInBody(relative_target, defaultTrajectoryLinkMode, limit);
}
bool Master::setTargetPostureRelativeTo(const Posture&           base_in_world,
                                        const Posture&           relative_target,
                                        const TrajectoryLinkMode link_mode,
                                        const TrajectoryLimit&   limit)
{
    // 这里的“相对”不是相对于当前自己，而是相对于调用者给出的某个世界系基准点。
    // 适合把某个动作锚定在固定起点上，例如“以上台阶起始点为基准执行动作”。
    osMutexAcquire(lock_, osWaitForever);
    const auto absolute_target = loc::IChassisLoc::RelativePosture2WorldPosture(base_in_world,
                                                                                relative_target);
    osMutexRelease(lock_);

    return setTargetPostureInWorld(absolute_target, link_mode, limit);
}
bool Master::setTargetPostureRelativeTo(const Posture& base_in_world,
                                        const Posture& relative_target)
{
    return setTargetPostureRelativeTo(base_in_world,
                                      relative_target,
                                      defaultTrajectoryLinkMode,
                                      limit_);
}
bool Master::setTargetPostureRelativeTo(const Posture&           base_in_world,
                                        const Posture&           relative_target,
                                        const TrajectoryLinkMode link_mode)
{
    return setTargetPostureRelativeTo(base_in_world, relative_target, link_mode, limit_);
}
bool Master::setTargetPostureRelativeTo(const Posture&         base_in_world,
                                        const Posture&         relative_target,
                                        const TrajectoryLimit& limit)
{
    return setTargetPostureRelativeTo(base_in_world,
                                      relative_target,
                                      defaultTrajectoryLinkMode,
                                      limit);
}
bool Master::isTrajectoryFinished() const
{
    return ctrl_mode_ == CtrlMode::Posture &&
           posture_trajectory_.now >= posture_trajectory_.total_time &&
           isTrajectoryTrackingWithinThreshold();
}
bool Master::isTrajectoryTrackingWithinThreshold() const
{
    if (ctrl_mode_ != CtrlMode::Posture)
        return true;

    const auto [x, y, yaw] = postureInWorld();
    const Posture target{
        .x   = posture_trajectory_.curve.x.CalcX(posture_trajectory_.now),
        .y   = posture_trajectory_.curve.y.CalcX(posture_trajectory_.now),
        .yaw = posture_trajectory_.curve.yaw.CalcX(posture_trajectory_.now),
    };

    return std::fabs(x - target.x) <= tracking_threshold_.x &&
           std::fabs(y - target.y) <= tracking_threshold_.y &&
           std::fabs(Posture::yawError(yaw, target.yaw)) <= tracking_threshold_.yaw;
}
void Master::setVelocityInWorld(const Velocity& world_velocity, const bool target_in_world)
{
    osMutexAcquire(lock_, osWaitForever);
    const auto [vx, vy, wz] = loc_->WorldVelocity2BodyVelocity(world_velocity);

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
void Master::setVelocityInBody(const Velocity& body_velocity, const bool target_in_world)
{
    osMutexAcquire(lock_, osWaitForever);
    const auto [vx, vy, wz] = loc_->BodyVelocity2WorldVelocity(body_velocity);

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
void Master::stop()
{
    osMutexAcquire(lock_, osWaitForever);
    const uint32_t saved = isr_lock();

    // stop 并不是简单清零速度，而是把当前位置记为新的锁定参考点。
    ctrl_mode_ = CtrlMode::Stopped;

    posture_trajectory_.p_ref_curr_ = postureInWorld();
    posture_trajectory_.v_ref_curr_ = { 0, 0, 0 };

    isr_unlock(saved);
    osMutexRelease(lock_);
}
void Master::hold()
{
    setVelocityInBody(Velocity::zero(), false);
}
void Master::profileUpdate(const float dt)
{
    if (!enabled() || ctrl_mode_ != CtrlMode::Posture)
        return;

    // 推进曲线
    const float now = this->posture_trajectory_.now + dt;

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
void Master::errorUpdate()
{
    if (!this->enabled() || !(ctrl_mode_ == CtrlMode::Posture || ctrl_mode_ == CtrlMode::Stopped))
        return;

    const auto [x, y, yaw] = postureInWorld();

    // 使用 pd 控制器跟随当前目标
    posture_trajectory_.pd.vx.calc(posture_trajectory_.p_ref_curr_.x, x);
    posture_trajectory_.pd.vy.calc(posture_trajectory_.p_ref_curr_.y, y);
    posture_trajectory_.pd.wz.calc(posture_trajectory_.p_ref_curr_.yaw, yaw);
    apply_position_velocity();
}
void Master::controllerUpdate()
{
    if (!enabled())
        return;

    // 只有纯速度模式需要这里持续下发；位姿模式在 profile/error update 中已完成下发。
    if (ctrl_mode_ == CtrlMode::Velocity)
        update_velocity_control();
}
float Master::nearestEquivalentYaw(const float reference_yaw, const float target_yaw)
{
    const float turns = (reference_yaw - target_yaw) / 360.0f;
    return target_yaw + std::round(turns) * 360.0f;
}
void Master::update_velocity_control()
{
    if (velocity_ref_.target_in_world)
    { // 如果基于世界坐标计算速度，则需要转为车身坐标系，并应用到底盘驱动器
        velocity_ref_.in_body = this->loc().WorldVelocity2BodyVelocity(velocity_ref_.in_world);
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
        // 直接应用最近一次保存下来的车体系速度参考。
    }
    applyVelocity(velocity_ref_.in_body);
}
void Master::apply_position_velocity()
{
    // 叠加前馈和 pd 输出，先在世界系下得到期望速度，再统一换算到底盘车体系。
    const Velocity velocity_in_world = {
        posture_trajectory_.v_ref_curr_.vx + posture_trajectory_.pd.vx.getOutput(),
        posture_trajectory_.v_ref_curr_.vy + posture_trajectory_.pd.vy.getOutput(),
        posture_trajectory_.v_ref_curr_.wz + posture_trajectory_.pd.wz.getOutput(),
    };

    // 将世界坐标系速度转换为底盘坐标系速度
    const Velocity body_velocity = loc_->WorldVelocity2BodyVelocity(velocity_in_world);

    // 应用速度
    applyVelocity(body_velocity);
}
} // namespace chassis::controller