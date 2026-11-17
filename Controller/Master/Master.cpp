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
    auto_nearest_yaw_target_(cfg.auto_nearest_yaw_target), accel_Tff(cfg.accel_Tff),
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

    Velocity     v{};
    Posture      p{};
    Acceleration a{};
    float        yaw{}; // S 曲线内部可使用连续角，不写入 Posture。

    // 如果选择衔接当前状态 或 之前不是位置控制（没有曲线可以衔接）
    if (link_mode == TrajectoryLinkMode::CurrentState || ctrl_mode_ != CtrlMode::Posture)
    {
        // 衔接之前的状态

        // copy 当前位置和速度
        p = postureInWorld();
        yaw = p.yaw;
        v = velocityInWorld();
    }
    else
    {
        // 否则衔接之前的曲线，用于可能的多段路径规划
        p.x   = posture_trajectory_.curve.x.CalcX(posture_trajectory_.now);
        p.y   = posture_trajectory_.curve.y.CalcX(posture_trajectory_.now);
        yaw   = posture_trajectory_.curve.yaw.CalcX(posture_trajectory_.now);
        p.yaw = Posture::smallestAngle(yaw);

        v.vx = posture_trajectory_.curve.x.CalcV(posture_trajectory_.now);
        v.vy = posture_trajectory_.curve.y.CalcV(posture_trajectory_.now);
        v.wz = posture_trajectory_.curve.yaw.CalcV(posture_trajectory_.now);

        a.ax    = posture_trajectory_.curve.x.CalcA(posture_trajectory_.now);
        a.ay    = posture_trajectory_.curve.y.CalcA(posture_trajectory_.now);
        a.alpha = posture_trajectory_.curve.yaw.CalcA(posture_trajectory_.now);
    }
    auto [vx, vy, wz]    = v;
    auto [ax, ay, alpha] = a;

    // 初始化 S 型曲线
    // 此处需要保证不超过限制，避免产生规划失败的问题
    clamp_vel_acc(vx, ax, limit_x);
    clamp_vel_acc(vy, ay, limit_y);
    clamp_vel_acc(wz, alpha, limit_yaw);

    const float target_yaw = auto_nearest_yaw_target_
                                     ? yaw + Posture::yawError(absolute_target.yaw, yaw)
                                     : absolute_target.yaw;

    const velocity_profile::SCurveProfile //
            curve_x(limit_x, p.x, vx, ax, absolute_target.x),
            curve_y(limit_y, p.y, vy, ay, absolute_target.y),
            curve_yaw(limit_yaw, yaw, wz, alpha, target_yaw);

    // 错误日志记录
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

    posture_trajectory_.p_ref_curr_ = p;
    posture_trajectory_.v_ref_curr_ = v;
    posture_trajectory_.a_ref_curr_ = a;

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
        posture_trajectory_.curve.x.CalcX(posture_trajectory_.now),
        posture_trajectory_.curve.y.CalcX(posture_trajectory_.now),
        posture_trajectory_.curve.yaw.CalcX(posture_trajectory_.now),
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

    stop_locked(postureInWorld());

    isr_unlock(saved);
    osMutexRelease(lock_);
}
void Master::stop_locked(const Posture& current_pose)
{
    // stop 并不是简单清零速度，而是把当前位置记为新的锁定参考点。
    ctrl_mode_                      = CtrlMode::Stopped;
    posture_trajectory_.p_ref_curr_ = current_pose;
    posture_trajectory_.v_ref_curr_ = {};
    posture_trajectory_.a_ref_curr_ = {};
}
void Master::switchLoc(loc::IChassisLoc& new_loc)
{
    osMutexAcquire(lock_, osWaitForever);

    const Posture old_pose = postureInWorld();

    if (ctrl_mode_ != CtrlMode::Stopped)
    {
        ISRGuard guard{};
        stop_locked(old_pose);
    }

    const Posture new_pose = new_loc.postureInWorld();

    const auto p_ref_b =
            loc::IChassisLoc::WorldPosture2RelativePosture(old_pose,
                                                           posture_trajectory_.p_ref_curr_);

    const auto pref = loc::IChassisLoc::RelativePosture2WorldPosture(new_pose, p_ref_b);

    {
        ISRGuard guard{};
        posture_trajectory_.p_ref_curr_ = pref;

        loc_ = &new_loc;
    }

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

    // 加速度与速度使用同一曲线时刻；系数在车体系中应用。
    this->posture_trajectory_.a_ref_curr_ = { .ax    = posture_trajectory_.curve.x.CalcA(now),
                                              .ay    = posture_trajectory_.curve.y.CalcA(now),
                                              .alpha = posture_trajectory_.curve.yaw.CalcA(now) };

    // 计算当前目标
    this->posture_trajectory_.p_ref_curr_ = {
        posture_trajectory_.curve.x.CalcX(now),
        posture_trajectory_.curve.y.CalcX(now),
        posture_trajectory_.curve.yaw.CalcX(now),
    };

    apply_position_velocity();
}
void Master::errorUpdate()
{
    if (!this->enabled() || !(ctrl_mode_ == CtrlMode::Posture || ctrl_mode_ == CtrlMode::Stopped))
        return;

    const auto [x, y, yaw] = loc().WorldPosture2BodyPosture(posture_trajectory_.p_ref_curr_);

    // 车体系相对目标对零反馈闭环；yaw 由坐标变换折返为最短角。
    posture_trajectory_.pd.vx.calc(x, 0.0f);
    posture_trajectory_.pd.vy.calc(y, 0.0f);
    posture_trajectory_.pd.wz.calc(yaw, 0.0f);
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
    const auto& v = posture_trajectory_.v_ref_curr_;
    const auto& a = posture_trajectory_.a_ref_curr_;

    const auto [vb, ab] = loc().VelocityAndAccelerationInBody(v, a);

    // 先把世界系 v/a 旋转到车体系，再分别叠加车体轴 PD 和 T * a_b。
    const Velocity body_velocity = {
        .vx = vb.vx + posture_trajectory_.pd.vx.getOutput() + accel_Tff.x * ab.ax,
        .vy = vb.vy + posture_trajectory_.pd.vy.getOutput() + accel_Tff.y * ab.ay,
        .wz = vb.wz + posture_trajectory_.pd.wz.getOutput() + accel_Tff.yaw * ab.alpha,
    };

    applyVelocity(body_velocity);
}
} // namespace chassis::controller