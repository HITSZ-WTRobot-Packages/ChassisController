/**
 * @file    IChassis.cpp
 * @author  syhanjin
 * @date    2026-01-31
 */
#include "IChassis.hpp"
#include <cmath>

extern "C"
{
#include "cmsis_compiler.h"
static uint32_t isr_lock()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}
static void isr_unlock(uint32_t primask)
{
    __DSB();
    __ISB();
    __set_PRIMASK(primask);
}
}

namespace chassis
{
IChassis::Velocity IChassis::WorldVelocity2BodyVelocity(const Velocity& velocity_in_world) const
{
    const float _sin_yaw = sinf(DEG2RAD(-posture_.in_world.yaw)),
                _cos_yaw = cosf(DEG2RAD(-posture_.in_world.yaw));

    const Velocity velocity_in_body = {
        .vx = velocity_in_world.vx * _cos_yaw - velocity_in_world.vy * _sin_yaw,
        .vy = velocity_in_world.vx * _sin_yaw + velocity_in_world.vy * _cos_yaw,
        .wz = velocity_in_world.wz
    };

    return velocity_in_body;
}

IChassis::Velocity IChassis::BodyVelocity2WorldVelocity(const Velocity& velocity_in_body) const
{
    const float sin_yaw = sinf(DEG2RAD(posture_.in_world.yaw)),
                cos_yaw = cosf(DEG2RAD(posture_.in_world.yaw));

    const Velocity velocity_in_world = {
        .vx = velocity_in_body.vx * cos_yaw - velocity_in_body.vy * sin_yaw,
        .vy = velocity_in_body.vx * sin_yaw + velocity_in_body.vy * cos_yaw,
        .wz = velocity_in_body.wz,
    };

    return velocity_in_world;
}

IChassis::Posture IChassis::WorldPosture2BodyPosture(const Posture& posture_in_world) const
{
    const float _sin_yaw = sinf(DEG2RAD(-posture_.in_world.yaw)),
                _cos_yaw = cosf(DEG2RAD(-posture_.in_world.yaw));

    const float tx = posture_in_world.x - posture_.in_world.x;
    const float ty = posture_in_world.y - posture_.in_world.y;

    const Posture posture_in_body = {
        .x   = tx * _cos_yaw - ty * _sin_yaw,
        .y   = tx * _sin_yaw + ty * _cos_yaw,
        .yaw = posture_in_world.yaw - posture_.in_world.yaw,
    };

    return posture_in_body;
}

IChassis::Posture IChassis::BodyPosture2WorldPosture(const Posture& posture_in_body) const
{
    const float sin_yaw            = sinf(DEG2RAD(posture_.in_world.yaw)),
                cos_yaw            = cosf(DEG2RAD(posture_.in_world.yaw));
    const Posture posture_in_world = {
        .x   = posture_in_body.x * cos_yaw - posture_in_body.y * sin_yaw + posture_.in_world.x,
        .y   = posture_in_body.x * sin_yaw + posture_in_body.y * cos_yaw + posture_.in_world.y,
        .yaw = posture_in_body.yaw + posture_.in_world.yaw,
    };

    return posture_in_world;
}

void IChassis::feedbackUpdate()
{
    if (!enabled())
        return;
    update_posture();
    update_velocity_feedback();
}

void IChassis::profileUpdate(const float dt)
{
    if (!enabled() || ctrl_mode_ != CtrlMode::Posture)
        return;

    // 推进曲线
    const float now         = posture_.trajectory.now + dt;
    posture_.trajectory.now = now;

    // 计算前馈速度
    posture_.trajectory.v_ref_curr_ = { .vx = posture_.trajectory.curve.x.CalcV(now),
                                        .vy = posture_.trajectory.curve.y.CalcV(now),
                                        .wz = posture_.trajectory.curve.yaw.CalcV(now) };

    // 计算当前目标
    posture_.trajectory.p_ref_curr_ = { .x   = posture_.trajectory.curve.x.CalcX(now),
                                        .y   = posture_.trajectory.curve.y.CalcX(now),
                                        .yaw = posture_.trajectory.curve.yaw.CalcX(now) };

    apply_position_velocity();
}

void IChassis::errorUpdate()
{
    if (!enabled() || ctrl_mode_ != CtrlMode::Posture)
        return;

    // 使用 pd 控制器跟随当前目标
    posture_.trajectory.pd.vx.calc(posture_.trajectory.p_ref_curr_.x,
                                   posture_.in_world.x,
                                   posture_.trajectory.v_ref_curr_.vx,
                                   velocity_.feedback.in_world.vx);
    posture_.trajectory.pd.vy.calc(posture_.trajectory.p_ref_curr_.y,
                                   posture_.in_world.y,
                                   posture_.trajectory.v_ref_curr_.vy,
                                   velocity_.feedback.in_world.vy);
    posture_.trajectory.pd.wz.calc(posture_.trajectory.p_ref_curr_.yaw,
                                   posture_.in_world.yaw,
                                   posture_.trajectory.v_ref_curr_.wz,
                                   velocity_.feedback.in_world.wz);
    apply_position_velocity();
}

void IChassis::controllerUpdate()
{
    if (!enabled())
        return;

    if (ctrl_mode_ == CtrlMode::Velocity)
        update_velocity_control();
    velocityControllerUpdate();
}

bool IChassis::setTargetPostureInWorld(const Posture& absolute_target)
{
    osMutexAcquire(lock_, osWaitForever);

    // copy 当前位置和速度
    const auto [x, y, yaw]  = posture_.in_world;
    const auto [vx, vy, wz] = velocity_.in_world;

    float ax = 0, ay = 0, ayaw = 0;
    if (ctrl_mode_ == CtrlMode::Posture)
    {
        ax   = posture_.trajectory.curve.x.CalcA(posture_.trajectory.now);
        ay   = posture_.trajectory.curve.y.CalcA(posture_.trajectory.now);
        ayaw = posture_.trajectory.curve.yaw.CalcA(posture_.trajectory.now);
    }
    // 初始化 S 型曲线
    // 衔接当前位置，速度，如果之前是位置控制还会衔接加速度
    const velocity_profile::SCurveProfile //
            curve_x(limit_x_, x, absolute_target.x, vx, ax),
            curve_y(limit_y_, y, absolute_target.y, vy, ay),
            curve_yaw(limit_yaw_, yaw, absolute_target.yaw, wz, ayaw);

    if (!curve_x.success() || !curve_y.success() || !curve_yaw.success())
        return false;

    float total_time = std::fmaxf(curve_x.getTotalTime(),
                                  std::fmaxf(curve_y.getTotalTime(), curve_yaw.getTotalTime()));

    const uint32_t saved = isr_lock(); // 写入过程加中断锁

    posture_.trajectory.now        = 0;
    posture_.trajectory.total_time = total_time;

    posture_.trajectory.curve.x   = curve_x;
    posture_.trajectory.curve.y   = curve_y;
    posture_.trajectory.curve.yaw = curve_yaw;

    ctrl_mode_ = CtrlMode::Posture;

    isr_unlock(saved);

    osMutexRelease(lock_);
    return true;
}

bool IChassis::setTargetPostureInBody(const Posture& relative_target)
{
    osMutexAcquire(lock_, osWaitForever);
    const auto absolute_target = BodyPosture2WorldPosture(relative_target);
    osMutexRelease(lock_);

    return setTargetPostureInWorld(absolute_target);
}
bool IChassis::isTrajectoryFinished() const
{
    return posture_.trajectory.now >= posture_.trajectory.total_time;
}
void IChassis::waitTrajectoryFinish() const
{
    while (!isTrajectoryFinished())
        osDelay(1);
}

void IChassis::setVelocityInWorld(const Velocity& world_velocity, const bool target_in_world)
{
    osMutexAcquire(lock_, osWaitForever);
    const auto [vx, vy, wz] = WorldVelocity2BodyVelocity(world_velocity);

    const uint32_t saved = isr_lock(); // 写入过程加中断锁

    velocity_.target_in_world = target_in_world;
    velocity_.in_world.vx     = world_velocity.vx;
    velocity_.in_world.vy     = world_velocity.vy;
    velocity_.in_world.wz     = world_velocity.wz;
    velocity_.in_body.vx      = vx;
    velocity_.in_body.vy      = vy;
    velocity_.in_body.wz      = wz;

    ctrl_mode_ = CtrlMode::Velocity;

    isr_unlock(saved);

    osMutexRelease(lock_);
}

void IChassis::setVelBodyFrame(const Velocity& body_velocity, const bool target_in_world)
{
    osMutexAcquire(lock_, osWaitForever);
    const auto [vx, vy, wz] = BodyVelocity2WorldVelocity(body_velocity);

    const uint32_t saved = isr_lock(); // 写入过程加中断锁

    velocity_.target_in_world = target_in_world;
    velocity_.in_body.vx      = body_velocity.vx;
    velocity_.in_body.vy      = body_velocity.vy;
    velocity_.in_body.wz      = body_velocity.wz;
    velocity_.in_world.vx     = vx;
    velocity_.in_world.vy     = vy;
    velocity_.in_world.wz     = wz;

    ctrl_mode_ = CtrlMode::Velocity;

    isr_unlock(saved);
    osMutexRelease(lock_);
}

void IChassis::setWorldFromCurrent()
{
    // 启用 OPS 的情况下 world 由 OPS 管理
    if (isOpsEnabled())
        return;

    osMutexAcquire(lock_, osWaitForever);

    const uint32_t saved = isr_lock(); // 写入过程加中断锁

    world_.posture.x += posture_.in_world.x;
    world_.posture.y += posture_.in_world.y;
    world_.posture.yaw += posture_.in_world.yaw;
    posture_.in_world.x         = 0.0f;
    posture_.in_world.y         = 0.0f;
    posture_.in_world.yaw       = 0.0f;
    velocity_.in_world          = velocity_.in_body;
    velocity_.feedback.in_world = velocity_.feedback.in_body;

    isr_unlock(saved);

    osMutexRelease(lock_);
}

IChassis::IChassis(const Config& cfg) :
    lock_(osMutexNew(nullptr)),                                                    //
    limit_x_{ cfg.limit.x }, limit_y_{ cfg.limit.y }, limit_yaw_{ cfg.limit.yaw }, //
    posture_{
        .trajectory = { .pd    = { MITPD(cfg.posture_error_pd_cfg.vx),
                                   MITPD(cfg.posture_error_pd_cfg.vy),
                                   MITPD(cfg.posture_error_pd_cfg.wz) },
                        .curve = { velocity_profile::SCurveProfile(cfg.limit.x, 0, 0, 0, 0),
                                   velocity_profile::SCurveProfile(cfg.limit.y, 0, 0, 0, 0),
                                   velocity_profile::SCurveProfile(cfg.limit.yaw, 0, 0, 0, 0) } }
    },
    feedback_(cfg.feedback_source) //
{
}

void IChassis::update_posture()
{
    if (isOpsEnabled())
    {
        // 直接读取 OPS
        posture_.in_world.x   = *feedback_.x;
        posture_.in_world.y   = *feedback_.y;
        posture_.in_world.yaw = *feedback_.yaw;
    }
    else
    {
        // 通过里程计或者运动学解算计算位置
        const float sx  = feedback_.sx != nullptr ? *feedback_.sx : forwardGetX();
        const float sy  = feedback_.sy != nullptr ? *feedback_.sy : forwardGetY();
        const float yaw = feedback_.yaw != nullptr ? *feedback_.yaw : forwardGetYaw();

        const float dx                   = sx - last_feedback_.sx;
        const float dy                   = sy - last_feedback_.sy;
        const float ave_yaw              = (yaw + last_feedback_.yaw) * 0.5f;
        const float ave_yaw_in_world_rad = DEG2RAD(ave_yaw - world_.posture.yaw);

        last_feedback_.sx  = sx;
        last_feedback_.sy  = sy;
        last_feedback_.yaw = yaw;

        posture_.in_world.x += dx * cosf(ave_yaw_in_world_rad) - dy * sinf(ave_yaw_in_world_rad);
        posture_.in_world.y += dx * sinf(ave_yaw_in_world_rad) + dy * cosf(ave_yaw_in_world_rad);
        posture_.in_world.yaw = yaw - world_.posture.yaw;
    }
}
void IChassis::update_velocity_control()
{
    if (velocity_.target_in_world)
    { // 如果基于世界坐标计算速度，则需要转为车身坐标系，并应用到底盘驱动器
        velocity_.in_body = WorldVelocity2BodyVelocity(velocity_.in_world);
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
    applyVelocity(velocity_.in_body);
}
void IChassis::update_velocity_feedback()
{
    velocity_.feedback.in_body.vx = feedback_.vx != nullptr ? *feedback_.vx : forwardGetVx();
    velocity_.feedback.in_body.vy = feedback_.vy != nullptr ? *feedback_.vy : forwardGetVy();
    velocity_.feedback.in_body.wz = feedback_.wz != nullptr ? *feedback_.wz : forwardGetWz();
    velocity_.feedback.in_world   = BodyVelocity2WorldVelocity(velocity_.feedback.in_body);
}
void IChassis::apply_position_velocity()
{
    // 叠加前馈和 pd 输出
    const Velocity velocity_in_world = {
        posture_.trajectory.v_ref_curr_.vx + posture_.trajectory.pd.vx.getOutput(),
        posture_.trajectory.v_ref_curr_.vy + posture_.trajectory.pd.vy.getOutput(),
        posture_.trajectory.v_ref_curr_.wz + posture_.trajectory.pd.wz.getOutput(),
    };

    // 将世界坐标系速度转换为底盘坐标系速度
    const Velocity body_velocity = WorldVelocity2BodyVelocity(velocity_in_world);

    // 应用速度
    applyVelocity(body_velocity);
}
} // namespace chassis