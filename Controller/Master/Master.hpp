/**
 * @file    Master.hpp
 * @author  syhanjin
 * @date    2026-02-24
 * @brief   Brief description of the file
 */
#pragma once
#include "IChassisController.hpp"
#include "isr_lock.h"
#include "s_curve.hpp"
#include "mit_pd.hpp"
#include "cmsis_os2.h"

#include <algorithm>
#include <cmath>

namespace chassis::controller
{

class Master : public IChassisController
{
public:
    using AxisLimit = velocity_profile::SCurveProfile::Config;

    struct TrajectoryLimit
    {
        AxisLimit x, y, yaw;
    };

    struct Config
    {
        struct
        {
            MITPD::Config vx; ///< x 速度 PD 控制器
            MITPD::Config vy; ///< y 速度 PD 控制器
            MITPD::Config wz; ///< 角速度 PD 控制器
        } posture_error_pd_cfg;

        TrajectoryLimit limit{};
    };

    enum class CtrlMode
    {
        Stopped,
        Velocity,
        Posture,
    };

    /**
     * 设置位置目标时曲线的衔接方式
     */
    enum class TrajectoryLinkMode
    {
        CurrentState,  // 使用当前估计的位姿 / 速度，加速度置零
        PreviousCurve, // 使用上一条轨迹在 now 时刻的位置 / 速度 / 加速度
    };

    static constexpr auto defaultTrajectoryLinkMode = TrajectoryLinkMode::CurrentState;

    Master(motion::IChassisMotion& motion, loc::IChassisLoc& loc, const Config& cfg) :
        IChassisController(motion, loc), lock_(osMutexNew(nullptr)), limit_(cfg.limit),
        posture_trajectory_{ .pd    = { MITPD(cfg.posture_error_pd_cfg.vx),
                                        MITPD(cfg.posture_error_pd_cfg.vy),
                                        MITPD(cfg.posture_error_pd_cfg.wz) },
                             .curve = { velocity_profile::SCurveProfile(cfg.limit.x, 0, 0, 0, 0),
                                        velocity_profile::SCurveProfile(cfg.limit.y, 0, 0, 0, 0),
                                        velocity_profile::SCurveProfile(cfg.limit.yaw, 0, 0, 0, 0) }

        }
    {
    }

    /**
     * 在世界中设置绝对目标位置
     * @param absolute_target 绝对目标值
     * @param link_mode 曲线衔接模式，如果上一控制状态不为 Posture，则该项不生效
     * @param limit 执行过程限制
     * @return 是否规划成功
     */
    bool setTargetPostureInWorld(const Posture&           absolute_target,
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

        const velocity_profile::SCurveProfile //
                curve_x(limit_x, x, vx, ax, absolute_target.x),
                curve_y(limit_y, y, vy, ay, absolute_target.y),
                curve_yaw(limit_yaw, yaw, wz, ayaw, absolute_target.yaw);

        if (!curve_x.success() || !curve_y.success() || !curve_yaw.success())
        {
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

    bool setTargetPostureInWorld(const Posture& absolute_target)
    {
        return setTargetPostureInWorld(absolute_target, defaultTrajectoryLinkMode, limit_);
    }
    bool setTargetPostureInWorld(const Posture& absolute_target, const TrajectoryLinkMode link_mode)
    {
        return setTargetPostureInWorld(absolute_target, link_mode, limit_);
    }
    bool setTargetPostureInWorld(const Posture& absolute_target, const TrajectoryLimit& limit)
    {
        return setTargetPostureInWorld(absolute_target, defaultTrajectoryLinkMode, limit);
    }

    /**
     * 在世界中设置相对目标位置
     * @param relative_target 相对目标值
     * @param link_mode 曲线衔接模式，如果上一控制状态不为 Posture，则该项不生效
     * @param limit 执行过程限制
     * @return 是否规划成功
     */
    bool setTargetPostureInBody(const Posture&           relative_target,
                                const TrajectoryLinkMode link_mode,
                                const TrajectoryLimit&   limit)
    {
        osMutexAcquire(lock_, osWaitForever);
        const auto absolute_target = this->loc().BodyPosture2WorldPosture(relative_target);
        osMutexRelease(lock_);

        return setTargetPostureInWorld(absolute_target, link_mode, limit);
    }

    bool setTargetPostureInBody(const Posture& relative_target)
    {
        return setTargetPostureInBody(relative_target, defaultTrajectoryLinkMode, limit_);
    }
    bool setTargetPostureInBody(const Posture& relative_target, const TrajectoryLinkMode link_mode)
    {
        return setTargetPostureInBody(relative_target, link_mode, limit_);
    }
    bool setTargetPostureInBody(const Posture& relative_target, const TrajectoryLimit& limit)
    {
        return setTargetPostureInBody(relative_target, defaultTrajectoryLinkMode, limit);
    }

    bool setTargetPostureRelativeTo(const Posture&           base_in_world,
                                    const Posture&           relative_target,
                                    const TrajectoryLinkMode link_mode,
                                    const TrajectoryLimit&   limit)
    {
        osMutexAcquire(lock_, osWaitForever);
        const auto absolute_target =
                loc::IChassisLoc::RelativePosture2WorldPosture(base_in_world, relative_target);
        osMutexRelease(lock_);

        return setTargetPostureInWorld(absolute_target, link_mode, limit);
    }

    bool setTargetPostureRelativeTo(const Posture& base_in_world, const Posture& relative_target)
    {
        return setTargetPostureRelativeTo(base_in_world,
                                          relative_target,
                                          defaultTrajectoryLinkMode,
                                          limit_);
    }

    bool setTargetPostureRelativeTo(const Posture&           base_in_world,
                                    const Posture&           relative_target,
                                    const TrajectoryLinkMode link_mode)
    {
        return setTargetPostureRelativeTo(base_in_world, relative_target, link_mode, limit_);
    }

    bool setTargetPostureRelativeTo(const Posture&         base_in_world,
                                    const Posture&         relative_target,
                                    const TrajectoryLimit& limit)
    {
        return setTargetPostureRelativeTo(base_in_world,
                                          relative_target,
                                          defaultTrajectoryLinkMode,
                                          limit);
    }

    [[nodiscard]] bool isTrajectoryFinished() const
    {
        return posture_trajectory_.now >= posture_trajectory_.total_time;
    }

    void waitTrajectoryFinish() const
    {
        while (!isTrajectoryFinished())
            osDelay(1);
    }

    void setVelocityInWorld(const Velocity& world_velocity, const bool target_in_world)
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

    void setVelocityInBody(const Velocity& body_velocity, const bool target_in_world)
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

    void stop() override
    {
        osMutexAcquire(lock_, osWaitForever);
        const uint32_t saved = isr_lock();

        ctrl_mode_ = CtrlMode::Stopped;

        posture_trajectory_.p_ref_curr_ = postureInWorld();
        posture_trajectory_.v_ref_curr_ = { 0, 0, 0 };

        isr_unlock(saved);
        osMutexRelease(lock_);
    }

    /**
     * 更新底盘轨迹规划曲线
     *
     * 仅在 CtrlMode::Posture 下有效
     * @param dt 更新间隔
     * @note 推荐 100Hz
     */
    void profileUpdate(const float dt)
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

    /**
     * 更新底盘轨迹 PD 控制器
     * @note 推荐 200 ~ 500Hz
     */
    void errorUpdate()
    {
        if (!this->enabled() ||
            !(ctrl_mode_ == CtrlMode::Posture || ctrl_mode_ == CtrlMode::Stopped))
            return;

        // 使用 pd 控制器跟随当前目标
        posture_trajectory_.pd.vx.calc(posture_trajectory_.p_ref_curr_.x,
                                       postureInWorld().x,
                                       posture_trajectory_.v_ref_curr_.vx,
                                       velocityInWorld().vx);
        posture_trajectory_.pd.vy.calc(posture_trajectory_.p_ref_curr_.y,
                                       postureInWorld().y,
                                       posture_trajectory_.v_ref_curr_.vy,
                                       velocityInWorld().vy);
        posture_trajectory_.pd.wz.calc(posture_trajectory_.p_ref_curr_.yaw,
                                       postureInWorld().yaw,
                                       posture_trajectory_.v_ref_curr_.wz,
                                       velocityInWorld().wz);
        apply_position_velocity();
    }

    void controllerUpdate()
    {
        if (!enabled())
            return;

        if (ctrl_mode_ == CtrlMode::Velocity)
            update_velocity_control();
    }

    // void setWorldFromCurrent()
    // {
    //     if (this->isOpsEnabled())
    //         return;
    //     osMutexAcquire(lock_, osWaitForever);
    //     const auto saved = isr_lock();
    //     this->applySetWorldFromCurrent();
    //     isr_unlock(saved);
    //     osMutexRelease(lock_);
    // }

private:
    osMutexId_t lock_;

    CtrlMode ctrl_mode_{ CtrlMode::Stopped }; ///< 当前控制模式

    TrajectoryLimit limit_;

    struct
    {
        volatile bool target_in_world; ///< 速度是否相对于世界坐标系不变
        Velocity      in_world;        ///< 世界坐标系下速度
        Velocity      in_body;         ///< 车体坐标系下速度
    } velocity_ref_{};

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
    void update_velocity_control()
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
            // 直接应用速度
        }
        applyVelocity(velocity_ref_.in_body);
    }

    void apply_position_velocity()
    {
        // 叠加前馈和 pd 输出
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
};

} // namespace chassis::controller
