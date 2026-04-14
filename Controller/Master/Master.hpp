/**
 * @file    Master.hpp
 * @author  syhanjin
 * @date    2026-02-24
 * @brief   下位机主控模式的底盘控制器。
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

/**
 * 下位机主控模式。
 *
 * 这个控制器自己负责：
 * - 速度模式下的参考保持
 * - 位姿模式下的 S 曲线规划
 * - 基于定位反馈的误差闭环
 *
 * 公开接口里有 7 种常用目标设置组合：
 * - 位姿目标 3 种：绝对目标、相对自身目标、相对指定基准点目标
 * - 速度目标 4 种：输入坐标系（世界 / 车身） × 保持不变的参考系（世界 / 车身）
 *
 * 适合“上层给目标，底盘自己走过去”的使用方式。
 */
class Master : public IChassisController
{
public:
    using AxisLimit = velocity_profile::SCurveProfile::Config;

    /// 三个自由度各自的运动学约束。
    struct TrajectoryLimit
    {
        AxisLimit x, y, yaw;
    };

    /// 控制器配置：误差 PD 参数 + 轨迹曲线限制。
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
        Stopped,  ///< 锁定当前位姿
        Velocity, ///< 直接按速度参考控制
        Posture,  ///< 先规划位姿曲线，再跟踪曲线
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

    /// 构造时立即建立三个轴各自的 PD 与 S 曲线对象。
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
     * 在世界坐标系中设置绝对目标位姿。
     *
     * 这是最直接的“去某个绝对点”接口，适合地图中已经有明确目标位姿的场景。
     *
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

    // 下面这组同名函数是 C++ 的“函数重载”。
    // 对不熟悉 C++ 的同学来说，可以简单理解成：函数名相同，但参数个数或类型不同，
    // 编译器会根据你传入的实参，自动挑选最合适的那个版本。
    // 这里这样做的目的，是让调用者在常见场景下少写一些样板参数；真正的规划逻辑
    // 仍然集中在上面这个“参数最全”的实现里，避免复制多份实现后改漏。
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
     * 以“当前车体位姿”为基准，设置相对目标位姿。
     *
     * 适合简单动作命令，例如“向前一段距离”或“相对当前朝向转一个角度”。
     *
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

    // 这组重载的作用和上面一样：让“相对自身”的常见调用可以只关心核心目标值，
    // 而把 link_mode / limit 交给默认配置处理；如果确实需要细调，再调用参数更全的版本。
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
        // 这里的“相对”不是相对于当前自己，而是相对于调用者给出的某个世界系基准点。
        // 适合把某个动作锚定在固定起点上，例如“以上台阶起始点为基准执行动作”。
        osMutexAcquire(lock_, osWaitForever);
        const auto absolute_target =
                loc::IChassisLoc::RelativePosture2WorldPosture(base_in_world, relative_target);
        osMutexRelease(lock_);

        return setTargetPostureInWorld(absolute_target, link_mode, limit);
    }

    // 这一组同样是重载，只是这里多了一个 base_in_world 基准点参数。
    // 保留同名接口的好处是：上层一看到函数名，就知道它们都属于“设置位姿目标”这类操作；
    // 真正的区别只在“目标相对于谁”以及“是否显式传入 link_mode / limit”。
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

    /// 阻塞等待当前位姿轨迹执行完成。常用于流程式脚本控制。
    void waitTrajectoryFinish() const
    {
        while (!isTrajectoryFinished())
            osDelay(1);
    }

    /**
     * 设置世界坐标系速度参考。
     *
     * 这个接口只回答“输入速度是按世界系表达的”。
     * 至于后续控制中究竟保持世界系速度不变，还是只在设置时换算一次后改成车体系保持，
     * 则由 target_in_world 决定。
     *
     * @param world_velocity 目标世界系速度
     * @param target_in_world true 表示后续控制周期中都保持“世界系速度不变”，并在每次执行时
     *                        重新换算到车体系；平移和旋转并存时通常表现为边平移边自旋
     *                        false 表示只在设置时做一次换算，后续按车体系速度保持；此时常见表现是圆弧轨迹
     */
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

    /**
     * 设置车体坐标系速度参考。
     *
     * 这个接口只回答“输入速度是按车体系表达的”。
     * 至于后续控制中究竟保持车体系速度不变，还是先把当前车体系速度锁存成世界系速度，
     * 则由 target_in_world 决定。
     *
     * @param body_velocity 目标车体系速度
     * @param target_in_world true 表示先换算并保存对应的世界系速度，之后按“世界系速度保持不变”
     *                        的方式运行；平移和旋转并存时通常表现为边平移边自旋
     *                        false 表示保持普通车体系速度控制；此时常见表现是圆弧轨迹
     */
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

        // stop 并不是简单清零速度，而是把当前位置记为新的锁定参考点。
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

        // 只有纯速度模式需要这里持续下发；位姿模式在 profile/error update 中已完成下发。
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
    osMutexId_t lock_; ///< 保护配置切换和目标写入过程

    CtrlMode ctrl_mode_{ CtrlMode::Stopped }; ///< 当前控制模式

    TrajectoryLimit limit_; ///< 默认轨迹约束

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

        Posture  p_ref_curr_{}; ///< 当前时刻的位姿参考点
        Velocity v_ref_curr_{}; ///< 当前时刻的前馈速度参考
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
            // 直接应用最近一次保存下来的车体系速度参考。
        }
        applyVelocity(velocity_ref_.in_body);
    }

    void apply_position_velocity()
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
};

} // namespace chassis::controller
