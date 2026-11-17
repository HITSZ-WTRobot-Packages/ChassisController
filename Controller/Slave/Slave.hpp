/**
 * @file    Slave.hpp
 * @author  syhanjin
 * @date    2026-02-24
 * @brief   上位机主控模式的底盘控制器。
 */
#pragma once
#include "IChassisController.hpp"
#include "IChassisDef.hpp"
#include "IChassisLoc.hpp"
#include "isr_lock.h"
#include "RingBuffer.hpp"
#include "pid_pd.hpp"
#include "cmsis_os2.h"

#include <type_traits>

namespace chassis::controller
{

/**
 * 从机控制模式的底盘控制器。
 *
 * 该控制器不自己规划轨迹，而是消费外部送来的离散轨迹点，
 * 再将世界系目标变换到车体系，按车体各轴做位置误差 PD 闭环。
 *
 * 支持两种模式：
 * - 在线模式 (Offline = false)：通过 pushTrajectoryPoint() 写入环形缓冲区，
 *   trajectoryUpdate() 消费。
 * - 离线模式 (Offline = true)：不持有缓冲区，直接通过
 *   profileUpdate(const TrajectoryPoint&) 注入轨迹点。
 *
 * @tparam BufferCapacity 轨迹点缓冲区容量（离线模式下忽略）
 * @tparam Offline 离线模式开关
 * @note BufferCapacity >= 轨迹时长 s * (主机发送频率 Hz - trajectoryUpdate 调用频率 Hz),
 *       buffer 占用空间 BufferCapacity * 24 Byte
 */
template <size_t BufferCapacity = 0, bool Offline = false> class Slave : public IChassisController
{
    static_assert(Offline || BufferCapacity >= 2,
                  "BufferCapacity must be >= 2 when Offline is false");

public:
    /// 三个自由度各自的位置误差 PD 参数（车体坐标系下）。
    struct PDConfig
    {
        PD::Config vx; ///< 车体 x 位置误差 PD
        PD::Config vy; ///< 车体 y 位置误差 PD
        PD::Config wz; ///< yaw 最短角位置误差 PD
    };

    /// 上位机发来的单个轨迹点：位姿参考 + 速度前馈。
    struct TrajectoryPoint
    {
        Posture  p_ref; ///< 世界系位姿；写入控制器时 yaw 归一化到 [-180, 180] deg
        Velocity v_ref; ///< 世界系速度前馈
    };

    /// 构造时绑定 Motion、Loc 以及三个自由度各自的 PD 控制器。
    Slave(motion::IChassisMotion& motion, loc::IChassisLoc& loc, const PDConfig& pd_cfg) :
        IChassisController(motion, loc), lock_(osMutexNew(nullptr)), pd_vx_(pd_cfg.vx),
        pd_vy_(pd_cfg.vy), pd_wz_(pd_cfg.wz)
    {
    }

    /**
     * 消费一个新的轨迹点作为当前参考。
     *
     * 成功取点后进入跟踪；缓冲区为空时保留当前状态，误差闭环由 errorUpdate() 完成。
     */
    template <bool B = !Offline, typename = std::enable_if_t<B>> void trajectoryUpdate()
    {
        if (!enabled())
            return;

        const uint32_t  saved = isr_lock();
        TrajectoryPoint point;
        if (cmd_buffer_.pop(point))
        {
            p_ref_   = point.p_ref;
            v_ref_   = point.v_ref;
            stopped_ = false;
        }
        isr_unlock(saved);
    }

    /**
     * 离线模式下注入一个轨迹点作为当前参考。
     *
     * 该接口直接设置参考位姿和速度并进入跟踪，不做缓冲区操作。
     *
     * @param point 外部生成的轨迹点
     */
    template <bool B = Offline, typename = std::enable_if_t<B>>
    void profileUpdate(const TrajectoryPoint& point)
    {
        const uint32_t saved = isr_lock();
        p_ref_               = point.p_ref;
        v_ref_               = point.v_ref;
        stopped_             = false;
        isr_unlock(saved);
    }

    /**
     * 误差跟踪，在车体坐标系下做位置误差 PD 闭环。
     *
     * 世界系参考位姿转换为车体系相对目标后，以零为反馈计算 PD；
     * yaw 由坐标变换折返为最短角。世界系速度前馈单独转换到车体系，
     * 再叠加车体各轴 PD 输出。
     */
    void errorUpdate()
    {
        if (!enabled())
            return;

        const auto [x, y, yaw] = loc_->WorldPosture2BodyPosture(p_ref_);

        pd_vx_.calc(x, 0.0f);
        pd_vy_.calc(y, 0.0f);
        pd_wz_.calc(yaw, 0.0f);

        apply_position_velocity();
    }

    /// 丢弃待消费轨迹，锁定当前位置并清零前馈和 PD 历史，进入停止状态。
    void stop() override
    {
        osMutexAcquire(lock_, osWaitForever);
        const uint32_t saved = isr_lock();

        clearTrajectory();
        p_ref_   = loc_->postureInWorld();
        v_ref_   = Velocity::zero();
        stopped_ = true;

        pd_vx_.reset();
        pd_vy_.reset();
        pd_wz_.reset();

        isr_unlock(saved);
        osMutexRelease(lock_);
    }

    /**
     * 切换定位源并进入停止状态，丢弃旧定位源世界系下的所有待消费轨迹。
     *
     * 跟踪中先取消跟踪并锁定旧定位源的当前位置；已停止时保留锁定目标的
     * 车体系相对位姿。迁移参考时不重置 PD 历史，避免定位坐标跳变引入误差突变。
     * 该接口与 stop() 通过互斥锁串行化；周期入口仍由调用方按 Core 契约串行化。
     */
    void switchLoc(loc::IChassisLoc& new_loc) override
    {
        osMutexAcquire(lock_, osWaitForever);

        const Posture old_pose   = loc_->postureInWorld();
        const Posture new_pose   = new_loc.postureInWorld();
        const Posture locked_ref = stopped_ ? p_ref_ : old_pose;
        const auto ref_body  = loc::IChassisLoc::WorldPosture2RelativePosture(old_pose, locked_ref);
        const auto ref_world = loc::IChassisLoc::RelativePosture2WorldPosture(new_pose, ref_body);

        const uint32_t saved = isr_lock();
        clearTrajectory();
        p_ref_   = ref_world;
        v_ref_   = Velocity::zero();
        stopped_ = true;
        loc_     = &new_loc;
        isr_unlock(saved);

        osMutexRelease(lock_);
    }

    /**
     * 轨迹点
     * @param point 轨迹点
     * @return true 表示成功入队；false 表示缓冲区已满，轨迹点未写入
     */
    template <bool B = !Offline, typename = std::enable_if_t<B>>
    bool pushTrajectoryPoint(const TrajectoryPoint& point)
    {
        const uint32_t saved  = isr_lock();
        const bool     result = cmd_buffer_.push(
                [&point](TrajectoryPoint& buffered)
                {
                    buffered.p_ref = { point.p_ref.x, point.p_ref.y, point.p_ref.yaw };
                    buffered.v_ref = point.v_ref;
                });
        isr_unlock(saved);
        return result;
    }

    /// 清空所有尚未消费的轨迹点，常用于控制权交接前丢弃旧命令。
    void clearTrajectory()
    {
        if constexpr (!Offline)
            cmd_buffer_.clear();
    }

private:
    osMutexId_t lock_; ///< 串行化 stop 与 switchLoc

    PD pd_vx_; ///< 车体 x 位置误差 PD 控制器
    PD pd_vy_; ///< 车体 y 位置误差 PD 控制器
    PD pd_wz_; ///< yaw 最短角位置误差 PD 控制器

    Posture  p_ref_{};         ///< 当前跟踪或停止锁定的目标位姿（世界系）
    Velocity v_ref_{};         ///< 当前速度前馈（世界系，停止时清零）
    bool     stopped_{ true }; ///< 停止时跨定位源切换保留当前锁定目标的车体系相对位姿

    /// 当 Offline = true 时替代 RingBuffer 的空占位类型。
    struct NoBuffer
    {
    };

    [[no_unique_address]] std::
            conditional_t<Offline, NoBuffer, libs::RingBuffer<TrajectoryPoint, BufferCapacity>>
                    cmd_buffer_;

    void apply_position_velocity()
    {
        // 仅将世界系前馈旋转到车体系，车体轴 PD 输出直接叠加。
        const Velocity feedforward_body = loc_->WorldVelocity2BodyVelocity(v_ref_);
        const Velocity body_velocity    = {
            feedforward_body.vx + pd_vx_.getOutput(),
            feedforward_body.vy + pd_vy_.getOutput(),
            feedforward_body.wz + pd_wz_.getOutput(),
        };

        // 应用速度
        applyVelocity(body_velocity);
    }
};

} // namespace chassis::controller
