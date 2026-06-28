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
 * 再在世界坐标系下做位置误差闭环（与 Master 的 errorUpdate 一致）。
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
template <size_t BufferCapacity = 0, bool Offline = false>
class Slave : public IChassisController
{
    static_assert(Offline || BufferCapacity >= 2,
                  "BufferCapacity must be >= 2 when Offline is false");

public:
    /// 三个自由度各自的位置误差 PD 参数（世界坐标系下）。
    struct PDConfig
    {
        PD::Config vx; ///< x 位置误差 PD
        PD::Config vy; ///< y 位置误差 PD
        PD::Config wz; ///< yaw 位置误差 PD
    };

    /// 上位机发来的单个轨迹点：位姿参考 + 速度前馈。
    struct TrajectoryPoint
    {
        Posture  p_ref;
        Velocity v_ref;
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
     * 该接口只负责从缓冲区取点，不做误差闭环；真正的跟踪在 errorUpdate() 中完成。
     */
    template <typename = std::enable_if_t<!Offline>>
    void trajectoryUpdate()
    {
        if (!enabled())
            return;

        TrajectoryPoint point;
        if (!cmd_buffer_.pop(point))
            return;

        p_ref_ = point.p_ref;
        v_ref_ = point.v_ref;
    }

    /**
     * 离线模式下注入一个轨迹点作为当前参考。
     *
     * 该接口直接设置参考位姿和速度，不做缓冲区操作。
     *
     * @param point 外部生成的轨迹点
     */
    template <typename = std::enable_if_t<Offline>>
    void profileUpdate(const TrajectoryPoint& point)
    {
        p_ref_ = point.p_ref;
        v_ref_ = point.v_ref;
    }

    /**
     * 误差跟踪，在世界坐标系下做位置误差闭环。
     *
     * 与 Master 的 errorUpdate 一致：世界系下用 PD 计算位置误差，
     * 叠加前馈速度后统一转换为车体系速度下发。
     */
    void errorUpdate()
    {
        if (!enabled())
            return;

        const auto [x, y, yaw] = postureInWorld();

        pd_vx_.calc(p_ref_.x, x);
        pd_vy_.calc(p_ref_.y, y);
        pd_wz_.calc(p_ref_.yaw, yaw);

        apply_position_velocity();
    }

    void stop() override
    {
        osMutexAcquire(lock_, osWaitForever);
        const uint32_t saved = isr_lock();

        clearTrajectory();
        p_ref_ = loc_->postureInWorld();
        v_ref_ = { 0, 0, 0 };

        pd_vx_.reset();
        pd_vy_.reset();
        pd_wz_.reset();

        isr_unlock(saved);
        osMutexRelease(lock_);
    }

    /**
     * 轨迹点
     * @param point 轨迹点
     * @return true 表示成功入队；false 表示缓冲区已满，轨迹点未写入
     */
    template <typename = std::enable_if_t<!Offline>>
    bool pushTrajectoryPoint(const TrajectoryPoint& point)
    {
        const uint32_t saved = isr_lock();
        const bool     result = cmd_buffer_.push(point);
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
    osMutexId_t lock_; ///< 保护 stop 等状态切换

    PD pd_vx_; ///< x 位置误差 PD 控制器
    PD pd_vy_; ///< y 位置误差 PD 控制器
    PD pd_wz_; ///< yaw 位置误差 PD 控制器

    Posture  p_ref_{}; ///< 当前正在跟踪的参考位姿（世界系）
    Velocity v_ref_{}; ///< 当前正在跟踪的参考速度（世界系）

    /// 当 Offline = true 时替代 RingBuffer 的空占位类型。
    struct NoBuffer
    {
    };

    [[no_unique_address]] std::conditional_t<Offline, NoBuffer,
                                             libs::RingBuffer<TrajectoryPoint, BufferCapacity>>
            cmd_buffer_;

    void apply_position_velocity()
    {
        // 叠加世界系前馈和误差闭环输出后，再统一转换为车体系速度命令。
        const Velocity velocity_in_world = {
            v_ref_.vx + pd_vx_.getOutput(),
            v_ref_.vy + pd_vy_.getOutput(),
            v_ref_.wz + pd_wz_.getOutput(),
        };

        // 将世界坐标系速度转换为底盘坐标系速度
        const Velocity body_velocity = loc_->WorldVelocity2BodyVelocity(velocity_in_world);

        // 应用速度
        applyVelocity(body_velocity);
    }
};

} // namespace chassis::controller
