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
#include "mit_pd.hpp"
#include "cmsis_os2.h"

namespace chassis::controller
{

/**
 * 从机控制模式的底盘
 *
 * 该控制器不自己规划轨迹，而是消费外部送来的离散轨迹点，
 * 再在本地根据定位反馈做误差闭环。
 *
 * @tparam BufferCapacity 轨迹点缓冲区容量
 * @note BufferCapacity >= 轨迹时长 s * (主机发送频率 Hz - trajectoryUpdate 调用频率 Hz),
 *       buffer 占用空间 BufferCapacity * 24 Byte
 */
template <size_t BufferCapacity> class Slave : public IChassisController
{
public:
    /// 三个自由度各自的 MIT 风格 PD 参数。
    struct PDConfig
    {
        MITPD::Config vx; ///< x 速度 PD 控制器
        MITPD::Config vy; ///< y 速度 PD 控制器
        MITPD::Config wz; ///< 角速度 PD 控制器
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
    void trajectoryUpdate()
    {
        if (!enabled())
            return;
        TrajectoryPoint point;
        // 缓冲区里没有点
        if (!cmd_buffer_.pop(point))
            return;
        p_ref_ = point.p_ref;
        v_ref_ = point.v_ref;
    }

    /**
     * 误差跟踪，此时我们在 body frame 下跟踪。
     *
     * 这样 Motion 层始终接收车体系速度，不需要理解世界系参考点。
     *
     * 作者注：这个原因是 AI 给出的，设计上没有这么考虑；
     *        目前测试看来，直接在世界下跟踪也没有产生什么问题
     */
    void errorUpdate()
    {
        if (!enabled())
            return;

        const auto& [x, y, yaw]  = loc_->WorldPosture2BodyPosture(p_ref_);
        const auto& [vx, vy, wz] = loc_->WorldVelocity2BodyVelocity(v_ref_);
        pd_vx_.calc(x, 0, vx, velocityInBody().vx);
        pd_vy_.calc(y, 0, vy, velocityInBody().vy);
        pd_wz_.calc(yaw, 0, wz, velocityInBody().wz);

        apply_position_velocity();
    }

    void stop() override
    {
        osMutexAcquire(lock_, osWaitForever);
        const uint32_t saved = isr_lock();

        // stop 时把“当前位置、零速度”作为新的参考点，避免控制器继续追旧轨迹。
        stopped_ = true;

        p_ref_ = loc_->postureInWorld();
        v_ref_ = { 0, 0, 0 };

        isr_unlock(saved);
        osMutexRelease(lock_);
    }

    /**
     * 轨迹点
     * @param point 轨迹点
     * @return true 表示成功入队；false 表示缓冲区已满，轨迹点未写入
     */
    bool pushTrajectoryPoint(const TrajectoryPoint& point) { return cmd_buffer_.push(point); }

private:
    osMutexId_t lock_;            ///< 保护 stop 等状态切换
    bool        stopped_{ true }; ///< 预留的停止标志，当前主要用于表达控制状态

    MITPD pd_vx_; ///< x 速度 PD 控制器
    MITPD pd_vy_; ///< y 速度 PD 控制器
    MITPD pd_wz_; ///< 角速度 PD 控制器

    Posture  p_ref_{}; ///< 当前正在跟踪的参考位姿
    Velocity v_ref_{}; ///< 当前正在跟踪的参考速度

    libs::RingBuffer<TrajectoryPoint, BufferCapacity> cmd_buffer_;

    void apply_position_velocity()
    {
        // 叠加世界系前馈和误差闭环输出后，再统一转换为车体系速度命令。
        const auto& [vx, vy, wz] = loc_->WorldVelocity2BodyVelocity(v_ref_);

        const Velocity velocity_in_body = {
            vx + pd_vx_.getOutput(),
            vy + pd_vy_.getOutput(),
            wz + pd_wz_.getOutput(),
        };

        // 应用速度
        applyVelocity(velocity_in_body);
    }
};

} // namespace chassis::controller
