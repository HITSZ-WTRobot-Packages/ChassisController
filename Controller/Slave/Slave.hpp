/**
 * @file    Slave.hpp
 * @author  syhanjin
 * @date    2026-02-24
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
 * @tparam BufferCapacity 轨迹点缓冲区容量
 * @note BufferCapacity >= 轨迹时长 s * (主机发送频率 Hz - trajectoryUpdate 调用频率 Hz),
 *       buffer 占用空间 BufferCapacity * 24 Byte
 */
template <size_t BufferCapacity> class Slave : public IChassisController
{
public:
    struct PDConfig
    {
        MITPD::Config vx; ///< x 速度 PD 控制器
        MITPD::Config vy; ///< y 速度 PD 控制器
        MITPD::Config wz; ///< 角速度 PD 控制器
    };

    struct TrajectoryPoint
    {
        Posture  p_ref;
        Velocity v_ref;
    };

    Slave(motion::IChassisMotion& motion, loc::IChassisLoc& loc, const PDConfig& pd_cfg) :
        IChassisController(motion, loc), lock_(osMutexNew(nullptr)), pd_vx_(pd_cfg.vx),
        pd_vy_(pd_cfg.vy), pd_wz_(pd_cfg.wz)
    {
    }

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
     * 误差跟踪，此时我们在 body frame 下跟踪
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

    void controllerUpdate() const
    {
        if (!enabled())
            return;
        velocityControllerUpdate();
    }

    void stop()
    {
        osMutexAcquire(lock_, osWaitForever);
        const uint32_t saved = isr_lock();

        stopped_ = true;

        p_ref_ = loc_->postureInWorld();
        v_ref_ = { 0, 0, 0 };

        isr_unlock(saved);
        osMutexRelease(lock_);
    }

    /**
     * 轨迹点
     * @param point 轨迹点
     * @return 是否已满
     */
    bool pushTrajectoryPoint(const TrajectoryPoint& point) { return cmd_buffer_.push(point); }

    bool enable()
    {
        stop();
        return IChassisController::enable();
    }

private:
    osMutexId_t lock_;
    bool        stopped_{ true };

    MITPD pd_vx_; ///< x 速度 PD 控制器
    MITPD pd_vy_; ///< y 速度 PD 控制器
    MITPD pd_wz_; ///< 角速度 PD 控制器

    Posture  p_ref_{};
    Velocity v_ref_{};

    libs::RingBuffer<TrajectoryPoint, BufferCapacity> cmd_buffer_;

    void apply_position_velocity() const
    {
        // 叠加前馈和 pd 输出
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
