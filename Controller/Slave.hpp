/**
 * @file    Slave.hpp
 * @author  syhanjin
 * @date    2026-02-24
 */
#pragma once
#include "IChassis.hpp"
#include "isr_lock.h"
#include "RingBuffer.hpp"

#include <algorithm>
#include <type_traits>

namespace chassis::controller
{

/**
 * 从机控制模式的底盘
 * @tparam Chassis 底盘类型
 * @tparam BufferCapacity 轨迹点缓冲区容量
 * @note BufferCapacity >= 轨迹时长 s * (主机发送频率 Hz - trajectoryUpdate 调用频率 Hz),
 *       buffer 占用空间 BufferCapacity * 24 Byte
 */
template <typename Chassis, size_t BufferCapacity> class Slave : public Chassis
{
    static_assert(std::is_base_of_v<IChassis, Chassis>);

public:
    using Velocity = IChassis::Velocity;
    using Posture  = IChassis::Posture;

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

    Slave(Chassis&& chassis, const PDConfig& pd_cfg);

    void trajectoryUpdate();
    void errorUpdate();
    void controllerUpdate();

    void stop();
    bool pushTrajectoryPoint(const TrajectoryPoint& point);

    bool enable()
    {
        stop();
        return Chassis::enable();
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

    void apply_position_velocity();
};

template <typename Chassis, size_t BufferCapacity>
Slave<Chassis, BufferCapacity>::Slave(Chassis&& chassis, const PDConfig& pd_cfg) :
    Chassis(std::move(chassis)), lock_(osMutexNew(nullptr)), pd_vx_(pd_cfg.vx), pd_vy_(pd_cfg.vy),
    pd_wz_(pd_cfg.wz)
{
}
template <typename Chassis, size_t BufferCapacity>
void Slave<Chassis, BufferCapacity>::trajectoryUpdate()
{
    if (!this->enabled())
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
template <typename Chassis, size_t BufferCapacity>
void Slave<Chassis, BufferCapacity>::errorUpdate()
{
    if (!this->enabled())
        return;

    const auto& [x, y, yaw]  = this->WorldPosture2BodyPosture(p_ref_);
    const auto& [vx, vy, wz] = this->WorldVelocity2BodyVelocity(v_ref_);
    pd_vx_.calc(x, 0, vx, this->velocity().in_body.vx);
    pd_vy_.calc(y, 0, vy, this->velocity().in_body.vy);
    pd_wz_.calc(yaw, 0, wz, this->velocity().in_body.wz);

    apply_position_velocity();
}

template <typename Chassis, size_t BufferCapacity>
void Slave<Chassis, BufferCapacity>::controllerUpdate()
{
    if (!this->enabled())
        return;
    this->velocityControllerUpdate();
}

template <typename Chassis, size_t BufferCapacity> void Slave<Chassis, BufferCapacity>::stop()
{
    osMutexAcquire(lock_, osWaitForever);
    const uint32_t saved = isr_lock();

    stopped_ = true;

    p_ref_ = this->posture().in_world;
    v_ref_ = { 0, 0, 0 };

    isr_unlock(saved);
    osMutexRelease(lock_);
}
/**
 * 轨迹点
 * @param point 轨迹点
 * @return 是否已满
 */
template <typename Chassis, size_t BufferCapacity>
bool Slave<Chassis, BufferCapacity>::pushTrajectoryPoint(const TrajectoryPoint& point)
{
    return cmd_buffer_.push(point);
}
template <typename Chassis, size_t BufferCapacity>
void Slave<Chassis, BufferCapacity>::apply_position_velocity()
{
    // 叠加前馈和 pd 输出
    const auto& [vx, vy, wz] = this->WorldVelocity2BodyVelocity(v_ref_);

    const Velocity velocity_in_body = {
        vx + pd_vx_.getOutput(),
        vy + pd_vy_.getOutput(),
        wz + pd_wz_.getOutput(),
    };

    // 应用速度
    this->applyVelocity(velocity_in_body);
}

} // namespace chassis::controller
