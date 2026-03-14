/**
 * @file    LocEKF.cpp
 * @author  syhanjin
 * @date    2026-03-11
 */
#include "LocEKF.hpp"

#include "IChassis.hpp"

namespace chassis_loc
{
LocEKF::PositionEKF::PositionEKF(const Config& cfg) :
    EKF(cfg.x_init.vec(), cfg.covQ.mat(), cfg.noiseQ.mat()), R_gyro_(cfg.noiseR.gyro.mat()),
    R_lidar_(cfg.noiseR.lidar.mat())
{
}

auto LocEKF::PositionEKF::state() const
{
    return x;
}

auto LocEKF::PositionEKF::covariance() const
{
    return P;
}

void LocEKF::PositionEKF::reset(const VecS& x, const MatS& P)
{
    this->x = x;
    this->P = P;
}

void LocEKF::PositionEKF::odomUpdate(const Velocity& vel, const float dt)
{
    auto next                = x;
    const auto& [vx, vy, wz] = vel;

    const auto yaw_w_rad = DEG2RAD(x[2] + x[3]);
    const auto c = cosf(yaw_w_rad), s = sinf(yaw_w_rad);

    next[0] += (vx * c - vy * s) * dt;
    next[1] += (vx * s + vy * c) * dt;

    MatS F  = MatS::identity();
    F[0][2] = (-vx * s - vy * c) * dt;
    F[0][3] = F[0][2];
    F[1][2] = (vx * c - vy * s) * dt;
    F[1][3] = F[1][2];

    predict(next, F);
}

void LocEKF::PositionEKF::gyroUpdate(const float& yaw)
{
    // gyro 观测 yaw
    using Vec1 = math::Vec<float, 1>;
    constexpr math::Mat<float, 1, 4> H{ { { 0, 0, 1, 0 } } };
    update(Vec1{ yaw - x[2] }, H, R_gyro_);
}

void LocEKF::PositionEKF::lidarUpdate(const Posture& pos)
{
    constexpr math::Mat<float, 3, 4> H{
        {
                { 1, 0, 0, 0 },
                { 0, 1, 0, 0 },
                { 0, 0, 1, 1 },
        },
    };
    const math::Vec3f y{ pos.x - x[0], pos.y - x[1], pos.yaw - x[2] - x[3] };
    update(y, H, R_lidar_);
}

void LocEKF::updateInput()
{
    const auto vel = chassis_->forwardGetVelocity();
    const auto yaw = gyro_.getYaw();
    // 将输入丢进缓冲区
    input_buffer_.push_back({ HAL_GetTick(), vel, yaw });
}

void LocEKF::updateEKF()
{
    while (!input_buffer_.empty())
    {
        const auto& [ticks, vel, yaw] = input_buffer_[0];
        input_buffer_.pop_front();
        // 1. 更新预测
        // TODO: HAL 库有没有什么办法告诉我 tick 更新频率的
        pos_ekf_.odomUpdate(vel, dt());
        // 2. 更新陀螺仪观测
        pos_ekf_.gyroUpdate(yaw);
        // 保存状态历史
        state_buffer_.push_back({ .x     = pos_ekf_.state(),
                                  .P     = pos_ekf_.covariance(),
                                  .input = { .vel = vel, .yaw_gyro = yaw } });
    }
    // TODO: 处理这一步的线程安全问题
    updateLoc();
}
void LocEKF::updateLoc()
{
    const auto s = pos_ekf_.state();

    posture_ = { { s[0], s[1], s[2] + s[3] } };

    const auto [vx, vy, wz] = chassis_->forwardGetVelocity();

    velocity_.in_body = { vx, vy, gyro_.getWz() };

    velocity_.in_world = BodyVelocity2WorldVelocity(velocity_.in_body);
}

void LocEKF::update()
{
    updateInput();
    // 如果锁了，说明雷达更在更新，跳过本次更新，等雷达更新完调用更新 EKF
    if (!lock_.load(std::memory_order_acquire))
    {
        updateEKF();
        // 通过状态更新定位数据
    }
}

void LocEKF::updateLidar(const Posture& pos, const uint32_t ticks)
{
    // 锁定更新状态, 此处假定 updateEKF 由中断调用，本身不会被打断
    lock_.store(true, std::memory_order_acquire);

    const uint32_t last_tick = state_buffer_.empty() ? 0 : state_buffer_.at(-1).input.ticks;

    if (ticks >= last_tick)
    {
        // 直接更新
        pos_ekf_.lidarUpdate(pos);
        return;
    }
    const int state_tick = -static_cast<int>(
            std::ceilf(static_cast<float>(last_tick - ticks) / static_cast<float>(dticks_)));
    // 获取回溯状态
    auto s = state_buffer_[state_tick];
    // 回溯到之前的状态
    pos_ekf_.reset(s.x, s.P);
    // 更新雷达
    pos_ekf_.lidarUpdate(pos);
    s.x = pos_ekf_.state(), s.P = pos_ekf_.covariance();

    for (auto& [x, P, input] : state_buffer_.range(state_tick + 1, 0))
    {
        // 重放并更新状态
        const auto [_, vel, yaw] = input;
        pos_ekf_.odomUpdate(vel, dt());
        pos_ekf_.gyroUpdate(yaw);
        x = pos_ekf_.state(), P = pos_ekf_.covariance();
    }

    lock_.store(false, std::memory_order_acquire);
    // 手动更新堆积的输入
    if (!input_buffer_.empty())
        updateEKF();
}

LocEKF::LocEKF(const Config& cfg, sensors::gyro::HWT101CT& gyro, const uint32_t delta_ticks) :
    gyro_(gyro), pos_ekf_(cfg), dticks_(delta_ticks)
{
}

} // namespace chassis_loc