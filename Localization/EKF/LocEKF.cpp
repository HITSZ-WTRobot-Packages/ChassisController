/**
 * @file    LocEKF.cpp
 * @author  syhanjin
 * @date    2026-03-11
 * @brief   基于扩展卡尔曼滤波的底盘定位后端。
 */
#include "LocEKF.hpp"

#include "IChassisMotion.hpp"

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

namespace chassis::loc
{
// 构造时把各种噪声配置转成底层 EKF 需要的矩阵形式。
LocEKF::PositionEKF::PositionEKF(const Config& cfg) :
    EKF(cfg.x_init.vec(), cfg.covP.mat(), cfg.noiseQ.mat()), R_gyro_(cfg.noiseR.gyro.mat()),
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

    // 预测时使用当前估计的世界系朝向，把车体系速度展开到世界系。
    const auto yaw_w_rad = DEG2RAD(x[2] + x[3]);
    const auto c = cosf(yaw_w_rad), s = sinf(yaw_w_rad);

    next[0] += (vx * c - vy * s) * dt;
    next[1] += (vx * s + vy * c) * dt;

    // F 是状态转移对状态的雅可比矩阵，只对 yaw / yaw_offset 引起的位置变化做线性化。
    MatS F  = MatS::identity();
    F[0][2] = (-vx * s - vy * c) * dt;
    F[0][3] = F[0][2];
    F[1][2] = (vx * c - vy * s) * dt;
    F[1][3] = F[1][2];

    predict(next, F);
}

void LocEKF::PositionEKF::gyroUpdate(const float& yaw)
{
    // gyro 只直接观测状态里的 yaw，本身不直接观测 yaw_offset。
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
    // 注意这里统一使用角度制，而不是弧度制。
    // 连续化处理雷达的角度数据，使其单次误差不会超过 180deg
    constexpr auto wrap = [](float a)
    {
        while (a > 180)
            a -= 360;
        while (a <= -180)
            a += 360;

        return a;
    };

    const math::Vec3f y{ pos.x - x[0], pos.y - x[1], wrap(pos.yaw - x[2] - x[3]) };
    update(y, H, R_lidar_);
}

void LocEKF::updateInput()
{
    const auto vel = forwardGetVelocity();
    const auto yaw = gyro_.getYaw();
    // 把本周期输入先缓存起来，真正推进滤波由 updateEKF() 统一处理。
    // 这里记录的 ticks 默认就是本地时间基准；若后续要和外部观测做回放匹配，
    // 工程侧传入 updateLidar() 的 ticks 也必须先换算到同一基准。
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
        // TODO: fixbug: 这里应保存原始输入对应的 ticks；当前省略 ticks 会导致晚到观测回放时
        // 使用的状态时间戳不一定就是该输入的真实时间戳。
        state_buffer_.push_back({ .x     = pos_ekf_.state(),
                                  .P     = pos_ekf_.covariance(),
                                  .input = { .vel = vel, .yaw_gyro = yaw } });
    }
    updateLoc();
}
void LocEKF::updateLoc()
{
    const auto s = pos_ekf_.state();

    const auto nxt = next_idx();

    // 对外暴露的 yaw = 主状态 yaw + 为兼容外部绝对观测而估计出的 yaw_offset。
    posture_[nxt] = { { s[0], s[1], s[2] + s[3] } };

    const auto [vx, vy, wz] = forwardGetVelocity();

    // 角速度优先使用陀螺仪 wz，平移速度则继续使用 Motion 反馈。
    velocity_[nxt].in_body = { vx, vy, gyro_.getWz() };

    velocity_[nxt].in_world = rotateVelocity(velocity_[nxt].in_body, posture_[nxt].in_world.yaw);

    idx_.store(nxt, std::memory_order_release);
}

void LocEKF::update()
{
    updateInput();
    // 如果锁了，说明雷达更在更新，跳过本次更新，等雷达更新完调用更新 EKF
    if (!lock_.is_locked())
    {
        updateEKF();
        // 通过状态更新定位数据
    }
}

void LocEKF::updateLidar(const Posture& pos, const uint32_t ticks)
{
    // 锁定更新状态, 此处假定 updateEKF 由中断调用，本身不会被打断
    AtomicFlagGuard guard(lock_);
    // 注意：这里不会做上下位机对时。调用方传入的 ticks 必须已经是统一时间基准，
    // 否则后面的“是否晚到”“需要回放多少状态”判断都会失真。

    const uint32_t last_tick = state_buffer_.empty() ? 0 : state_buffer_.at(-1).input.ticks;

    if (ticks >= last_tick)
    {
        // 这是一条不晚于当前状态的观测，可以直接融合，无需回溯。
        pos_ekf_.lidarUpdate(pos);
        return;
    }
    const int state_tick = static_cast<int>(
            std::ceilf(static_cast<float>(last_tick - ticks) / static_cast<float>(dticks_)));

    if (static_cast<int>(state_buffer_.size()) < state_tick)
    {
        // 雷达数据太早，超出保存的状态，跳过。
        return;
    }

    // 获取回溯状态
    auto& s = state_buffer_[-state_tick];
    // 回溯到之前的状态
    pos_ekf_.reset(s.x, s.P);
    // 更新雷达
    pos_ekf_.lidarUpdate(pos);
    s.x = pos_ekf_.state(), s.P = pos_ekf_.covariance();

    for (auto& [x, P, input] : state_buffer_.range(-state_tick + 1, 0))
    {
        // 重放并更新状态，让历史观测重新影响到现在。
        const auto [_, vel, yaw] = input;
        pos_ekf_.odomUpdate(vel, dt());
        pos_ekf_.gyroUpdate(yaw);
        x = pos_ekf_.state(), P = pos_ekf_.covariance();
    }

    // 手动更新堆积的输入
    if (!input_buffer_.empty())
        updateEKF();
}

LocEKF::LocEKF(motion::IChassisMotion&  motion,
               const Config&            cfg,
               sensors::gyro::HWT101CT& gyro,
               const uint32_t           delta_ticks) :
    IChassisLoc(motion), gyro_(gyro), pos_ekf_(cfg), dticks_(delta_ticks)
{
}

} // namespace chassis::loc
