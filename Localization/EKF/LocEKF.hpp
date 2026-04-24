/**
 * @file    LocEKF.hpp
 * @author  syhanjin
 * @date    2026-03-11
 * @brief   基于扩展卡尔曼滤波的底盘定位后端。
 *
 * 使用 fastlio2(10Hz) + encoder(1kHz) + HWT101CT(yaw, 1kHz) 融合定位
 */
#pragma once
#include "AtomicFlagLock.hpp"
#include "Deque.hpp"
#include "EKF.hpp"
#include "HWT101CT.hpp"
#include "IChassisLoc.hpp"
#include "Mat.hpp"
#include "Vec.hpp"
#include <atomic>
#include <cmath>
#include <cstddef>

#ifndef DEG2RAD
#    define DEG2RAD(__DEG__) ((__DEG__) * (float)3.14159265358979323846f / 180.0f)
#endif

namespace chassis::loc
{

/**
 * @brief 可配置历史缓冲容量的 ESKF 定位系统实现类。
 *
 * 特性：
 * - 继承 IChassisLoc 接口，与底盘系统统一对接
 * - 封装 ESKF 算法，提供轮速 + 陀螺仪 + 外部位姿观测的融合能力
 * - 自动同步 ESKF 状态到 IChassisLoc 的 posture 和 velocity
 * - 历史状态缓冲容量可在工程接入时按回放需求和 RAM 预算做权衡
 *
 * 时间基准边界：
 * - 本类默认 `update()` 和 `updateLidar()` 使用的是同一时间基准
 * - 如果外部观测来自另一套时钟，必须先由接入工程完成对时或时间映射
 * - 本驱动库不负责上下位机对时
 *
 * @tparam StateBufferCapacity 保存多少个历史状态点，用于晚到观测回放
 * @tparam InputBufferCapacity 保存多少个尚未推进进 EKF 的输入样本，默认 4
 */
template <std::size_t StateBufferCapacity, std::size_t InputBufferCapacity = 4>
class LocEKF : public IChassisLoc
{
    static_assert(StateBufferCapacity > 0, "LocEKF state buffer capacity must be > 0");
    static_assert(InputBufferCapacity > 0, "LocEKF input buffer capacity must be > 0");

public:
    sensors::gyro::HWT101CT& gyro_; ///< 提供 yaw / wz 观测的陀螺仪

private:
    /**
     * 内部位置 EKF。
     *
     * 状态向量是 `[x, y, yaw, yaw_offset]`，
     * 其中 yaw_offset 用来吸收陀螺仪与外部绝对观测之间可能存在的偏差。
     */
    class PositionEKF : math::ekf::EKF<float, 4>
    {
        // state: [x, y, yaw, yaw_offset]
        // x, y: m.
        // yaw, yaw_offset: deg
        // wz: deg/s
    public:
        using EKF::MatS;
        using EKF::VecS;

    public:
        struct Config
        {
            // 初始状态。通常由上电时的已知位姿给出，yaw_offset 一般从 0 开始。
            // 若工程里把“雷达首次返回的位姿 + 陀螺仪首次数据”作为初始观测，则常见做法是
            // 先等待这两类数据，再用它们准备 x_init 等初始条件，随后才构造 LocEKF。
            struct
            {
                float x, y;
                float yaw;
                float yaw_offset;

                [[nodiscard]] constexpr VecS vec() const { return VecS{ x, y, yaw, yaw_offset }; }
            } x_init;

            // 初始不确定度。值越大，表示越不相信初始化状态。
            struct
            {
                float xy;
                float yaw;
                float yaw_offset;

                [[nodiscard]] constexpr MatS mat() const
                {
                    return math::mat::diag(VecS{ xy, xy, yaw, yaw_offset });
                }
            } covP;

            // 过程噪声。值越大，表示越不相信仅靠轮速积分得到的预测过程。
            struct
            {
                float xy;
                float yaw;
                float yaw_offset;

                [[nodiscard]] constexpr MatS mat() const
                {
                    return math::mat::diag(VecS{ xy, xy, yaw, yaw_offset });
                }
            } noiseQ;

            struct
            {
                struct
                {
                    float yaw; ///< 陀螺仪 yaw 观测噪声

                    [[nodiscard]] constexpr auto mat() const
                    {
                        return math::mat::diag(math::Vecf<1>{ yaw });
                    }
                } gyro;
                struct
                {
                    float xy;  ///< 外部位姿观测中的平移噪声
                    float yaw; ///< 外部位姿观测中的航向噪声

                    [[nodiscard]] constexpr auto mat() const
                    {
                        return math::mat::diag(math::Vecf<3>{ xy, xy, yaw });
                    }
                } lidar;
            } noiseR;
        };

        explicit PositionEKF(const Config& cfg) :
            EKF(cfg.x_init.vec(), cfg.covP.mat(), cfg.noiseQ.mat()), R_gyro_(cfg.noiseR.gyro.mat()),
            R_lidar_(cfg.noiseR.lidar.mat())
        {
        }

        /// 返回当前 EKF 状态向量。
        [[nodiscard]] auto state() const { return x; }
        /// 返回当前状态协方差。
        [[nodiscard]] auto covariance() const { return P; }

        /// 用指定状态和协方差直接覆盖滤波器，供回溯重放使用。
        void reset(const VecS& state, const MatS& covariance)
        {
            x = state;
            P = covariance;
        }

        /// 只用轮速里程计推进一次预测。
        void odomUpdate(const Velocity& vel, const float dt)
        {
            auto next                = x;
            const auto& [vx, vy, wz] = vel;
            (void)wz;

            // 预测时使用当前估计的世界系朝向，把车体系速度展开到世界系。
            const auto yaw_w_rad = DEG2RAD(x[2] + x[3]);
            const auto c         = cosf(yaw_w_rad);
            const auto s         = sinf(yaw_w_rad);

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

        /// 用陀螺仪 yaw 做一次量测更新。
        void gyroUpdate(const float& yaw)
        {
            // gyro 只直接观测状态里的 yaw，本身不直接观测 yaw_offset。
            constexpr math::Mat<float, 1, 4> H{ { { 0, 0, 1, 0 } } };
            update(math::Vec<float, 1>{ yaw - x[2] }, H, R_gyro_);
        }

        /// 用外部绝对位姿做一次量测更新。
        void lidarUpdate(const Posture& pos)
        {
            constexpr math::Mat<float, 3, 4> H{
                {
                        { 1, 0, 0, 0 },
                        { 0, 1, 0, 0 },
                        { 0, 0, 1, 1 },
                },
            };
            // 注意这里统一使用角度制，而不是弧度制。
            // 连续化处理雷达的角度数据，使其单次误差不会超过 180deg。
            constexpr auto wrap = [](float a)
            {
                while (a > 180)
                    a -= 360;
                while (a <= -180)
                    a += 360;

                return a;
            };

            const math::Vec3f y{
                pos.x - x[0],
                pos.y - x[1],
                wrap(pos.yaw - x[2] - x[3]),
            };
            update(y, H, R_lidar_);
        }

    private:
        math::Mat<float, 1, 1> R_gyro_;
        math::Mat<float, 3, 3> R_lidar_;
    };
    PositionEKF pos_ekf_; ///< 真正执行滤波计算的内部对象

private:
    struct Input
    {
        uint32_t ticks{ HAL_GetTick() }; ///< 输入对应的时间戳；默认来自本地
                                         ///< tick，若外部观测参与回放，工程侧需先把时间基准对齐
        Velocity vel{};                  ///< 该时刻的里程计预测输入
        float    yaw_gyro{};             ///< 该时刻的陀螺仪 yaw 观测
    };

    struct StatePoint
    {
        // 在一个输入样本被完整处理之后的滤波状态。
        typename PositionEKF::VecS x;
        typename PositionEKF::MatS P;

        Input input; ///< 推导出该状态时所使用的输入样本
    };

    Deque<StatePoint, StateBufferCapacity> state_buffer_; ///< 历史状态，用于处理晚到观测回放
    Deque<Input, InputBufferCapacity>      input_buffer_; ///< 尚未推进进 EKF 的输入样本队列

    uint32_t dticks_{ 1 }; ///< 相邻 update 之间的 tick 间隔

    AtomicFlagLock lock_; ///< 雷达回放时用于阻止并发 updateEKF()

    /// 约定 HAL tick 为 1ms，因此这里直接换算成秒。
    [[nodiscard]] constexpr float dt() const { return static_cast<float>(dticks_) * 0.001f; }

    /// 把当前轮速和陀螺仪采样压入输入缓冲。
    void updateInput()
    {
        const auto vel = forwardGetVelocity();
        const auto yaw = gyro_.getYaw();
        // 把本周期输入先缓存起来，真正推进滤波由 updateEKF() 统一处理。
        // 这里记录的 ticks 默认就是本地时间基准；若后续要和外部观测做回放匹配，
        // 工程侧传入 updateLidar() 的 ticks 也必须先换算到同一基准。
        input_buffer_.push_back({ HAL_GetTick(), vel, yaw });
    }

    /// 消费输入缓冲并推进 EKF，然后同步对外可读状态。
    void updateEKF()
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
            // 保存状态历史。这里保存原始输入对应的 ticks，以便晚到观测按统一时间基准回放。
            state_buffer_.push_back({ .x     = pos_ekf_.state(),
                                      .P     = pos_ekf_.covariance(),
                                      .input = { .ticks = ticks, .vel = vel, .yaw_gyro = yaw } });
        }
        updateLoc();
    }

    /// 根据当前 EKF 状态刷新对外暴露的 posture / velocity 缓冲。
    void updateLoc()
    {
        const auto s = pos_ekf_.state();

        const auto nxt = next_idx();

        // 对外暴露的 yaw = 主状态 yaw + 为兼容外部绝对观测而估计出的 yaw_offset。
        posture_[nxt] = { { s[0], s[1], s[2] + s[3] } };

        const auto [vx, vy, wz] = forwardGetVelocity();
        (void)wz;

        // 角速度优先使用陀螺仪 wz，平移速度则继续使用 Motion 反馈。
        velocity_[nxt].in_body = { vx, vy, gyro_.getWz() };

        velocity_[nxt].in_world = rotateVelocity(velocity_[nxt].in_body,
                                                 posture_[nxt].in_world.yaw);

        idx_.store(nxt, std::memory_order_release);
    }

    // 注意：上下位机对时不属于本驱动库职责范围。
    // 如果外部观测与 update() 内部时间戳不在同一时间基准，必须先在接入工程完成时间映射。

public:
    using Config = typename PositionEKF::Config;

    /**
     * 按固定 dt 采样底盘速度和陀螺仪，并推进一次滤波状态。
     *
     * dt 由构造参数 delta_ticks 决定，默认假设 tick 时间基准为 1ms。
     */
    void update()
    {
        updateInput();
        // 如果锁了，说明雷达在更新，跳过本次更新，等雷达更新完调用更新 EKF。
        if (!lock_.is_locked())
        {
            updateEKF();
            // 通过状态更新定位数据
        }
    }

    /**
     * 注入一帧外部位姿观测。
     *
     * @param pos   外部观测的世界系位姿，单位语义与 Posture 一致
     * @param ticks 该观测对应的时间戳，必须与内部 update() 采样使用的时间基准一致；
     *              若观测来自另一套时钟，需先由接入工程换算到统一时基后再传入
     *
     * 如果观测晚到但仍在历史缓冲区范围内，当前实现会回溯状态并重放后续输入。
     * 可回放窗口长度由模板参数 `StateBufferCapacity` 决定。
     */
    void updateLidar(const Posture& pos, const uint32_t ticks)
    {
        // 锁定更新状态, 此处假定 updateEKF 由中断调用，本身不会被打断。
        AtomicFlagGuard guard(lock_);
        // 注意：这里不会做上下位机对时。调用方传入的 ticks 必须已经是统一时间基准，
        // 否则后面的“是否晚到”“需要回放多少状态”判断都会失真。

        const uint32_t last_tick = state_buffer_.empty() ? 0 : state_buffer_.at(-1).input.ticks;

        if (ticks >= last_tick)
        {
            // 这是一条不晚于当前状态的观测，可以直接融合，无需回溯。
            pos_ekf_.lidarUpdate(pos);
            updateLoc();
            return;
        }
        const int state_tick = static_cast<int>(
                std::ceilf(static_cast<float>(last_tick - ticks) / static_cast<float>(dticks_)));

        if (static_cast<int>(state_buffer_.size()) < state_tick)
        {
            // 雷达数据太早，超出保存的状态，跳过。
            return;
        }

        // 获取回溯状态。
        auto& s = state_buffer_[-state_tick];
        // 回溯到之前的状态。
        pos_ekf_.reset(s.x, s.P);
        // 更新雷达。
        pos_ekf_.lidarUpdate(pos);
        s.x = pos_ekf_.state();
        s.P = pos_ekf_.covariance();

        for (auto& [x, P, input] : state_buffer_.range(-state_tick + 1, 0))
        {
            // 重放并更新状态，让历史观测重新影响到现在。
            const auto [replay_ticks, vel, yaw] = input;
            (void)replay_ticks;
            pos_ekf_.odomUpdate(vel, dt());
            pos_ekf_.gyroUpdate(yaw);
            x = pos_ekf_.state();
            P = pos_ekf_.covariance();
        }

        // 手动更新堆积的输入。
        if (!input_buffer_.empty())
            updateEKF();
        else
            updateLoc();
    }

    /**
     * @param motion       作为里程计输入来源的 Motion
     * @param cfg          滤波配置
     * @param gyro         提供 yaw / wz 观测的陀螺仪
     * @param delta_ticks  两次 update() 调用之间预计相隔多少个 HAL tick
     *
     * 若初始状态依赖首次外部观测，推荐先让 Motion 独立运行，待拿到第一帧雷达位姿和
     * 第一帧陀螺仪数据，并准备好 cfg.x_init 等初始条件后，再构造本对象。
     */
    LocEKF(motion::IChassisMotion&  motion,
           const Config&            cfg,
           sensors::gyro::HWT101CT& gyro,
           const uint32_t           delta_ticks = 1) :
        IChassisLoc(motion), gyro_(gyro), pos_ekf_(cfg), dticks_(delta_ticks)
    {
    }

    /// 双缓冲读取，避免读到 update 中间态。
    [[nodiscard]] Velocity velocityInBody() const override
    {
        return velocity_[idx_.load(std::memory_order_acquire)].in_body;
    }
    /// 双缓冲读取，避免读到 update 中间态。
    [[nodiscard]] Velocity velocityInWorld() const override
    {
        return velocity_[idx_.load(std::memory_order_acquire)].in_world;
    }
    /// 双缓冲读取，避免读到 update 中间态。
    [[nodiscard]] Posture postureInWorld() const override
    {
        return posture_[idx_.load(std::memory_order_acquire)].in_world;
    }

private:
    struct
    {
        Posture in_world;
    } posture_[2]{};

    struct
    {
        Velocity in_body;
        Velocity in_world;
    } velocity_[2]{};

    std::atomic<size_t> idx_{ 0 }; ///< 当前对外可见缓冲区下标

    /// 只有两个缓冲区，因此这里直接在 0 / 1 之间切换。
    [[nodiscard]] uint32_t next_idx() const
    {
        return (idx_.load(std::memory_order_relaxed) + 1) & (2 - 1);
    }
};

} // namespace chassis::loc
