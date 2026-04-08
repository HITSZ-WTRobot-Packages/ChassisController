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
#include "HWT101CT.hpp"
#include "IChassisLoc.hpp"
#include "Vec.hpp"
#include "EKF.hpp"
#include "Mat.hpp"
#include <mutex>

#ifndef DEG2RAD
#    define DEG2RAD(__DEG__) ((__DEG__) * (float)3.14159265358979323846f / 180.0f)
#endif

namespace chassis::loc
{

/**
 * @brief ESKF 定位系统实现类
 *
 * 特性：
 * - 继承 IChassisLoc 接口，与底盘系统统一对接
 * - 封装 ESKF 算法，提供轮速 + 陀螺仪 + 外部位姿观测的融合能力
 * - 自动同步 ESKF 状态到 IChassisLoc 的 posture 和 velocity
 *
 * 时间基准边界：
 * - 本类默认 `update()` 和 `updateLidar()` 使用的是同一时间基准
 * - 如果外部观测来自另一套时钟，必须先由接入工程完成对时或时间映射
 * - 本驱动库不负责上下位机对时
 */
class LocEKF : public IChassisLoc
{
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
        using VecS = VecS;
        using MatS = MatS;
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

        explicit PositionEKF(const Config& cfg);
        /// 返回当前 EKF 状态向量。
        [[nodiscard]] auto state() const;
        /// 返回当前状态协方差。
        [[nodiscard]] auto covariance() const;

        /// 用指定状态和协方差直接覆盖滤波器，供回溯重放使用。
        void reset(const VecS& x, const MatS& P);

        /// 只用轮速里程计推进一次预测。
        void odomUpdate(const Velocity& vel, float dt);

        /// 用陀螺仪 yaw 做一次量测更新。
        void gyroUpdate(const float& yaw);

        /// 用外部绝对位姿做一次量测更新。
        void lidarUpdate(const Posture& pos);

    private:
        math::Mat<float, 1, 1> R_gyro_;
        math::Mat<float, 3, 3> R_lidar_;
    };
    PositionEKF pos_ekf_; ///< 真正执行滤波计算的内部对象

private:
    struct Input
    {
        uint32_t ticks{ HAL_GetTick() }; ///< 输入对应的时间戳；默认来自本地 tick，若外部观测参与回放，工程侧需先把时间基准对齐
        Velocity vel{};                  ///< 该时刻的里程计预测输入
        float    yaw_gyro{};             ///< 该时刻的陀螺仪 yaw 观测
    };

    struct StatePoint
    {
        // 在一个输入样本被完整处理之后的滤波状态。
        PositionEKF::VecS x;
        PositionEKF::MatS P;

        Input input; ///< 推导出该状态时所使用的输入样本
    };

    // TODO: 缓存的观测点 / 历史状态数量可以改成由外部配置，以便在不同项目里按回放需求和 RAM 预算做权衡，避免固定 512 带来不必要的内存占用。
    Deque<StatePoint, 512> state_buffer_; ///< 历史状态，用于处理晚到观测回放
    Deque<Input, 4>        input_buffer_; ///< 尚未推进进 EKF 的输入样本队列

    uint32_t dticks_{ 1 }; ///< 相邻 update 之间的 tick 间隔

    AtomicFlagLock lock_; ///< 雷达回放时用于阻止并发 updateEKF()

    /// 约定 HAL tick 为 1ms，因此这里直接换算成秒。
    [[nodiscard]] constexpr float dt() const { return static_cast<float>(dticks_) * 0.001f; }

    /// 把当前轮速和陀螺仪采样压入输入缓冲。
    void updateInput();
    /// 消费输入缓冲并推进 EKF，然后同步对外可读状态。
    void updateEKF();
    /// 根据当前 EKF 状态刷新对外暴露的 posture / velocity 缓冲。
    void updateLoc();

    // 注意：上下位机对时不属于本驱动库职责范围。
    // 如果外部观测与 update() 内部时间戳不在同一时间基准，必须先在接入工程完成时间映射。

public:
    using Config = PositionEKF::Config;

    /**
     * 按固定 dt 采样底盘速度和陀螺仪，并推进一次滤波状态。
     *
     * dt 由构造参数 delta_ticks 决定，默认假设 tick 时间基准为 1ms。
     */
    void update();
    /**
     * 注入一帧外部位姿观测。
     *
     * @param pos   外部观测的世界系位姿，单位语义与 Posture 一致
     * @param ticks 该观测对应的时间戳，必须与内部 update() 采样使用的时间基准一致；
     *              若观测来自另一套时钟，需先由接入工程换算到统一时基后再传入
     *
     * 如果观测晚到但仍在历史缓冲区范围内，当前实现会回溯状态并重放后续输入。
     */
    void updateLidar(const Posture& pos, uint32_t ticks);

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
           uint32_t                 delta_ticks = 1);

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
        Velocity in_world;
        Velocity in_body;
    } velocity_[2]{};

    std::atomic<size_t> idx_{ 0 }; ///< 当前对外可见缓冲区下标

    /// 只有两个缓冲区，因此这里直接在 0 / 1 之间切换。
    [[nodiscard]] uint32_t next_idx() const
    {
        return (idx_.load(std::memory_order_relaxed) + 1) & (2 - 1);
    }
};

} // namespace chassis::loc
