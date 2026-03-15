/**
 * @file    LocEKF.hpp
 * @author  syhanjin
 * @date    2026-03-11
 * @brief   ESKF-based Localization System Implementation
 *
 * 使用 fastlio2(10Hz) + encoder(1kHz) + HWT101CT(yaw, 1kHz) 融合定位
 */
#pragma once
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
 * @brief ESKF定位系统实现类
 *
 * 特性:
 * - 继承 ILoc 接口，与底盘系统无缝集成
 * - 封装 ESKF 算法库，提供多传感器融合能力
 * - 支持 IMU(高频)、轮速计(中频)、Lidar/SLAM(低频)数据融合
 * - 自动同步 ESKF 状态到 ILoc 的 posture 和 velocity
 */
class LocEKF : public IChassisLoc
{
public:
    sensors::gyro::HWT101CT& gyro_;

private:
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
            // 初始值，初始清零offset
            struct
            {
                float x, y;
                float yaw;
                float yaw_offset;

                [[nodiscard]] constexpr VecS vec() const { return VecS{ x, y, yaw, yaw_offset }; }
            } x_init;

            // 初始不确定度
            struct
            {
                float xy;
                float yaw;
                float yaw_offset;

                [[nodiscard]] constexpr MatS mat() const
                {
                    return math::mat::diag(VecS{ xy, xy, yaw, yaw_offset });
                }
            } covQ;

            // 过程噪声
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
                    float yaw;

                    [[nodiscard]] constexpr auto mat() const
                    {
                        return math::mat::diag(math::Vecf<1>{ yaw });
                    }
                } gyro;
                struct
                {
                    float xy;
                    float yaw;

                    [[nodiscard]] constexpr auto mat() const
                    {
                        return math::mat::diag(math::Vecf<3>{ xy, xy, yaw });
                    }
                } lidar;
            } noiseR;
        };

        explicit PositionEKF(const Config& cfg);
        [[nodiscard]] auto state() const;
        [[nodiscard]] auto covariance() const;

        void reset(const VecS& x, const MatS& P);

        void odomUpdate(const Velocity& vel, float dt);

        void gyroUpdate(const float& yaw);

        void lidarUpdate(const Posture& pos);

    private:
        math::Mat<float, 1, 1> R_gyro_;
        math::Mat<float, 3, 3> R_lidar_;
    };
    PositionEKF pos_ekf_;

private:
    struct Input
    {
        uint32_t ticks{ HAL_GetTick() };
        Velocity vel{};      // 该状态时的预测输入
        float    yaw_gyro{}; // 该状态时的陀螺仪观测输入
    };

    struct StatePoint
    {
        // 在输入完成之后的状态
        PositionEKF::VecS x;
        PositionEKF::MatS P;

        Input input;
    };

    Deque<StatePoint, 512> state_buffer_;
    Deque<Input, 4>        input_buffer_;

    uint32_t dticks_{ 1 };

    std::atomic<bool>             lock_{ false };
    [[nodiscard]] constexpr float dt() const { return static_cast<float>(dticks_) * 0.001f; }

    void updateInput();
    void updateEKF();
    void updateLoc();

    // TODO: 在上下位机之间同步时间

public:
    using Config = PositionEKF::Config;

    void update();
    void updateLidar(const Posture& pos, uint32_t ticks);

    LocEKF(motion::IChassisMotion&  motion,
           const Config&            cfg,
           sensors::gyro::HWT101CT& gyro,
           uint32_t                 delta_ticks = 1);

    [[nodiscard]] const Velocity& velocityInBody() const override { return velocity_.in_body; }
    [[nodiscard]] const Velocity& velocityInWorld() const override { return velocity_.in_world; }
    [[nodiscard]] const Posture&  postureInWorld() const override { return posture_.in_world; }

private:
    struct
    {
        Posture in_world;
    } posture_{};

    struct
    {
        Velocity in_world;
        Velocity in_body;
    } velocity_{};
};

} // namespace chassis::loc
