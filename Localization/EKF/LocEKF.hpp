/**
 * @file    LocEKF.hpp
 * @author  syhanjin
 * @date    2026-03-11
 * @brief   ESKF-based Localization System Implementation
 *
 * 使用 fastlio2(10Hz) + encoder(1kHz) + HWT101CT(yaw, 1kHz) 融合定位
 */
#pragma once
#include "HWT101CT.hpp"
#include "ILoc.hpp"
#include "Vec.hpp"
#include "EKF.hpp"
#include "Mat.hpp"

#ifndef DEG2RAD
#    define DEG2RAD(__DEG__) ((__DEG__) * (float)3.14159265358979323846f / 180.0f)
#endif

namespace chassis_loc
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
class LocEKF : public ILoc
{
public:
    // sensors::gyro::HWT101CT& gyro;

    // TODO: 通过雷达反馈时间做回溯和重积分
    class PositionEKF : math::ekf::EKF<float, 4>
    {
        // state: [x, y, yaw, yaw_offset]
        // x, y: m.
        // yaw, yaw_offset: deg
        // wz: deg/s
    public:
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

        explicit PositionEKF(const Config& cfg) :
            EKF(cfg.x_init.vec(), cfg.covQ.mat(), cfg.noiseQ.mat()), R_gyro_(cfg.noiseR.gyro.mat()),
            R_lidar_(cfg.noiseR.lidar.mat())
        {
        }

        [[nodiscard]] auto state() const { return x; }

        void odomUpdate(const Velocity& vel, const float dt)
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

        void gyroUpdate(const float& yaw, const float dt)
        {
            // gyro 观测 yaw
            using Vec1 = math::Vec<float, 1>;
            constexpr math::Mat<float, 1, 4> H{ { { 0, 0, 1, 0 } } };
            update(Vec1{ yaw - x[2] }, H, R_gyro_);
        }

        void lidarUpdate(const Posture& pos, const float dt)
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

    private:
        math::Mat<float, 1, 1> R_gyro_;
        math::Mat<float, 3, 3> R_lidar_;
    };
    PositionEKF pos_ekf_;
};

} // namespace chassis_loc
