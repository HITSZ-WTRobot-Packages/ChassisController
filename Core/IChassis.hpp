/**
 * @file    IChassis.hpp
 * @author  syhanjin
 * @date    2026-01-31
 */
#pragma once

#include "mit_pd.hpp"
#include "s_curve.hpp"
#include "cmsis_os2.h"

namespace chassis
{

class IChassis
{
public:
    virtual ~IChassis() = default;

    struct Velocity
    {
        float vx; ///< 指向车体前方 (unit: m/s)
        float vy; ///< 指向车体左侧 (unit: m/s)
        float wz; ///< 向上（逆时针）为正 (unit: deg/s)
    };

    struct Posture
    {
        float x;   ///< 指向车体前方 (unit: m)
        float y;   ///< 指向车体左侧 (unit: m)
        float yaw; ///< 向上（逆时针）为正 (unit: deg)
    };

    /**
     * 车体运动反馈，用于闭环控制
     *
     * 指针指向反馈来源。为 NULL 表示没有数据来源，在必要情况下会通过运动学解算开环控制
     */
    struct Feedback
    {
        // 相对于车体，该数据一般来源于 IMU
        const float* vx = nullptr; ///< 指向车体前方 (unit: m/s)
        const float* vy = nullptr; ///< 指向车体左侧 (unit: m/s)
        const float* wz = nullptr; ///< 车体角速度 (unit: deg/s)

        // 该数据一般来源于里程计，因为是计算增量，所以没有相对一说
        const float* sx = nullptr; ///< x 方向里程计读数 (unit: m)
        const float* sy = nullptr; ///< y 方向里程计读数 (unit: m)

        // 该数据一般来源于全向定位平台（OPS），相对于 OPS World。这组数据的优先级高于 sx, sy
        // 如果要使用 OPS，世界坐标系的位置由 OPS 管理，机体不再执行换算。
        // 此时 Chassis_SetWorldFromCurrent 将不起作用
        // 当 x, y, yaw 都不为 nullptr 时，启用 OPS，请保证三个量都来自 OPS
        const float* x = nullptr; ///< x 方向位置 (unit: m)
        const float* y = nullptr; ///< y 方向位置 (unit: m)
        // 该数据一般来源于陀螺仪（或码盘），底盘不需要管相对于谁
        const float* yaw = nullptr; ///< 车体角度 (unit: deg)
    };

    struct Config
    {
        Feedback feedback_source;
    };

    [[nodiscard]] Velocity WorldVelocity2BodyVelocity(const Velocity& velocity_in_world) const;
    [[nodiscard]] Velocity BodyVelocity2WorldVelocity(const Velocity& velocity_in_body) const;
    [[nodiscard]] Posture  WorldPosture2BodyPosture(const Posture& posture_in_world) const;
    [[nodiscard]] Posture  BodyPosture2WorldPosture(const Posture& posture_in_body) const;

    void feedbackUpdate(float dt);

    [[nodiscard]] bool isOpsEnabled() const
    {
        return feedback_.x != nullptr && feedback_.y != nullptr && feedback_.yaw != nullptr;
    }

    [[nodiscard]] virtual bool enable() = 0;

    virtual void disable() = 0;

    [[nodiscard]] virtual bool enabled() const { return false; }

    [[nodiscard]] const auto& velocity() const { return velocity_; }
    [[nodiscard]] const auto& posture() const { return posture_; }

protected:
    explicit IChassis(const Config& cfg);

    enum class WheeledKinematicsType
    {
        /**
         * nonholonomic rolling-without-slipping constraint
         * 直接用轮子位移就能计算出总位移的底盘类型，如 Mecanum4, Omni4
         */
        RollingConstrained,
        /**
         * pose via time integration of velocity
         * 只能由速度量积分得到总位移的底盘类型，如 Steering4
         */
        VelocityIntegrated
    };

    [[nodiscard]] virtual WheeledKinematicsType kinematicsType() const = 0;

    virtual void applyVelocity(const Velocity& velocity) = 0;

    virtual void velocityControllerUpdate() = 0;

    virtual float forwardGetX()   = 0;
    virtual float forwardGetY()   = 0;
    virtual float forwardGetYaw() = 0;
    virtual float forwardGetVx()  = 0;
    virtual float forwardGetVy()  = 0;
    virtual float forwardGetWz()  = 0;

    void applySetWorldFromCurrent();

private:
    struct
    {
        Velocity in_world;
        Velocity in_body;
    } velocity_{};

    struct
    {
        /**
         * feedback_yaw - world_yaw = body_yaw
         */
        Posture in_world{}; ///< 车身位置（世界坐标系与车身坐标系的变换关系）、
    } posture_;

    struct
    {
        Posture posture; ///< 世界坐标系位置（相对于反馈）
    } world_{};

    Feedback feedback_;

    struct
    {
        float vx;
        float vy;
        float wz;
        float sx;
        float sy;
        float yaw;
    } last_feedback_{};
};

} // namespace chassis
