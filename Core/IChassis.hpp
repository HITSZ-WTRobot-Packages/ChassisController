/**
 * @file    IChassis.hpp
 * @author  syhanjin
 * @date    2026-01-31
 */
#pragma once

#include "mit_pd.hpp"
#include "s_curve.hpp"
#include "cmsis_os2.h"

#define DEG2RAD(__DEG__) ((__DEG__) * (float) 3.14159265358979323846f / 180.0f)

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

    using AxisLimit = velocity_profile::SCurveProfile::Config;

    enum class CtrlMode
    {
        Velocity,
        Posture,
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
        struct
        {
            MITPD::Config vx; ///< x 速度 PD 控制器
            MITPD::Config vy; ///< y 速度 PD 控制器
            MITPD::Config wz; ///< 角速度 PD 控制器
        } posture_error_pd_cfg;

        Feedback feedback_source;

        struct
        {
            AxisLimit x, y, yaw;
        } limit{};
    };

    [[nodiscard]] Velocity WorldVelocity2BodyVelocity(const Velocity& velocity_in_world) const;
    [[nodiscard]] Velocity BodyVelocity2WorldVelocity(const Velocity& velocity_in_body) const;
    [[nodiscard]] Posture  WorldPosture2BodyPosture(const Posture& posture_in_world) const;
    [[nodiscard]] Posture  BodyPosture2WorldPosture(const Posture& posture_in_body) const;

    void feedbackUpdate();
    void profileUpdate(float dt);
    void errorUpdate();
    void controllerUpdate();

    [[nodiscard]] bool isOpsEnabled() const
    {
        return feedback_.x != nullptr && feedback_.y != nullptr && feedback_.yaw != nullptr;
    }

    bool setTargetPostureInWorld(const Posture& absolute_target);
    bool setTargetPostureInBody(const Posture& relative_target);

    [[nodiscard]] bool isTrajectoryFinished() const;
    void               waitTrajectoryFinish() const;

    void setVelocityInWorld(const Velocity& world_velocity, bool target_in_world);
    void setVelBodyFrame(const Velocity& body_velocity, bool target_in_world);

    void setWorldFromCurrent();

    virtual bool enable()  = 0;
    virtual void disable() = 0;

    [[nodiscard]] virtual bool enabled() const
    {
        return false;
    }

protected:
    explicit IChassis(const Config& cfg);

    virtual void applyVelocity(const Velocity& velocity) = 0;

    virtual void velocityControllerUpdate() = 0;

    virtual float forwardGetX()   = 0;
    virtual float forwardGetY()   = 0;
    virtual float forwardGetYaw() = 0;
    virtual float forwardGetVx()  = 0;
    virtual float forwardGetVy()  = 0;
    virtual float forwardGetWz()  = 0;

private:
    osMutexId_t lock_;

    CtrlMode ctrl_mode_{ CtrlMode::Velocity }; ///< 当前控制模式

    AxisLimit limit_x_;
    AxisLimit limit_y_;
    AxisLimit limit_yaw_;

    struct
    {
        volatile bool target_in_world; ///< 速度是否相对于世界坐标系不变
        Velocity      in_world;        ///< 世界坐标系下速度
        Velocity      in_body;         ///< 车体坐标系下速度

        struct
        {
            Velocity in_world;
            Velocity in_body;
        } feedback; ///< 计算的车体速度反馈
    } velocity_{};

    struct
    {
        /**
         * feedback_yaw - world_yaw = body_yaw
         */
        Posture in_world{}; ///< 车身位置（世界坐标系与车身坐标系的变换关系）

        struct
        {
            float now{};        ///< 当前执行时间
            float total_time{}; ///< 总执行时间

            struct
            {
                MITPD vx; ///< x 速度 PD 控制器
                MITPD vy; ///< y 速度 PD 控制器
                MITPD wz; ///< 角速度 PD 控制器
            } pd;

            struct
            {
                velocity_profile::SCurveProfile x;
                velocity_profile::SCurveProfile y;
                velocity_profile::SCurveProfile yaw;
            } curve;

            Posture  p_ref_curr_{};
            Velocity v_ref_curr_{};
        } trajectory;
    } posture_;

    struct
    {
        volatile Posture posture; ///< 世界坐标系位置（相对于反馈）
    } world_{};

    Feedback feedback_;

    struct
    {
        float sx;
        float sy;
        float yaw;
    } last_feedback_{};

private:
    void update_posture();
    void update_velocity_control();
    void update_velocity_feedback();
    void apply_position_velocity();
};

} // namespace chassis
