/**
 * @file    Mecanum4.cpp
 * @author  syhanjin
 * @date    2026-01-31
 */
#include "Mecanum4.hpp"

namespace chassis
{
static constexpr size_t idx(Mecanum4::WheelType w)
{
    return static_cast<size_t>(w);
}

Mecanum4::Mecanum4(const Config& driver_cfg, const IChassis::Config& base_cfg) :
    IChassis(base_cfg), type_(driver_cfg.chassis_type),
    wheel_radius_(driver_cfg.wheel_radius * 1e-3f)
{
    if (type_ == ChassisType::OType)
        k_omega_ = driver_cfg.wheel_distance_x * 1e-3f * 0.5f +
                   driver_cfg.wheel_distance_y * 1e-3f * 0.5f;
    else if (type_ == ChassisType::XType)
        k_omega_ = driver_cfg.wheel_distance_x * 1e-3f * 0.5f -
                   driver_cfg.wheel_distance_y * 1e-3f * 0.5f;

    wheel_[idx(WheelType::FrontRight)] = driver_cfg.wheel_front_right;
    wheel_[idx(WheelType::FrontLeft)]  = driver_cfg.wheel_front_left;
    wheel_[idx(WheelType::RearLeft)]   = driver_cfg.wheel_rear_left;
    wheel_[idx(WheelType::RearRight)]  = driver_cfg.wheel_rear_right;
}
bool Mecanum4::enable()
{
    bool enabled = true;

    for (const auto& w : wheel_)
        enabled &= w->enable();

    if (!enabled)
    {
        for (const auto& w : wheel_)
            w->disable();
    }
    else
    {
        // 进行一次角度归零，用于计算正解
        for (const auto& w : wheel_)
            w->getMotor()->resetAngle();
    }
    enabled_ = enabled;
    return enabled;
}
void Mecanum4::disable()
{
    for (const auto& w : wheel_)
        w->disable();
    enabled_ = false;
}

/**
 * 设置底盘速度
 */
void Mecanum4::applyVelocity(const Velocity& velocity)
{
    const auto& [vx, vy, wz] = velocity;
    if (type_ == ChassisType::OType)
    {
        /** Mecanum4 O 型运动学解算
         * w_fr = (+ vx + vy + (w + h) * ω) / r
         * w_fl = (+ vx - vy - (w + h) * ω) / r
         * w_rl = (+ vx + vy - (w + h) * ω) / r
         * w_rr = (+ vx - vy + (w + h) * ω) / r
         */
        wheel_[idx(WheelType::FrontRight)]->setRef(
                RPS2RPM((vx + vy + k_omega_ * DEG2RAD(wz)) / wheel_radius_));
        wheel_[idx(WheelType::FrontLeft)]->setRef(
                RPS2RPM((vx - vy - k_omega_ * DEG2RAD(wz)) / wheel_radius_));
        wheel_[idx(WheelType::RearLeft)]->setRef(
                RPS2RPM((vx + vy - k_omega_ * DEG2RAD(wz)) / wheel_radius_));
        wheel_[idx(WheelType::RearRight)]->setRef(
                RPS2RPM((vx - vy + k_omega_ * DEG2RAD(wz)) / wheel_radius_));
    }
    else if (type_ == ChassisType::XType)
    {
        /** Mecanum4 X 型运动学解算
         * w_fr = (+ vx - vy + (w - h) * ω) / r
         * w_fl = (+ vx + vy - (w - h) * ω) / r
         * w_rl = (+ vx - vy - (w - h) * ω) / r
         * w_rr = (+ vx + vy + (w - h) * ω) / r
         */
        wheel_[idx(WheelType::FrontRight)]->setRef(
                RPS2RPM((vx - vy + k_omega_ * DEG2RAD(wz)) / wheel_radius_));
        wheel_[idx(WheelType::FrontLeft)]->setRef(
                RPS2RPM((vx + vy - k_omega_ * DEG2RAD(wz)) / wheel_radius_));
        wheel_[idx(WheelType::RearLeft)]->setRef(
                RPS2RPM((vx - vy - k_omega_ * DEG2RAD(wz)) / wheel_radius_));
        wheel_[idx(WheelType::RearRight)]->setRef(
                RPS2RPM((vx + vy + k_omega_ * DEG2RAD(wz)) / wheel_radius_));
    }
}

/**
 * 底盘控制更新函数
 *
 * 本函数自动处理控制逻辑，并依序调用每个轮子的 PID 更新函数
 *
 * @note 推荐控制调用频率 1kHz，调用频率将会影响轮子的 PID 参数
 */
void Mecanum4::velocityControllerUpdate()
{
    if (!enabled())
        return;
    for (const auto& wheel : wheel_)
        wheel->update();
}

float Mecanum4::forwardGetYaw()
{
    if (type_ == ChassisType::OType)
        return wheel_radius_ / (4 * k_omega_) *
               (wheel_[idx(WheelType::FrontRight)]->getMotor()->getAngle() -
                wheel_[idx(WheelType::FrontLeft)]->getMotor()->getAngle() +
                wheel_[idx(WheelType::RearRight)]->getMotor()->getAngle() -
                wheel_[idx(WheelType::RearLeft)]->getMotor()->getAngle());

    if (type_ == ChassisType::XType)
        return wheel_radius_ / (4 * k_omega_) *
               (wheel_[idx(WheelType::FrontRight)]->getMotor()->getAngle() -
                wheel_[idx(WheelType::FrontLeft)]->getMotor()->getAngle() -
                wheel_[idx(WheelType::RearRight)]->getMotor()->getAngle() +
                wheel_[idx(WheelType::RearLeft)]->getMotor()->getAngle());

    return 0.0f;
}

float Mecanum4::forwardGetX()
{
    return wheel_radius_ * 0.25f *
           DEG2RAD(wheel_[idx(WheelType::FrontRight)]->getMotor()->getAngle() +
                   wheel_[idx(WheelType::FrontLeft)]->getMotor()->getAngle() +
                   wheel_[idx(WheelType::RearRight)]->getMotor()->getAngle() +
                   wheel_[idx(WheelType::RearLeft)]->getMotor()->getAngle());
}

float Mecanum4::forwardGetY()
{
    if (type_ == ChassisType::OType)
        return wheel_radius_ * 0.25f *
               DEG2RAD(wheel_[idx(WheelType::FrontRight)]->getMotor()->getAngle() -
                       wheel_[idx(WheelType::FrontLeft)]->getMotor()->getAngle() -
                       wheel_[idx(WheelType::RearRight)]->getMotor()->getAngle() +
                       wheel_[idx(WheelType::RearLeft)]->getMotor()->getAngle());

    if (type_ == ChassisType::XType)
        return wheel_radius_ / (4 * k_omega_) *
               DEG2RAD(-wheel_[idx(WheelType::FrontRight)]->getMotor()->getAngle() +
                       wheel_[idx(WheelType::FrontLeft)]->getMotor()->getAngle() +
                       wheel_[idx(WheelType::RearRight)]->getMotor()->getAngle() -
                       wheel_[idx(WheelType::RearLeft)]->getMotor()->getAngle());

    return 0.0f;
}

float Mecanum4::forwardGetWz()
{
    if (type_ == ChassisType::OType)
        return RPM2DPS(wheel_radius_ / (4 * k_omega_) *
                       (wheel_[idx(WheelType::FrontRight)]->getMotor()->getVelocity() -
                        wheel_[idx(WheelType::FrontLeft)]->getMotor()->getVelocity() +
                        wheel_[idx(WheelType::RearRight)]->getMotor()->getVelocity() -
                        wheel_[idx(WheelType::RearLeft)]->getMotor()->getVelocity()));

    if (type_ == ChassisType::XType)
        return RPM2DPS(wheel_radius_ / (4 * k_omega_) *
                       (wheel_[idx(WheelType::FrontRight)]->getMotor()->getVelocity() -
                        wheel_[idx(WheelType::FrontLeft)]->getMotor()->getVelocity() -
                        wheel_[idx(WheelType::RearRight)]->getMotor()->getVelocity() +
                        wheel_[idx(WheelType::RearLeft)]->getMotor()->getVelocity()));

    return 0.0f;
}

float Mecanum4::forwardGetVx()
{
    return RPM2DPS(wheel_radius_ * 0.25f *
                   DEG2RAD(wheel_[idx(WheelType::FrontRight)]->getMotor()->getVelocity() +
                           wheel_[idx(WheelType::FrontLeft)]->getMotor()->getVelocity() +
                           wheel_[idx(WheelType::RearRight)]->getMotor()->getVelocity() +
                           wheel_[idx(WheelType::RearLeft)]->getMotor()->getVelocity()));
}

float Mecanum4::forwardGetVy()
{
    if (type_ == ChassisType::OType)
        return RPM2DPS(wheel_radius_ * 0.25f *
                       DEG2RAD(wheel_[idx(WheelType::FrontRight)]->getMotor()->getVelocity() -
                               wheel_[idx(WheelType::FrontLeft)]->getMotor()->getVelocity() -
                               wheel_[idx(WheelType::RearRight)]->getMotor()->getVelocity() +
                               wheel_[idx(WheelType::RearLeft)]->getMotor()->getVelocity()));

    if (type_ == ChassisType::XType)
        return RPM2DPS(wheel_radius_ / (4 * k_omega_) *
                       DEG2RAD(-wheel_[idx(WheelType::FrontRight)]->getMotor()->getVelocity() +
                               wheel_[idx(WheelType::FrontLeft)]->getMotor()->getVelocity() +
                               wheel_[idx(WheelType::RearRight)]->getMotor()->getVelocity() -
                               wheel_[idx(WheelType::RearLeft)]->getMotor()->getVelocity()));

    return 0.0f;
}

} // namespace chassis