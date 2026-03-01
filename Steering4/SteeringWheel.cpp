/**
 * @file    SteeringWheel.cpp
 * @author  syhanjin
 * @date    2026-02-28
 */
#include "SteeringWheel.hpp"

namespace chassis::steering
{
SteeringWheel::SteeringWheel(const Config&            cfg,
                             const bool               enable_calib,
                             const CalibrationConfig& calib_cfg) :
    cfg_(cfg), enable_calib_(enable_calib), calib_cfg_(calib_cfg)
{
    assert(cfg_.drive_motor != nullptr);
    assert(cfg_.steer_motor != nullptr);
    assert(!enable_calib_ || calib_cfg_.steer_motor != nullptr);
}

/**
 * 舵轮校准，只允许被调用一次
 */
void SteeringWheel::startCalibration()
{
    if (!enable_calib_ || !enabled() || calib_state_ != CalibState::Idle)
        return;
    // disable steer pos and drive vel
    cfg_.drive_motor->disable();
    cfg_.steer_motor->disable();
    // register photogate callback
    GPIO_EXTI_RegisterCallback(&calib_cfg_.photogate, PhotogateCallback, this);
    if (GPIO_ReadPin(&calib_cfg_.photogate) == calib_cfg_.photogate_active_state)
    {
        // 开始就在光电门内，低速正转
        calib_state_ = CalibState::FineCapture;
        calib_cfg_.steer_motor->setRef(5);
    }
    else
    {
        // 开始不在光电门内，高速正转寻找光电门
        calib_state_ = CalibState::SeekGate;
        calib_cfg_.steer_motor->setRef(30);
    }
    calib_cfg_.steer_motor->enable();
}

void SteeringWheel::setTargetVelocity(const Velocity& vel)
{
    target_vel_ = toBestVelocity(vel);
    cfg_.steer_motor->setRef(toMotorAngle(target_vel_.angle));
    cfg_.drive_motor->setRef(vel.speed);
}
void SteeringWheel::update() const
{
    cfg_.steer_motor->update();
    cfg_.drive_motor->update();
    if (enable_calib_)
        calib_cfg_.steer_motor->update();
}

void SteeringWheel::photogateTrigger()
{
    switch (calib_state_)
    {
    case CalibState::SeekGate: // 第一次触发，降低速度
        calib_cfg_.steer_motor->setRef(5);
        calib_state_ = CalibState::FineCapture;
        break;
    case CalibState::FineCapture: // 第二次触发，记录角度并锁定
        calib_cfg_.steer_motor->disable();
        cfg_.steer_motor->getMotor()->resetAngle();
        cfg_.steer_motor->setRef(toMotorAngle(0));
        // 使能轮向速度环和舵向位置环
        cfg_.steer_motor->enable();
        cfg_.drive_motor->enable();
        calib_state_ = CalibState::Done;
        break;
    default:;
    }
}

SteeringWheel::Velocity SteeringWheel::toBestVelocity(Velocity velocity) const
{
    uint32_t round         = 0;                 // 当前角度对应的圈数（整圈计数）
    float    current_angle = target_vel_.angle; // 当前角度（可能大于360°或小于0°）

    /* 角度归一化，将当前角度调整到 [0, 360) 范围内，同时记录整圈数量 */
    while (current_angle > 360.0f)
        current_angle -= 360.0f, round++;
    while (current_angle < 0.0f)
        current_angle += 360.0f, round--;

    /* 将目标角度也归一化到 [0, 360) */
    while (velocity.angle > 360.0f)
        velocity.angle -= 360.0f;
    while (velocity.angle < 0.0f)
        velocity.angle += 360.0f;

    /* 计算目标角度相对于当前角度的差值 */
    const float delta = velocity.angle - current_angle;

    /*
     * 角度差分区间说明：
     * -360° ≤ delta < -270° : 当前角度比目标角度多约一圈 → 加一圈
     * -270° ≤ delta < -90°  : 反向驱动更短（加180°并反向速度）
     * -90° < delta ≤ 90°    : 最短路径，不需调整
     *  90° < delta ≤ 270°   : 反向驱动更短（减180°并反向速度）
     * 270° < delta ≤ 360°   : 当前角度比目标角度少约一圈 → 减一圈
     */
    if (-360.0f <= delta && delta < -270.0f)
    {
        velocity.angle += 360.0f;
    }
    else if (-270.0f <= delta && delta < -90.0f)
    {
        velocity.angle += 180.0f;
        velocity.speed = -velocity.speed;
    }
    // else if (-90.0f < delta && delta <= 90.0f) // do nothing
    else if (90.0f < delta && delta <= 270.0f)
    {
        velocity.angle -= 180.0f;
        velocity.speed = -velocity.speed;
    }
    else if (270.0f < delta && delta <= 360.0f)
    {
        velocity.angle -= 360.0f;
    }
    velocity.angle += static_cast<float>(round) * 360.0f;
    return velocity;
}
} // namespace chassis::steering