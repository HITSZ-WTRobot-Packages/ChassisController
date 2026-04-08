/**
 * @file    SteeringWheel.cpp
 * @author  syhanjin
 * @date    2026-02-28
 * @brief   单个舵轮模块的控制与校准逻辑。
 */
#include "SteeringWheel.hpp"

namespace chassis::steering
{
// 禁用校准时使用的默认占位配置，避免每次都构造一份临时对象。
constexpr SteeringWheel::CalibrationConfig kDefaultCalibration{};

SteeringWheel::SteeringWheel(const Config&            cfg,
                             const bool               enable_calib = false,
                             const CalibrationConfig& calib_cfg    = kDefaultCalibration) :
    cfg_(cfg), enable_calib_(enable_calib), calib_cfg_(calib_cfg)
{
    // 这里用 assert 明确表达构造期前提，避免运行期出现空指针后难以排查。
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
    // 当前实现默认 photogate 的 EXTI 已经配置成双边沿触发，这样进入挡光区和离开挡光区
    // 都会回调到这里，才能让“第一次发现门、第二次固定边缘”的状态机成立。
    // TODO: 在此检测 EXTI 边沿触发是否被正常配置
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
    // 先做“最短路径”优化，再分别下发到舵向位置环和轮向速度环。
    target_vel_ = toBestVelocity(vel);
    cfg_.steer_motor->setRef(toMotorAngle(target_vel_.angle));
    cfg_.drive_motor->setRef(target_vel_.speed);
}
void SteeringWheel::update() const
{
    // 正常运行与校准使用的是不同控制器，因此校准开启时需要都更新。
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
        // 第一次命中只说明已经找到门，需要减速再来一次，降低零点误差。
        calib_cfg_.steer_motor->setRef(5);
        calib_state_ = CalibState::FineCapture;
        break;
    case CalibState::FineCapture: // 第二次触发，记录角度并锁定
        calib_cfg_.steer_motor->disable();
        // 将当前位置记为机械零点，再把正常控制器切回“舵向零度”。
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
    int32_t round         = 0;                 // 当前角度对应的圈数（整圈计数）
    float   current_angle = target_vel_.angle; // 当前角度（可能大于360°或小于0°）

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
