/**
 * @file    SteeringWheel.hpp
 * @author  syhanjin
 * @date    2026-02-28
 */
#pragma once
#include "motor_pos_controller.hpp"
#include "motor_vel_controller.hpp"
#include "gpio_driver.h"

namespace chassis::steering
{

class SteeringWheel
{
public:
    struct Velocity
    {
        float angle; ///< 舵向角度，行进正前方为零点，逆时针为正    (unit: deg)
        float speed; ///< 轮向速度，angle = 0 时向前运动的方向为正 (unit: rpm)
    };
    enum class CalibState
    {
        Idle,
        SeekGate,
        FineCapture,
        Done,
    };

    struct CalibrationConfig
    {
        controllers::MotorVelController* steer_motor            = nullptr; // 舵向电机速度环控制器
        GPIO_t                           photogate              = {};      // 光电门GPIO
        GPIO_PinState                    photogate_active_state = GPIO_PIN_RESET; // 光电门有效状态
    };

    struct Config
    {
        controllers::MotorVelController* drive_motor; // 轮向电机速度环控制器
        controllers::MotorPosController* steer_motor; // 舵向电机位置环控制器
        float steer_offset; // 舵向偏置，如果有光电门，则是光电门所在角度，如果是云台电机，则是零点所在角度
    };

    explicit SteeringWheel(const Config&            cfg,
                           bool                     enable_calib = false,
                           const CalibrationConfig& calib_cfg    = {});
    void startCalibration();
    void setTargetVelocity(const Velocity& vel);

    [[nodiscard]] float getSteerAngle() const
    {
        return toSteerAngle(cfg_.steer_motor->getMotor()->getAngle());
    }

    [[nodiscard]] float getDriveSpeed() const
    {
        return cfg_.drive_motor->getMotor()->getVelocity();
    }

    [[nodiscard]] bool enable() const
    {
        const auto steer_enabled = cfg_.steer_motor->enable();
        const auto drive_enabled = cfg_.drive_motor->enable();
        if (steer_enabled && drive_enabled)
            return true;
        if (steer_enabled && !drive_enabled)
            cfg_.steer_motor->disable();
        if (drive_enabled && !steer_enabled)
            cfg_.drive_motor->disable();
        return false;
    }

    void disable() const
    {
        cfg_.steer_motor->disable();
        cfg_.drive_motor->disable();
    }

    [[nodiscard]] bool enabled() const
    {
        return cfg_.drive_motor->enabled() && cfg_.steer_motor->enabled();
    }

    [[nodiscard]] bool isCalibrated() const
    {
        return calib_state_ == CalibState::Done;
    }

    void update() const;

private:
    Config cfg_;

    bool              enable_calib_;
    CalibrationConfig calib_cfg_;
    CalibState        calib_state_{ CalibState::Idle };

    Velocity target_vel_{}; // 目标速度

    extern "C" static void PhotogateCallback(const GPIO_t* gpio, uint32_t counter, void* data)
    {
        static_cast<SteeringWheel*>(data)->photogateTrigger();
    }

    void photogateTrigger();

    [[nodiscard]] float toMotorAngle(const float steer_angle) const
    {
        return steer_angle + cfg_.steer_offset;
    }

    [[nodiscard]] float toSteerAngle(const float motor_angle) const
    {
        return motor_angle - cfg_.steer_offset;
    }

    [[nodiscard]] Velocity toBestVelocity(Velocity velocity) const;
};

} // namespace chassis::steering