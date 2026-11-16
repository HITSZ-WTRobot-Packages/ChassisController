/**
 * @file    Master.hpp
 * @author  syhanjin
 * @date    2026-02-24
 * @brief   下位机主控模式的底盘控制器。
 */
#pragma once
#include "IChassisController.hpp"
#include "isr_lock.h"
#include "s_curve.hpp"
#include "pid_pd.hpp"
#include "cmsis_os2.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace chassis::controller
{

/**
 * 下位机主控模式。
 *
 * 这个控制器自己负责：
 * - 速度模式下的参考保持
 * - 位姿模式下的 S 曲线规划
 * - 基于定位反馈的误差闭环
 *
 * 公开接口里有 7 种常用目标设置组合：
 * - 位姿目标 3 种：绝对目标、相对自身目标、相对指定基准点目标
 * - 速度目标 4 种：输入坐标系（世界 / 车身） × 保持不变的参考系（世界 / 车身）
 *
 * 适合“上层给目标，底盘自己走过去”的使用方式。
 */
class Master : public IChassisController
{
public:
    using AxisLimit         = velocity_profile::SCurveProfile::Config;
    using SCurveFailureInfo = velocity_profile::SCurveProfile::FailureInfo;

    /// 三个自由度各自的运动学约束。
    struct TrajectoryLimit
    {
        AxisLimit x, y, yaw;

        constexpr TrajectoryLimit operator*(const float ratio) const
        {
            return { x * ratio, y * ratio, yaw * ratio };
        }
        constexpr TrajectoryLimit operator/(const float factor) const
        {
            return { x / factor, y / factor, yaw / factor };
        }
    };

    /// 位姿轨迹跟踪 / 完成判定阈值。
    struct TrajectoryTrackingThreshold
    {
        float x{ 0.01f };  ///< x 方向允许误差 (unit: m)
        float y{ 0.01f };  ///< y 方向允许误差 (unit: m)
        float yaw{ 0.5f }; ///< yaw 方向允许误差 (unit: deg)
    };

    /// 控制器配置：误差 PD 参数 + 轨迹曲线限制 + 完成判定阈值。
    struct Config
    {
        struct
        {
            PD::Config vx; ///< x 速度 PD 控制器
            PD::Config vy; ///< y 速度 PD 控制器
            PD::Config wz; ///< 角速度 PD 控制器
        } posture_error_pd_cfg;

        TrajectoryLimit limit{};

        TrajectoryTrackingThreshold tracking_threshold{};

        /// 设置位姿目标时，是否把目标 yaw 转换为相对本次规划起点最近的等效连续角。
        bool auto_nearest_yaw_target{ true };
    };

    enum class CtrlMode
    {
        Stopped,  ///< 锁定当前位姿
        Velocity, ///< 直接按速度参考控制
        Posture,  ///< 先规划位姿曲线，再跟踪曲线
    };

    /**
     * 设置位置目标时曲线的衔接方式
     */
    enum class TrajectoryLinkMode
    {
        CurrentState,  // 使用当前估计的位姿 / 速度，加速度置零
        PreviousCurve, // 使用上一条轨迹在 now 时刻的位置 / 速度 / 加速度
    };

    struct SCurveFailureRecord
    {
        bool    valid{ false };
        uint8_t axis_mask{ 0 };

        std::array<SCurveFailureInfo, 3> axis{};
    };

    static constexpr auto defaultTrajectoryLinkMode = TrajectoryLinkMode::PreviousCurve;

    /// 构造时立即建立三个轴各自的 PD 与 S 曲线对象。
    Master(motion::IChassisMotion& motion, loc::IChassisLoc& loc, const Config& cfg);

    /**
     * 在世界坐标系中设置绝对目标位姿。
     *
     * 这是最直接的“去某个绝对点”接口，适合地图中已经有明确目标位姿的场景。
     *
     * @param absolute_target 绝对目标值
     * @param link_mode 曲线衔接模式，如果上一控制状态不为 Posture，则该项不生效
     * @param limit 执行过程限制
     * @return 是否规划成功
     */
    bool setTargetPostureInWorld(const Posture&         absolute_target,
                                 TrajectoryLinkMode     link_mode,
                                 const TrajectoryLimit& limit);

    // 下面这组同名函数是 C++ 的“函数重载”。
    // 对不熟悉 C++ 的同学来说，可以简单理解成：函数名相同，但参数个数或类型不同，
    // 编译器会根据你传入的实参，自动挑选最合适的那个版本。
    // 这里这样做的目的，是让调用者在常见场景下少写一些样板参数；真正的规划逻辑
    // 仍然集中在上面这个“参数最全”的实现里，避免复制多份实现后改漏。
    bool setTargetPostureInWorld(const Posture& absolute_target);
    bool setTargetPostureInWorld(const Posture& absolute_target, TrajectoryLinkMode link_mode);
    bool setTargetPostureInWorld(const Posture& absolute_target, const TrajectoryLimit& limit);

    /**
     * 以“当前车体位姿”为基准，设置相对目标位姿。
     *
     * 适合简单动作命令，例如“向前一段距离”或“相对当前朝向转一个角度”。
     *
     * @param relative_target 相对目标值
     * @param link_mode 曲线衔接模式，如果上一控制状态不为 Posture，则该项不生效
     * @param limit 执行过程限制
     * @return 是否规划成功
     */
    bool setTargetPostureInBody(const Posture&         relative_target,
                                TrajectoryLinkMode     link_mode,
                                const TrajectoryLimit& limit);

    // 这组重载的作用和上面一样：让“相对自身”的常见调用可以只关心核心目标值，
    // 而把 link_mode / limit 交给默认配置处理；如果确实需要细调，再调用参数更全的版本。
    bool setTargetPostureInBody(const Posture& relative_target);
    bool setTargetPostureInBody(const Posture& relative_target, TrajectoryLinkMode link_mode);
    bool setTargetPostureInBody(const Posture& relative_target, const TrajectoryLimit& limit);

    bool setTargetPostureRelativeTo(const Posture&         base_in_world,
                                    const Posture&         relative_target,
                                    TrajectoryLinkMode     link_mode,
                                    const TrajectoryLimit& limit);

    // 这一组同样是重载，只是这里多了一个 base_in_world 基准点参数。
    // 保留同名接口的好处是：上层一看到函数名，就知道它们都属于“设置位姿目标”这类操作；
    // 真正的区别只在“目标相对于谁”以及“是否显式传入 link_mode / limit”。
    bool setTargetPostureRelativeTo(const Posture& base_in_world, const Posture& relative_target);

    bool setTargetPostureRelativeTo(const Posture&     base_in_world,
                                    const Posture&     relative_target,
                                    TrajectoryLinkMode link_mode);

    bool setTargetPostureRelativeTo(const Posture&         base_in_world,
                                    const Posture&         relative_target,
                                    const TrajectoryLimit& limit);

    [[nodiscard]] bool isTrajectoryFinished() const;

    /// 当前位姿是否跟随在曲线当前目标的阈值范围内。
    [[nodiscard]] bool isTrajectoryTrackingWithinThreshold() const;

    [[nodiscard]] CtrlMode controlMode() const { return ctrl_mode_; }

    [[nodiscard]] const SCurveFailureRecord& lastSCurveFailure() const
    {
        return last_scurve_failure_;
    }

    /// 阻塞等待当前位姿轨迹执行完成。常用于流程式脚本控制。
    void waitTrajectoryFinish() const
    {
        while (!isTrajectoryFinished())
            osDelay(1);
    }

    /**
     * 设置世界坐标系速度参考。
     *
     * 这个接口只回答“输入速度是按世界系表达的”。
     * 至于后续控制中究竟保持世界系速度不变，还是只在设置时换算一次后改成车体系保持，
     * 则由 target_in_world 决定。
     *
     * @param world_velocity 目标世界系速度
     * @param target_in_world true 表示后续控制周期中都保持“世界系速度不变”，并在每次执行时
     *                        重新换算到车体系；平移和旋转并存时通常表现为边平移边自旋
     *                        false 表示只在设置时做一次换算，后续按车体系速度保持；
     *                        此时常见表现是圆弧轨迹
     */
    void setVelocityInWorld(const Velocity& world_velocity, bool target_in_world);

    /**
     * 设置车体坐标系速度参考。
     *
     * 这个接口只回答“输入速度是按车体系表达的”。
     * 至于后续控制中究竟保持车体系速度不变，还是先把当前车体系速度锁存成世界系速度，
     * 则由 target_in_world 决定。
     *
     * @param body_velocity 目标车体系速度
     * @param target_in_world true 表示先换算并保存对应的世界系速度，之后按“世界系速度保持不变”
     *                        的方式运行；平移和旋转并存时通常表现为边平移边自旋
     *                        false 表示保持普通车体系速度控制；此时常见表现是圆弧轨迹
     */
    void setVelocityInBody(const Velocity& body_velocity, bool target_in_world);

    void stop() override;

    /**
     * 设置底盘速度为 0
     */
    void hold() override;

    /**
     * 更新底盘轨迹规划曲线
     *
     * 仅在 CtrlMode::Posture 下有效
     * @param dt 更新间隔
     * @note 推荐 100Hz
     */
    void profileUpdate(float dt);

    /**
     * 更新底盘轨迹 PD 控制器
     * @note 推荐 200 ~ 500Hz
     */
    void errorUpdate();

    void controllerUpdate();

private:
    osMutexId_t lock_; ///< 保护配置切换和目标写入过程

    CtrlMode ctrl_mode_{ CtrlMode::Stopped }; ///< 当前控制模式

    TrajectoryLimit limit_; ///< 默认轨迹约束

    TrajectoryTrackingThreshold tracking_threshold_; ///< 轨迹跟踪 / 完成判定阈值

    bool auto_nearest_yaw_target_{ true }; ///< 设置目标时是否自动选择最近等效 yaw

    SCurveFailureRecord last_scurve_failure_{};

    struct
    {
        volatile bool target_in_world; ///< 速度是否相对于世界坐标系不变
        Velocity      in_world;        ///< 世界坐标系下速度
        Velocity      in_body;         ///< 车体坐标系下速度
    } velocity_ref_{};

    struct
    {
        float now{};        ///< 当前执行时间
        float total_time{}; ///< 总执行时间

        struct
        {
            PD vx; ///< x 速度 PD 控制器
            PD vy; ///< y 速度 PD 控制器
            PD wz; ///< 角速度 PD 控制器
        } pd;

        struct
        {
            velocity_profile::SCurveProfile x;
            velocity_profile::SCurveProfile y;
            velocity_profile::SCurveProfile yaw;
        } curve;

        Posture  p_ref_curr_{}; ///< 当前时刻的位姿参考点
        Velocity v_ref_curr_{}; ///< 当前时刻的前馈速度参考
    } posture_trajectory_;

private:
    [[nodiscard]] static float nearestEquivalentYaw(float reference_yaw, float target_yaw);

    void update_velocity_control();

    void apply_position_velocity();
};

} // namespace chassis::controller
