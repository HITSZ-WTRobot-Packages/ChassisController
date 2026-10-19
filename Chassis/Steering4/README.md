# Steering4

`Steering4` 是四舵轮底盘的 `Motion` 实现。它和 `Mecanum4` / `Omni4` 的差别不在于对外接口，而在于“底盘能否立即进入可控状态”这件事上存在明显的启动阶段。

包标识为 `Chassis::Steering4`，依赖见 [cpkg.toml](cpkg.toml)：`Chassis::Core`、`FreeRTOS`、`bsp::GPIO_Driver`。通用分层、生命周期和控制权见[仓库综述](../../README.md)。本包需要底层轮向速度控制器和舵向位置控制器；启用校准时还需要舵向速度控制器及 GPIO EXTI，均由宿主提供。

## 适用场景

- 四个轮组都同时具备驱动轴和舵向轴。
- 上层希望仍然把整个底盘当作统一 `Motion` 使用，而不是自己分别控制四个舵轮。
- 启动时可能需要先完成舵向零点校准，再允许控制器接管。

## 对外仍然暴露的统一语义

`Steering4` 仍然遵守 `IChassisMotion` 语义：

- `enable()` / `disable()`：使能或失能四个轮组。
- `forwardGetVelocity()`：返回当前估算到的车体速度。
- `isReady()`：在未启用校准时恒为 `true`；启用校准时只有全部轮组校准完成后才为 `true`。

这也是为什么上层应依赖 `isReady()`，而不是假设“底盘一旦 enable 就一定可控”。

## 配置结构

`Steering4::Config` 分成两部分：

- 底盘公共几何参数：`radius`、`distance_x`、`distance_y`
- 每个轮组自己的 `Wheel` 配置：`SteeringWheel::Config` 和 `SteeringWheel::CalibrationConfig`

几何参数单位如下：

- `radius`：驱动轮半径，单位 `mm`
- `distance_x`：前后轮距，单位 `mm`
- `distance_y`：左右轮距，单位 `mm`

每个轮组都包含两类底层对象：

- `drive_motor`：轮向速度控制器
- `steer_motor`：舵向位置控制器

这些控制器必须有效，且与光电门资源一起覆盖 Motion 的使用生命周期。正常运行的舵向位置环与校准速度环应对应同一个舵向执行器，不能把两者接成不同电机。

如果启用校准，还要提供：

- `calib_cfg.steer_motor`：校准阶段用于慢速寻零的舵向速度控制器
- `calib_cfg.photogate`：光电门 GPIO
- `calib_cfg.photogate_active_state`：光电门有效电平

启用光电门校准时，宿主必须在 CubeMX 中把对应 GPIO / EXTI 配成 `Rising/Falling` 双边沿触发，并接通 GPIO 驱动的中断回调链路。库的运行期职责是注册回调、读取电平和推进校准，不重新初始化、修改或自动修复宿主的静态 EXTI 配置。当前实现尚未校验双边沿配置，不能把它当作已完成校验；若所用 HAL1 无法校验此约束，必须先通过 Ask 向用户确认处理方案，不得自行绕过 HAL1 或增加寄存器配置方案。

## 启动与校准流程

推荐接入顺序：

1. 构造 `Steering4`
2. 调用 `enable()`
3. 如果 `enable_calibration == true`，调用 `startCalibration()`
4. 周期调用 `update()`
5. 等待 `isReady()` 返回 `true`
6. 再交给 `Controller` 正常发速度或位姿目标

校准开始时若光电门已有效，直接低速精捕；否则先寻门，首次边缘后降低速度，再以精捕边缘建立零点。只有全部轮组完成后底盘才 ready。双边沿事件缺失可能使校准停在中间状态或零点边缘不符合预期，不能以超时后继续正常控制替代校准完成。具体状态与回调行为见 [SteeringWheel.hpp](SteeringWheel.hpp) 的现有 Doxygen。

`Steering4::applyVelocity()` 在 `!isReady()` 时会直接返回，不会下发速度。这是有意设计，不是“忘记控制”。它的目的是防止校准期间被上层误发运动指令。

## 轮组速度与舵向优化

轮组目标角使用 `deg`、轮向速度使用 `rpm`。实现会在等效方向间选择较短转向路径，必要时反向驱动轮向速度或使用跨圈等效角；上层不需再实现一套“翻轮”算法。角度与电机传动/偏移配置必须一致，配置细节见 [SteeringWheel.hpp](SteeringWheel.hpp)。

还有一个故意保守的处理：

- 当某个轮组的目标线速度接近零时，当前实现会保留该轮组现有舵向角，只把轮向速度设为零，避免静止时来回摆头。

## 轮序和几何方向

轮序在 `Steering4::WheelType` 中固定为：

- `FrontRight`
- `FrontLeft`
- `RearLeft`
- `RearRight`

坐标方向与[仓库综述](../../README.md#共通坐标与单位)一致：

- `x` 指向车体前方
- `y` 指向车体左侧
- `yaw` / `wz` 逆时针为正

底层配置和线速度分解都基于这套方向约定，接线或建模时不要自行换轴而不改配置。

## 周期调用建议

- `update()`：放在运动控制快环中。它不仅负责驱动各轮控制器更新，也负责在非校准阶段刷新底盘反馈速度。
- `startCalibration()`：只应在 `enable()` 成功后调用，且当前实现只打算执行一次启动校准。

如果上层已经有别的绝对零点建立方式，也可以把 `enable_calibration` 设为 `false`，让 `Steering4` 退化成“始终 ready 的四舵轮运动学层”。

一个常见例子是：舵向电机本身采用带双编码器的云台电机，掉电后能够依赖内部绝对参考完成零点恢复。此时本仓库不必再额外接入光电门做启动校准，只需要保证 `steer_offset` 与这套硬件恢复出来的绝对零点定义一致即可。

## 接口参考

逐符号接口、配置与校准状态见 [Steering4.hpp](Steering4.hpp) 和 [SteeringWheel.hpp](SteeringWheel.hpp) 的现有 Doxygen；本说明只维护包级集成和安全约束。
