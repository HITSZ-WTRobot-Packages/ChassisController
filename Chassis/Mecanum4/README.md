# Mecanum4

`Chassis::Mecanum4` 提供四轮麦克纳姆 Motion，支持 `XType` / `OType` 构型，不承担定位或轨迹规划。依赖见 [cpkg.toml](cpkg.toml)：`Chassis::Core`、`FreeRTOS`；底层轮速控制器使用 Core 引入的 `MotorDrivers::Controller`。

## 配置与接入

由调用方提供四个有效的 `MotorVelController`，按右前、左前、左后、右后的轮序绑定，且其生命周期必须覆盖 Motion。`wheel_radius` 为轮半径，`wheel_distance_x` / `wheel_distance_y` 为完整前后 / 左右轮距，均用 `mm`，不是半轮距。构型必须与实际轮组安装一致。

底盘接口遵循[公共坐标与单位](../../README.md#共通坐标与单位)：车体 x 向前、y 向左，速度为 `m/s`、`deg/s`；轮速控制器接口使用 `rpm`。几何参数不能直接传入以米计的数值。

先构造 Motion，再绑定 Loc 与 Controller；普通控制通过 Controller 使能并设置目标。当前 Motion 没有额外校准就绪阶段，`isReady()` 恒为真，但仍需检查使能结果。任一轮使能失败会整体回滚；成功使能会重置底层电机角度，不要把重复使能当作无副作用操作。

## 周期调用与边界

宿主周期调用 `update()` 更新轮速控制器，`forwardGetVelocity()` 从轮速反馈反解车体系速度。源码建议 `update()` 约 `1kHz`，这不是自动创建的任务，也不替代 Loc / Controller 自身更新。包不负责配置底层总线、编码器或电机硬件；这些由宿主与相应驱动负责。

配置字段、正逆解算和逐符号细节见 [Mecanum4.hpp](Mecanum4.hpp) 的现有 Doxygen；控制权及生命周期见[仓库综述](../../README.md)。
