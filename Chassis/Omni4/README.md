# Omni4

`Chassis::Omni4` 提供四轮全向轮 Motion，不承担定位融合或轨迹规划。依赖见 [cpkg.toml](cpkg.toml)：`Chassis::Core`、`FreeRTOS`；底层使用 Core 引入的 `MotorDrivers::Controller` 轮速控制器。

## 配置与接入

调用方提供四个有效且存活时间覆盖 Motion 的 `MotorVelController`，按右前、左前、左后、右后绑定。`wheel_radius` 为轮半径，`wheel_distance_x` / `wheel_distance_y` 为完整前后 / 左右轮距，全部使用 `mm`，不要传入半轮距或以米计的数值。

底盘速度采用[公共坐标与单位](../../README.md#共通坐标与单位)：车体 x 向前、y 向左，`vx/vy` 为 `m/s`，`wz` 为 `deg/s`；底层轮速接口使用 `rpm`。轮组安装和驱动反馈方向必须匹配模型。

先构造 Motion，再构造依赖它的 Loc / Controller；普通控制通过 Controller 使能和设置目标。当前 `isReady()` 恒为真，但不代表执行器已使能，应检查使能结果。任一轮使能失败会整体回滚；成功使能会重置底层电机角度，不要把重复使能当作无副作用操作。

## 周期调用与边界

宿主在运动控制周期中调用 `update()` 驱动四轮速度控制器，通过 `forwardGetVelocity()` 获取车体系反馈速度；还需分别调度所选 Loc / Controller。库不创建任务，不配置底层电机、总线或编码器硬件。

逐符号说明和运动学配置见 [Omni4.hpp](Omni4.hpp) 的现有 Doxygen；共同生命周期与控制权约束见[仓库综述](../../README.md)。
