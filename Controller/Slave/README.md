# Slave

`Chassis::ControllerSlave` 消费外部生成的离散轨迹点，在本地进行误差闭环跟踪，不生成路径或通信协议。依赖见 [cpkg.toml](cpkg.toml)：`Chassis::Core`、`libs::PID_PD`、`libs::RingBuffer`。实现直接使用 CMSIS-RTOS2 互斥量和中断锁；宿主必须提供相应环境，不能把包含阻塞互斥量操作的 `stop()` 或控制权申请当作任意 ISR 可调用接口。

## 模式与对象接入

模板参数为 `Slave<BufferCapacity, Offline>`，`Offline` 默认为 `false`：

- 在线模式要求 `BufferCapacity >= 2`，通过 `pushTrajectoryPoint()` 入队、`trajectoryUpdate()` 逐点消费。
- 离线模式使用 `Slave<0, true>`，不持有轨迹缓冲，通过 `profileUpdate(point)` 直接设置当前参考点；仍由 `errorUpdate()` 完成闭环。

先建立 Motion 和可用的 Loc，再构造本控制器并通过控制权接口使其可工作；两者生命周期须覆盖本控制器。控制权、使能与就绪的区别见[仓库综述](../../README.md)。

## 轨迹点、单位与坐标系

`TrajectoryPoint` 的 `p_ref` 为世界系位姿，`v_ref` 为世界系前馈速度。位置用 `m`、yaw 用 `deg`，线速度用 `m/s`、角速度用 `deg/s`；世界系必须与 Loc 一致。输入点应连续且使用一致的 yaw 表达，不应假定控制器自动消除跨 ±180° 的输入跳变。

当前误差闭环在**世界坐标系**计算，叠加世界系前馈后再转换为车体系速度下发给 Motion；因此 `PDConfig` 的 x/y 轴按世界系解释，不是车体系误差参数。底层始终收到车体系速度。

## 周期与缓冲契约

在线调用顺序为：生产方 `pushTrajectoryPoint()` → 以轨迹采样节拍调用 `trajectoryUpdate()` → 在控制快环持续调用 `errorUpdate()`。离线模式以 `profileUpdate(point)` 替代入队和消费，调用方负责参考点更新节拍。轨迹点没有独立时间戳或持续时间，库不按到达时间自动重采样、插值或规划。

缓冲容量需覆盖发送与消费的累计差、突发和通信抖动。头文件中的“轨迹时长 ×（发送频率 − 消费频率）”仅可用作恒定速率下积压量的估算，不能替代容量下限或负载分析。生产速度持续大于消费速度时，有限缓冲最终仍会满。

- `pushTrajectoryPoint()` 返回 `false` 表示已满，本点没有写入，调用方必须处理。
- `trajectoryUpdate()` 仅在控制器可工作时消费；缓冲为空时保留上一次成功取得的参考**位姿和速度**，不会自动停止或把前馈速度清零。
- 缺点、断流、点间连续性和输入约束由调用方负责；本包不提供通信失联保护、轨迹平滑或自动重规划。需要停止时必须显式进入停止流程。

## 停止与交接

`stop()` 清空在线模式未消费轨迹，把当前位置设为锁定参考、前馈速度清零并重置误差 PD；继续调用 `errorUpdate()` 才能维持锁定。它不是执行器失能。`clearTrajectory()` 只丢弃尚未消费的点，不改变已生效参考点及速度，因此不能代替停止。控制权交接后按新模式显式设置新目标。

## 接口参考

完整模板接口及逐符号行为见 [Slave.hpp](Slave.hpp) 的现有 Doxygen；公共坐标与单位见[仓库综述](../../README.md#共通坐标与单位)。
