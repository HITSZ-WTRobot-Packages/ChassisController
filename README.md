# ChassisController

面向共享 STM32 宿主项目的全向底盘控制库，以 `Motion + Loc + Controller` 分离轮组运动学、定位和控制模式。本仓库作为 Git SubModule 接入，不是可独立运行的固件工程，不创建任务，也不负责通信协议、设备底层驱动、全局路径规划或整机业务状态机。

## 三层设计与包导航

| 层级 | 职责 | 包文档 / cpkg 标识 |
|---|---|---|
| Core | 公共数据结构、能力接口与控制权契约 | [Core](Core/README.md)：`Chassis::Core` |
| Motion | 轮组正逆解算、执行器使能和车体速度反馈 | [Mecanum4](Chassis/Mecanum4/README.md)：`Chassis::Mecanum4`；[Omni4](Chassis/Omni4/README.md)：`Chassis::Omni4`；[Steering4](Chassis/Steering4/README.md)：`Chassis::Steering4` |
| Loc | 融合 Motion 和可选外部观测，输出位姿与速度 | [JustEncoder](Localization/JustEncoder/README.md)：`ChassisLocalization::JustEncoder`；[EKF](Localization/EKF/README.md)：`ChassisLocalization::EKF` |
| Controller | 接收目标、规划或跟踪，并向 Motion 下发速度 | [Master](Controller/Master/README.md)：`Chassis::ControllerMaster`；[Slave](Controller/Slave/README.md)：`Chassis::ControllerSlave` |

Motion 只覆盖全向底盘的平面运动，升降、伸缩等独立自由度应由其他模块抽象。Loc 可以在 Motion 已经运行后才建立初始状态；Controller 不应依赖某种轮组或定位后端的内部结构。业务层优先持有 Controller，通过 Loc 读取状态，不绕过 Controller 直接写轮组速度。

包目录中的 `cpkg.toml` 是依赖与包边界的依据。宿主通过 cpkg 解析依赖并集成生成的构建目标，不另写仓库级构建入口。上表标识也是相应的 CMake alias；头文件 include 目录由包目标导出。

## 依赖与宿主责任

实际依赖由各包清单及实现决定：`Chassis::Core` 当前声明依赖 `FreeRTOS` 和 `MotorDrivers::Controller`，具体包还会引入 GPIO、曲线、PD、EKF、陀螺仪和容器等依赖，见各包说明与清单。Master / Slave 的实现直接使用 CMSIS-RTOS2 互斥量及中断锁；EKF 使用 `HAL_GetTick()`；Steering4 校准依赖 GPIO EXTI。这些是实际实现的集成要求，不能把接口分层理解成可直接替换 HAL、RTOS 或驱动的兼容保证。

宿主负责底层设备对象、CubeMX 静态硬件配置、时间基准和周期调度。硬件配置的专项限制见包文档；库不会因被接入就自动创建任务或修复宿主配置。本仓库没有面向所有 STM32 系列的统一兼容性或运行验证承诺。

## 共通坐标与单位

车体坐标系 `x` 指向前方、`y` 指向左侧、`z` 向上；`yaw` / `wz` 逆时针为正。世界系原点和绝对朝向由 Loc 初始状态与外部观测定义，不由 Motion 定义。

| 数据 | 单位 | 坐标语义 |
|---|---|---|
| `Velocity::vx` / `vy` | `m/s` | 所属坐标系由接口名及参数契约指定 |
| `Velocity::wz` | `deg/s` | 绕竖直轴角速度 |
| `Posture::x` / `y` | `m` | 世界系位置，或目标接口明确指定的相对位移 |
| `Posture::yaw` | `deg` | 朝向或相对转角 |

Motion 反馈为车体系速度；Loc 提供车体系速度、世界系速度和世界系位姿，并提供坐标变换。不要把角度换成 `rad` 后直接传入，也不要把世界系速度当成车体系速度。几何配置的 `mm`、底层电机速度的 `rpm`、后端时间参数及目标参考系保持方式见各包文档。类型与符号细节见 [Core 头文件导航](Core/README.md#接口参考)。

## 对象生命周期与接入顺序

1. 先建立底层电机控制器和所需传感器，再构造 Motion。
2. 按包要求使 Motion 可用；需要校准或等待首帧观测时，先维持 Motion 的更新，不提前假设定位已建立。
3. 初始状态可用后构造依赖 Motion 的 Loc，再构造依赖二者的 Controller。
4. 正常使用优先调用最上层 Controller 的 `enable()`，检查返回值；使能会向下传递，不需要重复逐个使能底层对象。`enable()` 成功不等于校准完成，应检查就绪状态。
5. 分别调度各层所需的周期入口，确认可工作后再设置目标。

这些对象引用底层依赖而不接管其生命周期：设备应比 Motion 存活更久，Motion 应比 Loc / Controller 存活更久，Loc 应比 Controller 存活更久。需要延迟构造时，调用方必须在周期回调访问对象前确认已构造，并在销毁前停止相关调用。

## 使能与控制权交接

执行器使能和 Controller 控制权是两件事。同一 Motion 同时只允许一个 Controller 写速度；Controller 只有持有控制权、Motion 已使能且 ready 时才认为自己 `enabled()`。`acquireControl()` 成功后会进入 `stop()` 状态，但不使能执行器；`releaseControl()` 不会关闭执行器。`disable()` 会释放控制权并关闭底层执行器，不应用作只交接模式的操作。

同一套 Motion + Loc 可共用多个 Controller。切换时按“旧控制器 `stop()` → `releaseControl()` → 新控制器 `acquireControl()`”交接，检查申请成功后再写新目标；执行器可以保持使能。失去控制权的控制器无法通过基类继续下发速度，但这不意味着并发调度和对象生命周期自动安全，仍须宿主协调。

## 周期调度边界

Motion、Loc、Controller 均没有统一的基类 `update()` 契约。具体入口和周期参数归对应包说明所有：轮组更新、定位推进、轨迹推进、误差闭环与速度下发可能需要不同节拍。调用方可以合并或拆分任务，但必须维持数据、时间基准及调用顺序，不能只调用一个控制入口便假定所有层都已更新。源码注释中的推荐频率不是仓库强制的线程模型。

具体接入请从上表选择所用包；包文档保留消费者可见的单位、启动、调度和失败边界，逐符号行为以链接的现有 Doxygen 为参考。
