# ChassisController 食用指南

`ChassisController` 是当前机器人工作区中的一个底盘控制模块，用来把“轮组运动学”“定位解算”和“上层控制模式”拆成稳定的接口边界，方便在不改上层业务逻辑的前提下切换不同底盘结构、定位后端和控制方式。

这不是一个可独立运行的整机工程。当前目录只提供底盘相关抽象与实现，不负责创建任务、不负责通信链路、不负责电机或传感器底层驱动，也不负责整机业务状态机。

## 当前支持能力与 TODO

- [x] 底盘类型
    - [x] 统一抽象面向全部全向底盘
    - [x] `Mecanum4` 四轮麦克纳姆，支持 `XType` / `OType` 构型
    - [x] `Omni4` 四轮全向轮
    - [x] `Steering4` 四轮舵轮
    - [ ] 更多全向底盘结构
- [x] 定位后端
    - [x] `JustEncoder`，仅依赖 Motion 反馈速度做纯积分定位
    - [x] `LocEKF`，融合编码器速度、`HWT101CT` 航向与外部雷达位姿观测
    - [ ] `Ops`，基于全方位定位平台（码盘）的定位
- [x] 控制模式
    - [x] `Master`，下位机为主机，内部持有轨迹曲线和误差闭环
    - [x] `Slave`，上位机为主机，下位机消费轨迹点并执行跟踪
    - [ ] 更多控制模式扩展
- [x] 依赖边界
    - [x] 通过工作区已有 `MotorVelController` / `MotorPosController` / 陀螺仪驱动等抽象接入

## 库定位

这个库解决的问题：

- 把全向底盘统一抽象成 `Motion + Loc + Controller` 三层能力。
- 用统一的 `Velocity` / `Posture` 语义表示底盘速度与位姿。
- 让上层逻辑优先依赖 `IChassisMotion`、`IChassisLoc`、`IChassisController` 这些能力接口，而不是依赖某个具体底盘型号或某个定位协议分支。
- 为不同控制模式提供一致入口，例如“直接速度控制”“相对位姿目标”“绝对位姿目标”“轨迹点跟踪”。

这个库不解决的问题：

- 电机驱动器、编码器、陀螺仪、雷达等底层设备驱动。
- CAN / UART / SPI / GPIO 等总线或外设协议细节。
- 上下位机通信协议、任务创建、线程模型和业务工程目录结构。
- 全局路径规划、任务编排、地图管理和整机行为决策。
- 与全向底盘平面运动独立的额外自由度。例如某些带升降机构的特殊底盘，整机可以被视作 4-dof “飞行器”，但升降自由度与全向底盘工作范围相互独立，因此不并入本仓库的 Motion 抽象。

## 适用范围

当前实现面向以下场景：

- 平台：MCU 侧 C++17 工程，作为更大工作区中的一个模块接入。
- 运行环境：当前实现默认工作区已提供 `FreeRTOS` / `CMSIS-RTOS2`，部分后端还依赖
  `HAL_GetTick()`、GPIO 中断回调等 MCU 常见基础设施。
- 设备类型：抽象层面覆盖全部全向底盘；当前仓库已经实现的具体 Motion 主要是麦克纳姆轮、四全向轮、四舵轮。
- 总线范围：本库本身不直接抽象总线，底层总线差异应在 `MotorVelController`、`MotorPosController`、陀螺仪驱动等依赖模块中消化。
- 定位输入：可仅依赖底盘运动学反馈，也可叠加陀螺仪和外部定位观测。

这几个环境假设是“当前工作区实现的接入前提”，不是这个仓库在概念上的唯一可能接法。若未来工作区替换 RTOS、HAL 或底层驱动抽象，只要保留这里的公开语义，上层依赖关系仍然可以保持不变。

## 阅读导航

主 README 负责说明统一抽象、接入顺序和公共语义。遇到专项约束时，再进入对应子目录 README：

- `Core/`：公共数据结构和接口基类，阅读入口是 [`Core/IChassisDef.hpp`](Core/IChassisDef.hpp)、[`Core/IChassisMotion.hpp`](Core/IChassisMotion.hpp)、[`Core/IChassisLoc.hpp`](Core/IChassisLoc.hpp)、[`Core/IChassisController.hpp`](Core/IChassisController.hpp)
- [Chassis/Steering4/README.md](Chassis/Steering4/README.md)：四舵轮的校准流程、轮组转向优化与就绪条件
- [Localization/EKF/README.md](Localization/EKF/README.md)：EKF 状态定义、时间戳要求、雷达观测回放机制
- [Controller/Master/README.md](Controller/Master/README.md)：下位机主控模式的姿态/速度控制接口与周期任务拆分
- [Controller/Slave/README.md](Controller/Slave/README.md)：上位机主控模式的轨迹点缓冲、消费节奏与跟踪方式

## 设计理念与分层

底盘控制被拆成三层，不是为了“多一层封装”，而是为了稳定上层依赖面。

| 层级         | 目录              | 解决的问题                          | 上层应该依赖什么                                                                         |
|------------|-----------------|--------------------------------|----------------------------------------------------------------------------------|
| Motion     | `Chassis/`      | 轮组正逆解算、使能/失能、反馈底盘速度            | `IChassisMotion` 的使能、就绪状态、`forwardGetVelocity()`                                 |
| Loc        | `Localization/` | 融合 Motion 与外部传感器，输出位姿与速度       | `IChassisLoc` 的 `postureInWorld()`、`velocityInWorld()`、`velocityInBody()` 和坐标系变换 |
| Controller | `Controller/`   | 对外提供更高层的控制模式，例如速度控制、位姿控制、轨迹点跟踪 | `IChassisController` 以及控制器子类自身的目标设置接口                                            |

这样拆分的原因：

- `Motion` 可能在“电机已存在但底盘尚未 ready”的阶段独立工作，例如舵轮校准或机构变形阶段。控制器不能假设一上电就能直接发底盘速度。
- `Loc` 既依赖 `Motion` 的反馈，又可能依赖外部传感器，因此其初始化节奏不必和 `Motion` 完全一致。
- `Controller` 只关心“当前底盘状态是什么、我要把它带到哪里”，不该知道某个轮组是麦克纳姆还是舵轮，也不该知道定位后端到底是纯编码器还是融合定位。

因此，上层业务代码应尽量依赖能力而不是实现：

- 需要发送普通速度指令时，依赖 `Controller` 的速度接口，而不是直接操作轮子。
- 需要读取底盘状态时，依赖 `IChassisLoc` 的统一位姿/速度接口，而不是从某个后端私有结构中取值。
- 需要切换底盘类型或定位后端时，优先替换构造阶段的具体类，尽量不改控制层和业务层的语义代码。

## 当前模块导出的 CMake 入口

当前目录会把不同实现导出成多个 CMake target，供上层工作区按需链接：

| 类别           | CMake alias                                                   |
|--------------|---------------------------------------------------------------|
| Core         | `Chassis::Core`                                               |
| Motion       | `Chassis::Mecanum4`、`Chassis::Omni4`、`Chassis::Steering4`     |
| Localization | `ChassisLocalization::JustEncoder`、`ChassisLocalization::EKF` |
| Controller   | `Chassis::ControllerMaster`、`Chassis::ControllerSlave`        |

这些 target 的 include 目录由本模块自己导出，所以示例代码中直接写 `#include "Mecanum4.hpp"` 这类头文件名即可。具体是由哪个上级
`CMakeLists.txt` 把当前模块纳入工作区，由外层工程决定。

## 能力模型

### Motion 层

所有底盘运动学实现最终都表现为“可使能、可更新、能反馈车体速度”的对象：

- `enable()` / `disable()`：底盘电机级使能控制。
- `isReady()`：底盘是否已经可以被当作完整 `底盘` 使用。对于 `Mecanum4` 和 `Omni4`，当前实现始终 ready；对于
  `Steering4`，启用校准时只有完成校准后才 ready。
- `forwardGetVelocity()`：返回 Motion 当前估算到的车体坐标系速度。

`applyVelocity()` 被设计成受保护接口，只允许 `IChassisController` 及其派生类调用。这是为了避免业务层绕过控制器直接下发底盘速度，破坏控制模式切换时的状态一致性。

需要特别注意的是，这里的
`Motion` 抽象只覆盖“全向底盘平面运动”本身。对于带升降、伸缩或其他独立机构的特殊底盘，即使整机从业务角度看可以被理解为更高自由度系统，这些与全向底盘工作范围独立的自由度也不应直接并入本仓库；它们应在上层或其他模块中独立抽象，再与这里的底盘平面运动能力组合使用。

另一个需要特别强调的点是：虽然很多 Motion 实现都会提供自己的
`update()`，但本仓库不再要求在基类中统一约束这个更新入口。原因是特殊需求下，不同 Motion 的更新语义和调用节奏可能并不一致，最终应由用户按具体实现自行调度。

### Loc 层

定位后端统一提供三类输出：

- `velocityInBody()`：车体坐标系速度。
- `velocityInWorld()`：世界坐标系速度。
- `postureInWorld()`：世界坐标系位姿。

同时，`IChassisLoc` 内置了一组坐标变换工具：

- `WorldVelocity2BodyVelocity()`
- `BodyVelocity2WorldVelocity()`
- `WorldPosture2BodyPosture()`
- `BodyPosture2WorldPosture()`
- `RelativePosture2WorldPosture()`
- `WorldPosture2RelativePosture()`

这意味着上层如果只依赖 `IChassisLoc`，通常不需要重复实现一遍坐标变换。

和 Motion 一样，Loc 也可能有各自独立的更新入口，例如 `update(dt)`、`update()`、`updateLidar(...)` 等。本仓库不在基类里统一规定单一
`update()` 形式，调用者需要根据具体后端自行决定何时、以什么参数调用这些更新函数。

### Controller 层

控制器是业务代码直接持有和调用的对象。当前两个实现分工如下：

- `Master`：下位机自己做轨迹规划和误差闭环，上层下发速度或目标位姿。
- `Slave`：上位机主导轨迹生成，下位机只负责消费轨迹点并跟踪。

所有控制器都必须同时持有一个 `Motion` 和一个 `Loc`。因此，构造顺序必须是：

1. 先构造 `Motion`
2. 再构造依赖它的 `Loc`
3. 最后构造依赖二者的 `Controller`

从接入关系上看，`Motion + Loc + Controller` 这一整套底盘对象应被视为“逐层封装起来的高级控制器”：

- 上层业务通常直接持有最上层 `Controller`
- `Controller` 再向下持有 `Motion` 和 `Loc`
- `Motion` 再继续持有更底层的轮组控制器 / 电机控制器

因此在正常接入里，应优先使能最上层 `Controller`，而不是在业务层重复逐个使能下层对象。对当前实现而言：

- `Controller::enable()` 会继续调用 `Motion::enable()`
- `Motion::enable()` 又会继续把使能传递到更底层的轮组或电机控制器
- `Loc` 一般不承担独立 enable 语义，它主要负责状态反馈

也就是说，当你已经把底盘当成一个高级控制器使用时，通常不需要再额外手动对 `Motion` 或更底层轮控重复执行 `enable()`。

同样地，Controller 层也不强制规定统一的基类更新函数。像 `Master` 会拆成 `profileUpdate()`、`errorUpdate()`、
`controllerUpdate()`，`Slave` 则会拆成 `trajectoryUpdate()`、`errorUpdate()`。这些入口本来就是为不同控制节拍服务的，因此应由用户自行调度，而不是依赖某个统一的基类
`update()` 约束。

## 单位与语义约定

对外公开接口统一使用 [`Core/IChassisDef.hpp`](Core/IChassisDef.hpp) 中的两个基础结构：

| 类型 / 字段        | 物理意义             | 单位      |
|----------------|------------------|---------|
| `Velocity::vx` | 车体前向速度，正方向指向车体前方 | `m/s`   |
| `Velocity::vy` | 车体侧向速度，正方向指向车体左侧 | `m/s`   |
| `Velocity::wz` | 绕竖直轴角速度，逆时针为正    | `deg/s` |
| `Posture::x`   | 世界坐标系 `x` 轴位置分量  | `m`     |
| `Posture::y`   | 世界坐标系 `y` 轴位置分量  | `m`     |
| `Posture::yaw` | 绕竖直轴角度，逆时针为正     | `deg`   |

几个容易混淆的例外和补充约定：

- `Mecanum4::Config`、`Omni4::Config`、`Steering4::Config` 中的轮半径和轮距使用 `mm`，构造后会立即转成内部 `m`。
- 舵轮内部和轮速控制器之间的速度参考使用 `rpm`，因为底层电机控制器接口就是按 `rpm` 给定。
- `JustEncoder::update(dt)` 的 `dt` 单位是秒。
- `LocEKF::updateLidar(pos, ticks)` 里的 `ticks` 必须和该后端内部采样使用的时间戳属于同一时间基准。这个“同一时间基准”可以是业务工程先换算过的统一时基；驱动库本身不负责上下位机对时。
- 世界坐标系的原点和绝对朝向由定位后端决定。`JustEncoder` 默认从零位姿开始积分，`LocEKF` 则由初值配置和外部观测共同决定。

`Master` 中有一个特别需要强调的语义：

`Master` 的目标设置接口不是简单重名重载，而是明确分成 7 种组合：

- `Posture` 目标 3 种：绝对目标、相对自身目标、相对指定基准点目标
- `Velocity` 目标 4 种：输入表达坐标系（世界 / 车身） × 后续保持不变的参考系（世界 / 车身）

其中：

- `setTargetPostureInBody()` 适合“相对自己”的简单动作
- `setTargetPostureRelativeTo()` 适合“相对某个特定基准点”的动作，例如某段动作必须以某个起点为基准执行
- `setVelocityInWorld(world_velocity, target_in_world)` 的 `world_velocity` 一定是世界坐标系速度
- `setVelocityInBody(body_velocity, target_in_world)` 的 `body_velocity` 一定是车体坐标系速度
- `target_in_world` 不是在重复声明参数属于哪个坐标系，而是在声明“控制器后续是否要把保存下来的参考速度视为世界坐标系不变，并在每个周期重新投影回车体坐标系”

更完整的 7 种组合说明见 [Controller/Master/README.md](Controller/Master/README.md)。

如果只想给底盘一个普通车体坐标系速度指令，通常应使用：

- `setVelocityInBody(body_velocity, false)`

如果希望机器人在旋转时仍保持世界坐标系下的平移方向不变，则可以使用：

- `setVelocityInWorld(world_velocity, true)`

## 初始化与接入方式

推荐的最小接入顺序如下：

1. 准备底层电机控制器和传感器对象。这些对象来自工作区其他模块，不由本仓库创建。
2. 构造一个具体 `Motion`，例如 `Mecanum4`、`Omni4` 或 `Steering4`。
3. 在条件允许时构造一个具体 `Loc`，例如 `JustEncoder` 或 `LocEKF`。
4. 构造一个具体 `Controller`，例如 `Master` 或 `Slave`。
5. 调用控制器的 `enable()` 使能底盘。
6. 如果底盘是启用校准的 `Steering4`，在使能后调用 `startCalibration()`，并持续调用 `update()` 直到 `motion.isReady()` 变为
   `true`。
7. 进入周期任务，分别驱动 Motion、Loc 和 Controller 的更新接口。

关于使能关系，再强调一次：

- 当上层直接持有 `Controller` 时，推荐只调用 `Controller::enable()`
- 不需要在正常流程里再手动对 `Motion` 或更底层轮速/舵向控制器重复执行 `enable()`
- 重复使能虽然在某些实现里未必立即出错，但会破坏“底盘作为高级控制器统一接入”的语义边界

关于更新调用，再强调一次：

- `Motion`、`Loc`、`Controller` 三个部分都可能各自需要独立调用更新函数
- 本仓库不再在基类层面统一约束 `update()` 形式，也不要求三者必须共用同一种更新入口
- 用户需要根据具体实现，自行完成这些更新函数的调度与调用
- 因此这里给出的周期建议只是“当前实现常见写法”，不是基类层面的统一要求

对 `Loc` 的构造需要特别补充一点：

- `Loc` 往往不一定能在系统启动瞬间就完成构造。
- 某些后端需要等待底盘校准完成，或者等待上位机返回第一次绝对定位结果，才能确定初始状态。
- 对 `LocEKF` 来说，一个很典型的场景就是：初始观测由“雷达首次返回的位姿 + 陀螺仪首次数据”共同给出，因此往往要等拿到这两类首帧数据后，才能确定
  `x_init` 等初始条件并构造 `LocEKF`。
- 因此在真实接入里，`Loc` 常常是“依赖事件的延迟构造对象”。
- 由于 `Controller` 又依赖 `Loc`，所以很多工程里 `Controller` 也会跟着延迟构造。

这类接法下，常见做法是：

- `Motion` 先构造并先开始更新。
- `Loc` 和 `Controller` 先保持为 `nullptr`。
- 等到“校准完成”“上位机首次回包”“雷达首次返回位姿和陀螺仪首次数据已就绪”“外部定位已就绪”等事件到来后，再显式构造 `Loc` 和
  `Controller`。
- 在周期任务里调用 `Loc` / `Controller` 的 `update()` 或控制接口之前，先检查对象是否已经构造完成，也就是是否为 `nullptr`。

周期任务不一定必须拆成独立线程，但职责要区分清楚。结合源码注释与现有实现，下面列出的只是“当前实现常见更新入口”的调用建议，而不是基类统一约束：

| 组件                                 | 入口                        | 作用                 | 当前实现中的调用建议                        |
|------------------------------------|---------------------------|--------------------|-----------------------------------|
| `Mecanum4` / `Omni4` / `Steering4` | `update()`                | 刷新轮级控制器、舵轮校准和反馈速度  | 运动控制快环；`Mecanum4` 源码注释推荐约 `1kHz`  |
| `JustEncoder`                      | `update(dt_s)`            | 用底盘反馈速度做积分定位       | 与定位更新周期一致，`dt_s` 为秒               |
| `LocEKF`                           | `update()`                | 采样编码器速度与陀螺仪，推进滤波器  | 固定周期调用，周期需与构造参数 `delta_ticks` 对齐  |
| `LocEKF`                           | `updateLidar(pos, ticks)` | 注入外部定位观测，并在必要时回放状态 | 在外部定位数据到达时调用；调用前先由接入工程把时间戳对齐到统一时基 |
| `Master`                           | `profileUpdate(dt_s)`     | 推进 S 曲线轨迹          | 源码注释建议约 `100Hz`                   |
| `Master`                           | `errorUpdate()`           | 基于当前位置和目标做 PD 跟踪   | 源码注释建议约 `200~500Hz`               |
| `Master`                           | `controllerUpdate()`      | 处理纯速度模式下的控制下发      | 放在控制快环中                           |
| `Slave`                            | `trajectoryUpdate()`      | 从缓冲区取出下一个轨迹点       | 与上位机轨迹点消费节奏一致                     |
| `Slave`                            | `errorUpdate()`           | 在车体坐标系下跟踪当前轨迹点     | 放在控制快环中                           |

上表是当前实现的推荐方式，不是唯一线程模型。只要保持数据语义不变，上层可以把它们合并到同一任务，也可以拆成多个周期任务；更重要的是，调用者需要自己明确管理这三层各自的更新函数，而不是假设基类会提供统一
`update()` 接口。

## 最小调用样例

下面的例子展示一个“先构造 Motion，再按事件延迟构造 Loc / Controller”的接入流程。示例仍然使用仓库内真实存在的
`Mecanum4 + JustEncoder + Master` 符号；其中 `JustEncoder` 本身通常不依赖首次外部结果，这里只是顺带演示延迟构造写法。对
`LocEKF` 这类“常常要等雷达首次返回位姿与陀螺仪首次数据，再用它们共同定义初始观测和 `x_init`”的后端，这种写法更是典型场景。

```cpp
#include "Mecanum4.hpp"
#include "JustEncoder.hpp"
#include "Master.hpp"

struct ChassisHandles
{
    chassis::motion::Mecanum4*       motion     = nullptr;
    chassis::loc::JustEncoder*       loc        = nullptr;
    chassis::controller::Master*     controller = nullptr;
};

void InitChassis(ChassisHandles&                    handles,
                 controllers::MotorVelController&   wheel_fr,
                 controllers::MotorVelController&   wheel_fl,
                 controllers::MotorVelController&   wheel_rl,
                 controllers::MotorVelController&   wheel_rr)
{
    static chassis::motion::Mecanum4 motion({
        .wheel_radius     = 76.0f,
        .wheel_distance_x = 420.0f,
        .wheel_distance_y = 360.0f,
        .chassis_type     = chassis::motion::Mecanum4::ChassisType::XType,
        .wheel_front_right = &wheel_fr,
        .wheel_front_left  = &wheel_fl,
        .wheel_rear_left   = &wheel_rl,
        .wheel_rear_right  = &wheel_rr,
    });

    handles.motion     = &motion;
    handles.loc        = nullptr;
    handles.controller = nullptr;
}

void OnFirstLocReadyEvent(ChassisHandles& handles)
{
    static chassis::loc::JustEncoder loc(*handles.motion);

    static const chassis::controller::Master::Config master_cfg{
        .posture_error_pd_cfg =
                {
                    .vx = { .Kp = 4.0f, .Kd = 0.8f, .abs_output_max = 2.0f },
                    .vy = { .Kp = 4.0f, .Kd = 0.8f, .abs_output_max = 2.0f },
                    .wz = { .Kp = 3.0f, .Kd = 0.3f, .abs_output_max = 180.0f },
                },
        .limit = {
            .x   = { .max_spd = 1.5f, .max_acc = 3.0f, .max_jerk = 20.0f },
            .y   = { .max_spd = 1.5f, .max_acc = 3.0f, .max_jerk = 20.0f },
            .yaw = { .max_spd = 180.0f, .max_acc = 360.0f, .max_jerk = 1800.0f },
        },
    };

    static chassis::controller::Master controller(*handles.motion, loc, master_cfg);

    handles.loc        = &loc;
    handles.controller = &controller;

    handles.controller->enable();
    handles.controller->stop();
}

void ChassisFastLoop(ChassisHandles& handles, float dt_s)
{
    handles.motion->update();

    if (handles.loc != nullptr)
        handles.loc->update(dt_s);

    if (handles.controller != nullptr)
    {
        handles.controller->errorUpdate();
        handles.controller->controllerUpdate();
    }
}

void ChassisProfileLoop(ChassisHandles& handles, float dt_s)
{
    if (handles.controller != nullptr)
        handles.controller->profileUpdate(dt_s);
}

void MoveForwardHalfMeter(ChassisHandles& handles)
{
    if (handles.controller == nullptr)
        return;

    handles.controller->setTargetPostureInBody({
        .x   = 0.50f,
        .y   = 0.00f,
        .yaw = 0.00f,
    });
}
```

如果要切换为别的实现，优先替换构造阶段对象即可：

- 换底盘结构：替换 `Mecanum4` 为 `Omni4` 或 `Steering4`
- 换定位后端：替换 `JustEncoder` 为 `LocEKF`
- 换控制方式：替换 `Master` 为 `Slave`

若改为 `Steering4` 或
`LocEKF`，还需要补上各自子目录 README 中描述的专项初始化和周期调用要求，见 [Chassis/Steering4/README.md](Chassis/Steering4/README.md) 和 [Localization/EKF/README.md](Localization/EKF/README.md)。

## 什么时候看主 README，什么时候看子 README

优先看主 README 的情况：

- 想理解这个库总体抽象了什么
- 想知道上层应该依赖哪些能力接口
- 想快速完成一次最小接入
- 想确认单位、坐标系和控制入口的统一语义

进入子目录 README 的情况：

- 使用 `Steering4`，需要知道校准流程、就绪条件和舵向优化行为，见 [Chassis/Steering4/README.md](Chassis/Steering4/README.md)
- 使用 `LocEKF`，需要知道时间戳要求、状态回放和观测输入边界，见 [Localization/EKF/README.md](Localization/EKF/README.md)
- 使用 `Master` 或
  `Slave`，需要知道各自周期接口应该如何分工调用，见 [Controller/Master/README.md](Controller/Master/README.md) 和 [Controller/Slave/README.md](Controller/Slave/README.md)

如果后续新增别的底盘家族、定位后端或控制模式，也应遵循同样原则：公共抽象和统一语义写在主 README，专项时序、约束和兼容性问题写进对应子目录 README。
