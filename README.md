# ChassisController 食用指南

## 适用范围

本驱动适用于各种全向底盘的控制，如无特殊说明，本文所述底盘均为全向底盘。

支持的底盘类型：

- [x] : Mecanum4 四轮麦克纳姆轮底盘
- [x] : Omni4 四轮全向轮底盘
- [x] : Steering4 四轮舵轮底盘

支持的定位方式：

- [x] : JustEncoder 底盘编码器定位
- [ ] : Ops 基于全方位定位平台（码盘）的定位
- [x] : EKF 基于编码器解算 + HWT101CT（单轴高精度陀螺仪）+ 雷达的卡尔曼融合定位

支持的控制方式：

- [x] : Master 下位机为主机，接收目标点结合 SCurveProfile 控制
- [x] : Slave 上位机为主机，接收上位机发送的轨迹点并跟随轨迹

## 架构说明

从上面的内容可见，底盘控制有三个主要部分

- 运动学解算 Motion : 该部分的任务是整合底盘上的各类电机，成为一个抽象底盘概念（即可以设置底盘的目标速度，并可以读取底盘的实际速度）

  Motion 具有独立功能，在其还不是一个 `底盘` 时（比如校准未完成，或者正在变形），Motion 将自主更新；当其完成自我更新，成为 `底盘`
  这一抽象概念后，由 Controller 接管（所以直接控制函数 `applyVelocity()` 是受保护的，只能由 Controller 调用）

  通过 `isReady()` 判断底盘是否就绪（是否成为 `底盘`）

  故 Motion 具有独立的 `update()` 函数
- 定位系统 Loc : 该部分的任务是根据 Motion 的解算数据结合外部传感器，为 Controller 提供底盘当前的状态信息，包括
    - 底盘在世界中的位置：`postureInWorld()`
    - 底盘在世界中的速度：`velocityInWorld()`
    - 底盘在自己视角的速度：`velocityInBody()`

  Loc 依赖于 Motion 存在（虽然解算数据是可选的，但是为了保证其依赖关系，我们仍旧让 Loc 持有一个 Motion 的指针）

  由于 Loc 还依赖外部传感器，所以可能需要延迟初始化（比如 EKF 初始化时需要等待上位机的第一次定位数据）
- 控制器 Controller : Controller 是在使用过程中直接受用户控制的对象，当 `Motion::isReady()` 为 `true` 时将由 Controller
  直接控制。

  Controller 必须同时持有 Motion 和 Loc，所以构建时就必须传入二者的引用（这也是为什么 Motion 需要独立更新）

  在基类虚函数中，只提供 `enable` `disable` `stop` 三种必须功能，其余功能根据子类需求实现。同时对持有的 Motion, Loc
  中的常用函数做了转发，方便调用
