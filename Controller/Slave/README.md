# Slave

`Slave` 是“上位机主导轨迹、下位机负责执行”的控制模式。它不自己生成路径，而是消费外部塞进来的轨迹点，再在本地做误差闭环跟踪。

如果你的系统里轨迹规划和节拍控制主要发生在上位机，或者下位机只被要求做一个稳定的轨迹跟踪执行器，通常应选这个模式。

## 核心接口

`Slave` 是一个模板类：

```cpp
template <size_t BufferCapacity>
class Slave : public IChassisController;
```

模板参数 `BufferCapacity` 是轨迹点缓冲区容量。头文件中已经给出了一个实际估算方式：

- `BufferCapacity >= 轨迹时长(s) * (主机发送频率(Hz) - trajectoryUpdate 调用频率(Hz))`

这意味着缓冲区大小不是随便填的，而要和上位机发点节奏、下位机消费节奏一起算。

## 轨迹点语义

每个轨迹点由两部分组成：

- `p_ref`：世界坐标系参考位姿
- `v_ref`：世界坐标系参考速度

调用流程如下：

1. 上位机或接入层持续调用 `pushTrajectoryPoint()`
2. 下位机周期调用 `trajectoryUpdate()`，从缓冲区取出下一个点
3. 下位机周期调用 `errorUpdate()`，把当前参考点转换到车体坐标系下做跟踪

`pushTrajectoryPoint()` 返回 `false` 时表示缓冲区已满，当前轨迹点没有写入。

## 为什么误差在 body frame 下计算

当前实现的 `errorUpdate()` 会做两次转换：

- 用 `WorldPosture2BodyPosture()` 计算“参考位姿相对当前车体”的误差
- 用 `WorldVelocity2BodyVelocity()` 把参考速度转换到车体坐标系

然后再把前馈速度和 PD 输出叠加，最终通过 `applyVelocity()` 下发给 `Motion`。

这样做的目的，是让底盘执行器始终接收车体坐标系速度，而不要求底层运动学层自己理解世界系目标。

## 周期调用建议

- `trajectoryUpdate()`：负责“消费下一点”，频率由轨迹发送和期望消费速度决定
- `errorUpdate()`：负责“跟踪当前点”，应放在控制快环里持续调用

如果缓冲区为空，`trajectoryUpdate()` 会直接返回，当前参考点保持不变。这意味着底盘会继续朝最后一次成功取出的参考点进行跟踪，而不是自动清空目标。

## stop 语义

`stop()` 的行为是：

- 把 `stopped_` 设为 `true`
- 把当前位置记为新的参考位姿
- 把参考速度清零

因此它更接近“以当前位置为锁定点停止”，而不是简单地丢弃所有状态。

## 使用边界

- `Slave` 自身不定义上位机通信协议，也不关心轨迹点从哪里来。
- `Slave` 不做轨迹平滑或重规划，输入点的质量直接决定跟踪效果。
- 如果上位机发点过快、消费过慢，或者 `BufferCapacity` 过小，轨迹缓冲将成为接入工程需要自行处理的风险点。

## 与主 README 的边界

主 README 负责说明 `Controller` 是上层直接持有的对象。本文件负责说明 `Slave` 的专项语义：轨迹点缓冲区怎么用、为什么误差在车体坐标系下计算、以及 `trajectoryUpdate()` 和 `errorUpdate()` 的职责分工。
