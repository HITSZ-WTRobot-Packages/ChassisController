# JustEncoder

`ChassisLocalization::JustEncoder` 是仅按 Motion 反馈速度积分的平面定位后端，不引入外部绝对观测。依赖见 [cpkg.toml](cpkg.toml)：`Chassis::Core`，并继承其清单依赖。

## 接入与周期

先构造 Motion，再以它构造 `chassis::loc::JustEncoder loc(motion)`；Motion 生命周期应覆盖 Loc。默认从零世界系位姿开始，世界系原点和初始朝向由此建立。随后把 Motion 与 Loc 交给所选 Controller，遵守[共同接入顺序](../../README.md#对象生命周期与接入顺序)。

在定位周期调用 `loc.update(dt_s)`，`dt_s` 是本次积分间隔，单位为秒，不是毫秒或 HAL tick。调用方负责提供正确的实际时间间隔，并让 Motion 反馈与其更新节奏协调；本包不读取硬件时钟，不自动调度更新，也不需要等待首次外部定位观测。

## 输出与限制

输出遵循 [IChassisLoc](../../Core/IChassisLoc.hpp)：车体系速度、世界系速度及世界系位姿。位置为 `m`，线速度为 `m/s`，yaw 为 `deg`，角速度为 `deg/s`；车体 x 向前、y 向左、逆时针为正。

纯积分无法用绝对观测修正累积误差，轮滑、速度偏差和时间误差会影响定位。它不是绝对定位或三维姿态后端。若要融合外部位姿或陀螺仪，应选择具备相应输入的定位包，而不是把它们隐式塞入此包。

逐符号接口见 [JustEncoder.hpp](JustEncoder.hpp) 的现有 Doxygen；融合定位方案见 [EKF](../EKF/README.md)。
