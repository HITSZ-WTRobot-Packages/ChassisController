# Chassis::Core

本包定义底盘三层共同使用的数据结构与能力接口，不提供具体轮组运动学、定位融合或控制模式。依赖以 [cpkg.toml](cpkg.toml) 为准：`FreeRTOS`、`MotorDrivers::Controller`。不能因为接口头文件较轻量就忽略清单依赖。

## 集成契约

- `Velocity` 使用 `m/s`、`deg/s`，`Posture` 使用 `m`、`deg`；车体 x 向前、y 向左，角度逆时针为正。具体接口决定数据所处坐标系，世界系由定位后端建立。
- Motion 管理执行器、反馈车体速度并仲裁唯一 Controller 写入者；Loc 输出世界系位姿及两坐标系速度；Controller 对外承接控制目标。平面之外的机构不在接口职责内。
- 使能不等于就绪，控制权不等于执行器使能；不要绕过 Controller 写入 Motion。共同生命周期、交接和调度要求见[仓库综述](../README.md)。
- 三种基类均不统一规定周期更新入口，具体类的周期契约由其包说明给出。接口的控制权检查不构成宿主并发调度安全保证。

## 接口参考

逐符号说明使用现有 Doxygen，不在此重复 API：

- [IChassisDef.hpp](IChassisDef.hpp)：速度与位姿类型。
- [IChassisMotion.hpp](IChassisMotion.hpp)：Motion 能力、就绪与控制权。
- [IChassisLoc.hpp](IChassisLoc.hpp)：Loc 输出与坐标变换。
- [IChassisController.hpp](IChassisController.hpp)：Controller 生命周期与控制权接口。
