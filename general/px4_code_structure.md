# PX4 代码学习路径与核心目录索引

本文按照“状态估计 → 控制 → 状态机/失效保护 → 导航 → 控制分配与执行器 → 驱动 → 通信 → uORB 消息总线”的顺序梳理 PX4/PX4-Autopilot 主仓库中最常走的入口目录（以当前主线为准）。每一节都给出目录位置、用途，以及可以直接参考的源码/文档链接，并附 PlantUML 示意图帮助记忆“数据在闭环中如何流动”。

> 代码位置基于 commit `3e1c499d5ddb`；后续版本若有小幅调整，可按同名目录搜索。

## 总览：闭环数据流

```plantuml
@startuml
!theme plain
skinparam componentStyle rectangle

component "EKF2
src/modules/ekf2" as EKF2
component "MC/ FW 控制器
src/modules/mc_*" as CTRL
component "Commander
src/modules/commander" as CMD
component "Navigator
src/modules/navigator" as NAV
component "Control Allocator
src/modules/control_allocator" as CA
component "输出驱动
src/drivers/*" as DRV
component "MAVLink / ROS2
src/modules/mavlink
src/modules/uxrce_dds_client" as COMMS
component "uORB
msg/
src/modules/uORB
platforms/common/uORB" as UORB

EKF2 --> CTRL : 状态 (vehicle_local_position, ...)
CTRL --> CA : setpoint (ActuatorMotors, ...)
CA --> DRV : 控制量 -> 电机/舵机
CMD -> CTRL : 模式/解锁/Failsafe
NAV -> CTRL : 任务/轨迹 setpoint
COMMS <--> CMD : GCS/Offboard/MAVLink
COMMS <--> NAV : 任务上传/状态
UORB <.. EKF2
UORB <.. CTRL
UORB <.. CMD
UORB <.. NAV
UORB <.. CA
UORB <.. DRV
@enduml
```

## 1. EKF2 状态估计

- **入口目录**：`src/modules/ekf2/`
  - `EKF2.cpp`/`EKF2.hpp`：模块入口、uORB 订阅/发布、参数处理。
  - `EKF/` 子目录：ECL EKF 核心数学、观测融合（如 `aid_sources/external_vision/`）。
- **文档参考**：PX4 Modules Reference: [modules_estimator](https://docs.px4.io/main/en/modules/modules_estimator). ECL EKF 调参与解释见 [Tuning the ECL EKF](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf).
- **重点**：`vehicle_visual_odometry` 订阅、`vehicle_local_position`/`vehicle_global_position` 发布都在此。若需要读动捕融合逻辑，可定位 `ev_control.cpp`、`ev_pos_control.cpp` 等文件。

## 2. 控制器（多旋翼/固定翼）

### 多旋翼示例

- `src/modules/mc_att_control/`：姿态控制（引用 `vehicle_attitude`/`vehicle_rates_setpoint` 等话题）。
- `src/modules/mc_pos_control/`：位置/速度控制，负责把 `vehicle_local_position` 转成 `trajectory_setpoint`、`vehicle_attitude_setpoint`。
- `src/modules/mc_rate_control/`：速率环。

全部在 Modules Reference: [modules_controller](https://docs.px4.io/main/en/modules/modules_controller) 中列出。固定翼/VTOL 也在 `src/modules/fw_*`/`vtol_*` 里按同样结构组织。

## 3. Commander（状态机、解锁、失效保护）

- **目录**：`src/modules/commander/`
  - `Commander.cpp`：模式切换、健康检查入口。
  - `arming/`, `health_and_arming_checks/`, `failsafe/` 子目录：单独封装检查逻辑。
- **文档**：Modules Reference [modules_system](https://docs.px4.io/main/en/modules/modules_system).
- **用途**：监听传感器/控制回路状态，决定 `vehicle_status`, `vehicle_control_mode`, `failsafe_flags` 等 uORB 话题，对控制器/Navigator 起到“统筹”作用。

## 4. Navigator（任务/模式行为）

- **目录**：`src/modules/navigator/`
  - `navigator_main.cpp`, `mission.cpp`, `rtl.cpp`, `land.cpp` 等实现各任务模式。
- **功能**：根据 Commander 设置的模式，把任务点/RTL/着陆等轨迹转换为 `position_setpoint_triplet`、`trajectory_setpoint`，供控制器消费。

## 5. 控制分配 & 执行器链路

- **Control Allocator**：`src/modules/control_allocator/`
  - 负责将 `vehicle_torque_setpoint`/`thrust_setpoint` 转为 `ActuatorMotors`/`ActuatorServos`。
- **Mixer 模块库**：`src/lib/mixer_module/`
  - 抽象了 mixer 输入/输出的公共逻辑，供 PX4 传统 mixer 和 Control Allocator 共用。
- **输出驱动接口**：通常在 `src/lib/drivers/` 或 `src/modules` 下，依赖 uORB 消息 `actuator_outputs`, `actuator_motors`, `pwm_input` 等。

## 6. 驱动

- **顶层目录**：`src/drivers/`
  - 按外设分类（IMU/GPS/磁力计/ESC 等），如 `src/drivers/imu/bmi088`, `src/drivers/gps`, `src/drivers/dshot`。
- **文档**：Driver Development [docs](https://docs.px4.io/main/en/middleware/drivers).
- **提示**：驱动通常负责与 PX4 OS 层（NuttX/Posix）交互，并发布底层传感器 uORB 话题（如 `sensor_combined`、`vehicle_imu`）。

## 7. 通信

- **MAVLink**：`src/modules/mavlink/`
  - `Mavlink.cpp`/`mavlink_main.cpp` 负责桥接 uORB ↔ MAVLink。发送/接收哪些消息由 `MAV_X_*` 参数控制。
- **ROS2/uXRCE-DDS**（若使用 ROS2 外部控制）：`src/modules/uxrce_dds_client/`。
- **文档**：MAVLink Messaging [docs](https://docs.px4.io/main/en/mavlink/index).

## 8. uORB 消息总线

- **消息定义**：仓库根目录 `msg/`（每个 `.msg` 对应一个 uORB 话题）。
- **运行时模块**：`src/modules/uORB/`
- **底层实现**：`platforms/common/uORB/`（头文件 `uORB.h`、缓冲实现等）。
- **构建生成物**：编译后在 `build/<config>/uORB/topics/` 目录生成 C++ 头文件（别误认为手写源码）。
- **文档**：详见 [uORB Messaging](https://docs.px4.io/main/en/middleware/uorb)；深入理解可以参考 [uORB deep-dive](https://px4.io/px4-uorb-explained-part-3-the-deep-stuff/)。

## 附：如何选择起点阅读

1. **想看状态估计**：从 `src/modules/ekf2` 入手，`EKF2.cpp -> EKF/aid_sources/*`。
2. **想看姿态/位置控制**：从 `mc_att_control`, `mc_pos_control` 的 `Run()` 入口看起，结合 `vehicle_attitude_setpoint` 等话题。
3. **调试 failsafe**：阅读 `src/modules/commander/Commander.cpp` 中的 `run()`，了解如何设置 `failsafe_flags`。
4. **Mission/RTL 行为**：`src/modules/navigator/mission.cpp`、`rtl.cpp`。
5. **输出链路**：`control_allocator -> actuator_motors -> 驱动 (dshot/pwm)`。
6. **通信**：`src/modules/mavlink/mavlink_main.cpp`，查找 `handle_message_*`。
7. **自定义话题**：在 `msg/` 新建 `.msg`，并在 `CMakeLists` 登记，即可由 uORB 自动生成接口。

这样沿着“估计 → 控制 → 状态机 → 导航 → 分配 → 驱动 → 通信 → uORB”顺序跟读，就能快速建立 PX4 代码全貌。
