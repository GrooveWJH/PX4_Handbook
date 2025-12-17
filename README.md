# PX4 Enhanced Handbook

该目录整理了在调试 HKUST NXT-Dual 以及 PX4 FMU 系列过程中沉淀的手册笔记，覆盖 PX4 架构、ROS 2 外部控制、板级适配与排障记录。

如需在 GitHub 页面预览 PlantUML 生成的 SVG，请使用 Chrome 的 PlantUML extension。

- [core/](core/)：通用原理、参数与接口说明（索引见 [core/index.md](core/index.md)）。
- [ros2/](ros2/)：ROS 2 与 uXRCE-DDS、External Flight Modes、话题映射、桥接等说明。
- [boards/](boards/)：板级适配与对照（[boards/hkust_nxt_dual/](boards/hkust_nxt_dual/)、[boards/px4_fmu_v5/](boards/px4_fmu_v5/)）。
- [plans/](plans/)：方案设计与技术报告。
- [diagnostics/](diagnostics/)：故障排查记录。
- [archive/](archive/)：板级配置快照，包含 bootloader、Nuttx defconfig 及脚本，便于回溯。

## 快速索引

### 通用与机制

| 文档 | 说明 |
| --- | --- |
| [core/index.md](core/index.md) | 通用文档总索引（入口）。 |
| [core/px4_code_structure.md](core/px4_code_structure.md) | PX4 代码结构与主要模块入口。 |
| [core/px4_nuttx_architecture.md](core/px4_nuttx_architecture.md) | PX4 与 NuttX 架构分层。 |
| [core/mavlink_shell_commands.md](core/mavlink_shell_commands.md) | 常用 MAVLink shell 指令。 |
| [core/obstacle_avoidance.md](core/obstacle_avoidance.md) | 避障机制与关键参数。 |
| [core/estimator_status_control_flags.md](core/estimator_status_control_flags.md) | EKF 控制标志位与 bitmask 解释。 |
| [core/px4_parameter_generation.md](core/px4_parameter_generation.md) | 参数生成链路与板级差异。 |
| [core/px4_position_topics.md](core/px4_position_topics.md) | 位置相关 uORB 话题对比。 |

### ROS 2 与外部控制

| 文档 | 说明 |
| --- | --- |
| [ros2/external_controller_overview[important].md](ros2/external_controller_overview%5Bimportant%5D.md) | External Controller / Offboard / External Mode 全景解析。 |
| [ros2/ros2_external_controller_vs_mavros.md](ros2/ros2_external_controller_vs_mavros.md) | ROS2 External Controller vs MAVROS 对比。 |
| [ros2/ros2_direct_actuator_pipeline.md](ros2/ros2_direct_actuator_pipeline.md) | ROS 2 直通执行器链路与 PWM/DShot 映射。 |
| [ros2/uxrce_dds_topic_mapping.md](ros2/uxrce_dds_topic_mapping.md) | uXRCE-DDS 话题映射与配置。 |
| [ros2/vrpn_uxrce_dds_bridge.md](ros2/vrpn_uxrce_dds_bridge.md) | VRPN 到 uXRCE-DDS 桥接流程。 |
| [ros2/external_modes/PX4_ROS2_Control_Interface_Explained.md](ros2/external_modes/PX4_ROS2_Control_Interface_Explained.md) | External Flight Modes 框架解读。 |
| [ros2/external_modes/MatrixHoverExternalMode.md](ros2/external_modes/MatrixHoverExternalMode.md) | Matrix Hover 外部模式实战。 |

### HKUST NXT-Dual 板级

| 文档 | 说明 |
| --- | --- |
| [boards/hkust_nxt_dual/目录关系.md](boards/hkust_nxt_dual/%E7%9B%AE%E5%BD%95%E5%85%B3%E7%B3%BB.md) | 目录/源码结构说明。 |
| [boards/hkust_nxt_dual/串口映射.md](boards/hkust_nxt_dual/%E4%B8%B2%E5%8F%A3%E6%98%A0%E5%B0%84.md) | 串口映射与设备分配。 |
| [boards/hkust_nxt_dual/USB与CDC说明.md](boards/hkust_nxt_dual/USB%E4%B8%8ECDC%E8%AF%B4%E6%98%8E.md) | USB CDC 与端口关系。 |
| [boards/hkust_nxt_dual/USBC改NSH教程.md](boards/hkust_nxt_dual/USBC%E6%94%B9NSH%E6%95%99%E7%A8%8B.md) | USB-C 口改 NSH 调试。 |
| [boards/hkust_nxt_dual/uXRCE端口.md](boards/hkust_nxt_dual/uXRCE%E7%AB%AF%E5%8F%A3.md) | uXRCE-DDS 端口配置。 |
| [boards/hkust_nxt_dual/外部视觉融合.md](boards/hkust_nxt_dual/%E5%A4%96%E9%83%A8%E8%A7%86%E8%A7%89%E8%9E%8D%E5%90%88.md) | 外部视觉融合设置与验证。 |
| [boards/hkust_nxt_dual/与公版对比.md](boards/hkust_nxt_dual/%E4%B8%8E%E5%85%AC%E7%89%88%E5%AF%B9%E6%AF%94.md) | 与公版硬件/参数差异。 |
| [boards/hkust_nxt_dual/TFmini数据流.md](boards/hkust_nxt_dual/TFmini%E6%95%B0%E6%8D%AE%E6%B5%81.md) | TFmini 数据流与驱动路径。 |

### 方案与排障

| 文档 | 说明 |
| --- | --- |
| [plans/uxrcedds透传.md](plans/uxrcedds%E9%80%8F%E4%BC%A0.md) | Micro-XRCE-DDS 远程透传设计报告。 |
| [diagnostics/macOSROS2跨机通信故障排查记录.md](diagnostics/macOSROS2%E8%B7%A8%E6%9C%BA%E9%80%9A%E4%BF%A1%E6%95%85%E9%9A%9C%E6%8E%92%E6%9F%A5%E8%AE%B0%E5%BD%95.md) | macOS ↔ Linux ROS 2 跨机通信排障。 |

### 对照与归档

| 文档 | 说明 |
| --- | --- |
| [boards/px4_fmu_v5/uXRCE说明.md](boards/px4_fmu_v5/uXRCE%E8%AF%B4%E6%98%8E.md) | PX4 FMU V5 的 uXRCE-DDS 对照说明。 |
| [archive/README.md](archive/README.md) | 历史板级配置快照索引。 |
| [archive/board_config/](archive/board_config/) | 版本化板级配置与脚本。 |

新增文档时，请同步更新 [core/index.md](core/index.md) 与本 README 的索引。
