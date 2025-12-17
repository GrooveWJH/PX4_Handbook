# PX4 Enhanced Handbook

该目录整理了在调试 HKUST NXT-Dual 以及 PX4 FMU 系列过程中沉淀的手册笔记，覆盖 PX4 架构、ROS 2 外部控制、板级适配与排障记录。

- `general/`：通用原理、参数与接口说明（索引见 `general/index.md`）。
- `ros2/`：ROS 2 与 uXRCE-DDS、话题映射、桥接等说明。
- `external_modes/`：ROS 2 External Flight Modes 解析与实战。
- `hkust_nxt_dual/`：NXT-Dual 板级适配与实验记录。
- `px4_fmu_v5/`：公版 PX4 FMU V5 对照笔记。
- `plans/`：方案设计与技术报告。
- `diagonis/`：故障排查记录。
- `archive/`：板级配置快照，包含 bootloader、Nuttx defconfig 及脚本，便于回溯。

## 快速索引

### 通用与机制

| 文档 | 说明 |
| --- | --- |
| `general/index.md` | 通用文档总索引（入口）。 |
| `general/px4_code_structure.md` | PX4 代码结构与主要模块入口。 |
| `general/px4_nuttx_architecture.md` | PX4 与 NuttX 架构分层。 |
| `general/mavlink_shell_commands.md` | 常用 MAVLink shell 指令。 |
| `general/obstacle_avoidance.md` | 避障机制与关键参数。 |
| `general/estimator_status_control_flags.md` | EKF 控制标志位与 bitmask 解释。 |
| `general/px4_parameter_generation.md` | 参数生成链路与板级差异。 |
| `general/px4_position_topics.md` | 位置相关 uORB 话题对比。 |
| `general/ros2_direct_actuator_pipeline.md` | ROS 2 直通执行器链路与 PWM/DShot 映射。 |

### ROS 2 与外部控制

| 文档 | 说明 |
| --- | --- |
| `ros2/external_controller_overview[important].md` | External Controller / Offboard / External Mode 全景解析。 |
| `general/ros2_external_controller_vs_mavros.md` | ROS2 External Controller vs MAVROS 对比。 |
| `ros2/uxrce_dds_topic_mapping.md` | uXRCE-DDS 话题映射与配置。 |
| `ros2/vrpn_uxrce_dds_bridge.md` | VRPN 到 uXRCE-DDS 桥接流程。 |
| `external_modes/PX4_ROS2_Control_Interface_Explained.md` | External Flight Modes 框架解读。 |
| `external_modes/MatrixHoverExternalMode.md` | Matrix Hover 外部模式实战。 |

### HKUST NXT-Dual 板级

| 文档 | 说明 |
| --- | --- |
| `hkust_nxt_dual/目录关系.md` | 目录/源码结构说明。 |
| `hkust_nxt_dual/串口映射.md` | 串口映射与设备分配。 |
| `hkust_nxt_dual/USB与CDC说明.md` | USB CDC 与端口关系。 |
| `hkust_nxt_dual/USBC改NSH教程.md` | USB-C 口改 NSH 调试。 |
| `hkust_nxt_dual/uXRCE端口.md` | uXRCE-DDS 端口配置。 |
| `hkust_nxt_dual/外部视觉融合.md` | 外部视觉融合设置与验证。 |
| `hkust_nxt_dual/与公版对比.md` | 与公版硬件/参数差异。 |
| `hkust_nxt_dual/TFmini数据流.md` | TFmini 数据流与驱动路径。 |

### 方案与排障

| 文档 | 说明 |
| --- | --- |
| `plans/uxrcedds透传.md` | Micro-XRCE-DDS 远程透传设计报告。 |
| `diagonis/macOSROS2跨机通信故障排查记录.md` | macOS ↔ Linux ROS 2 跨机通信排障。 |

### 对照与归档

| 文档 | 说明 |
| --- | --- |
| `px4_fmu_v5/uXRCE说明.md` | PX4 FMU V5 的 uXRCE-DDS 对照说明。 |
| `archive/README.md` | 历史板级配置快照索引。 |
| `archive/board_config/` | 版本化板级配置与脚本。 |

## 阅读提示

如需在 GitHub 页面预览 PlantUML 生成的 SVG，请使用 Chrome 的 PlantUML extension。

新增文档时，请同步更新 `general/index.md` 与本 README 的索引。
