# Changelog

## 2025-12-17
- 目录重构：将文档划分为 `core/`、`ros2/`、`boards/`、`diagnostics/`，并将 ROS 2 / External Controller 资料集中到 `ros2/` 下，刷新 README 索引结构。
- README：整理导航链接、完善“结构概览”，并新增“PX4 Autopilot 中文社群 (562665086)” QQ 群入口。
- ros2：为 direct actuator pipeline 文档补充 DShot/PWM 公式、数据流 PlantUML，以及 External Controller/Matrix Hover 相关说明。
- README：移除“文档写作要点”临时内容，改为在根目录新增 `template.md`，示范标准写法并用引用块强调要点；`ros2_direct_actuator_pipeline.md` 延伸统一用语。
- template：模板改为高泛化格式，适配对比型、链路型、排障型文章，并要求源码引用使用 PX4 GitHub 链接示例；补充 PlantUML 要求及示例片段，统一绘图语言，并强调源码引用必须使用 Markdown 链接形式方便跳转。

## 2025-12-16
- core：新增 PX4 代码结构概览；撰写 `estimator_control_mode_flags` 解析；整理 PlantUML 区块；补充 `estimator control flags` 对照表（包含 EKF flag 细节）。

## 2025-12-15
- core：新增参数生成与 uORB 里程计话题指南，解释 EKF 外部视觉融合行为。

## 2025-12-05
- archive：新增 `board_config/hkust#2` 快照（USB=NSH、TELEM1=uXRCE、TELEM2=TFmini），并把 `hkust#1`、`hkust#2` 的说明统一为 README 形式，附 `README.example` 模板。
- `boards/hkust_nxt_dual/`：重写 README、`串口映射.md`、`uXRCE端口.md`，改为引用快照的方法论文档。
- `core/`：新增 `PX4_uXRCE_DDS话题配置.md`，同时更新 `core/index.md`。
- `.gitignore`：补充 macOS/Linux 杂项、PX4/NuttX 构建产物、ROS 工作区、STM32 产物及 CMake 临时文件的忽略规则。

## 2025-12-04
- archive：导入 `board_config/hkust#1` 快照（含 `nxt-dual`、`nxt-v1` 的 bootloader、Nuttx 配置、板级源码及脚本）。
- docs：迁移 `core/` 下的基线文章（MAVLink Shell、PX4 与 Nuttx 架构、TFmini 数据流、USB/CDC/NSH 教程等），建立手册主体内容。

## 2025-12-03
- `boards/hkust_nxt_dual/串口映射.md`：记录 USART2/TELEM1 控制台提速的说明，方便串口调试时设置 115200 以上带宽。

## 2025-11-30
- 建立 `PX4_Enhanced_Handbook` 目录，迁移原 `docs_gg_mba` 内容。
- 新增 README 与 archive 说明，明确文档及快照的用途。
