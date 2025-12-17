# Changelog

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
