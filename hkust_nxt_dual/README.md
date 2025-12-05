# HKUST NXT-Dual 研究目录

本目录收录“不会随着某个快照变化而反复改写”的方法论，所有具体的板级文件请查看 `archive/board_config/hkust#*/`。当需要复原某一阶段的配置，只需复制对应快照；这里的文章只讲解“怎么查、怎么改、为什么要这样做”。

| 文档 | 核心内容 |
| --- | --- |
| `目录关系.md` | `boards/hkust/nxt-dual/` 目录每个文件夹的角色、生成流程（附 PlantUML）。 |
| `与公版对比.md` | HKUST 固件 vs PX4 公版的差异点（驱动、参数、接口），便于对照升级。 |
| `USB与CDC说明.md` | USB-C/CDC 端口在 Bootloader 与应用阶段的行为，如何同时兼顾刷写、NSH、MAVLink。 |
| `USBC改NSH教程.md` | 通用步骤：把任何板子的 USB CDC 改成 NSH 控制台，需要动哪些配置。 |
| `串口映射.md` | 如何从 `.px4board`/`defconfig`/`board.h` 推导 `/dev/ttyS*` 与 PCB 丝印的对应关系，并给出当前快照的示例。 |
| `uXRCE端口.md` | uXRCE-DDS 串口的选择逻辑：`UXRCE_DDS_CFG` 与 `SER_TEL*_BAUD` 的作用、调试指引。 |
| `TFmini数据流.md` | 串行测距仪驱动（TFmini）数据从 UART 采集到 `distance_sensor` uORB 的流程。 |

> 📌 提示：串口、参数等“会随固件演进”但又需要留痕的内容，都快照到 `archive/board_config/hkust#N`。手册只引用快照编号，例如“以下示例基于 #2 (2025-12-05)”。这样即使将来有 #3、#4，本文也无需重写。 
