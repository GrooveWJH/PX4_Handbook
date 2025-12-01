# PX4 Enhanced Handbook

该目录整理了在调试 HKUST NXT-Dual 以及 PX4 FMU 系列过程中沉淀的手册笔记。

- `general/`：跨平台通用的说明，如 PX4 与 NuttX 架构、避障机制等。
- `hkust_nxt_dual/`：针对 HKUST NXT-Dual 定制固件的研究记录。
- `px4_fmu_v5/`：公版 PX4 FMU V5 的对照笔记。
- `archive/`：板级配置快照，包含 bootloader、Nuttx defconfig 及脚本，便于回溯。

## 快速索引

| 主题 | 文档 |
| --- | --- |
| 架构与 Shell | `general/` 内的架构篇、指令篇、避障篇 |
| HKUST 板研究 | `hkust_nxt_dual/README.md` 与子文档 |
| 公版对照 | `px4_fmu_v5/uXRCE说明.md` |
| 配置快照 | `archive/README.md` 与 `board_config/` |

以下提交会持续补充目录说明、串口映射、XRCE-DDS 配置等内容，保证与代码改动保持同步。
