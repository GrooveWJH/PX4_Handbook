# Archive 目录说明

`archive/` 用于保存不同时间节点的板级配置快照，避免在调参或固件升级时丢失原始状态。

- `board_config/hkust#1/`：保留最初的 NXT 系列 bootloader、Nuttx 配置及相关脚本，一旦需要可直接比较差异。
- 后续如需新增快照，请建立 `board_config/<name>/` 子目录，并在 `declare.txt` 中描述用途与关键变化。

该目录仅做备份，不建议直接在其中编辑生效版本，若需修改配置请在工作副本中进行并重新导出快照。
