# Archive 目录说明

`archive/` 用于保存不同时间节点的板级配置快照，避免在调参或固件升级时丢失原始状态。

- `board_config/hkust#1/`：第一次修改 NXT 系列 bootloader、Nuttx 配置及相关脚本。
- `board_config/hkust#2/`：USB 改为 NSH、TELEM1=uXRCE、TELEM2=TFmini，并补齐 `distance_sensor` 话题。
- `board_config/README.example.md`：创建新快照时复制此模板，撰写 `README.md`，记录串口映射、关键参数及额外说明。
- 后续如需新增快照，请建立 `board_config/<name>/` 子目录，复制板级源码后编写 `README.md`，说明用途与关键变化。

该目录仅做备份，不建议直接在其中编辑生效版本，若需修改配置请在工作副本中进行并重新导出快照。
