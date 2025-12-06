# HKUST NXT-Dual — Snapshot #3

- **来源目录**：`boards/hkust`（PX4 commit 3e1c499）
- **快照时间**：2025-12-06
- **变更简介**：USB-C 回到 MAVLink、NSH 移至 UART8；TELEM1 专供 uXRCE (921600)，TELEM2 自动启动 TFmini 距离传感器。

## 串口/接口映射

| 接口 | NuttX 设备 | 角色/用途 | 波特率 | 关键参数/宏 |
| --- | --- | --- | --- | --- |
| USB-C (CDCACM) | `/dev/ttyACM0` | MAVLink (cdcacm_autostart) | `USB_MAV_MODE=2` | `CONFIG_CDCACM_CONSOLE` 禁用，`SYS_USB_AUTO=2` |
| TELEM1 | `/dev/ttyS1` | uXRCE-DDS 客户端 | `SER_TEL1_BAUD=921600` | `UXRCE_DDS_CFG=101` |
| TELEM2 | `/dev/ttyS3` | TFmini/串行测距仪 | `SER_TEL2_BAUD=115200` | `SENS_TFMINI_CFG=102` |
| TELEM3 | `/dev/ttyS6` | 备用 | 57600 默认 | — |
| TELEM4 / UART8 | `/dev/ttyS7` | NSH Console | 921600 | `CONFIG_UART8_SERIAL_CONSOLE=y`, `CONFIG_UART8_BAUD=921600` |
| RC 串口 | `/dev/ttyS4` | SBUS RC 输入 | — | `CONFIG_BOARD_SERIAL_RC` |

## 关键文件概览

- `default.px4board`
  - 串口别名：`TEL1=/dev/ttyS1`、`TEL2=/dev/ttyS3`、`TEL4=/dev/ttyS7`。
  - 保持 `CONFIG_DRIVERS_CDCACM_AUTOSTART=y`、`CONFIG_MODULES_UXRCE_DDS_CLIENT=y`。
- `nuttx-config/nsh/defconfig`
  - 关闭 `CONFIG_CDCACM_CONSOLE`/`CONFIG_NSH_USBCONSOLE`，并启用 `CONFIG_UART8_SERIAL_CONSOLE`，让 NSH 走 UART8。
- `init/rc.board_defaults`
  - `UXRCE_DDS_CFG=101`、`SER_TEL1_BAUD=921600`（uXRCE in TELEM1）。
  - `SENS_TFMINI_CFG=102`、`SER_TEL2_BAUD=115200`（TFmini in TELEM2）。
  - 移除过时的 `SER_USB_BAUD`、`UXRCE_DDS_PRT` 等参数。

## 额外说明

- uXRCE-DDS Topic 列表在 `src/modules/uxrce_dds_client/dds_topics.yaml` 中有 `/fmu/out/distance_sensor` 等自定义项。
- 如需恢复 USB=NSH 的配置，可参考快照 #2；若需公版配置参照快照 #1。
- 建议在刷机后执行 `param reset SENS_TFMINI_CFG UXRCE_DDS_CFG SER_TEL1_BAUD SER_TEL2_BAUD` 以确保默认值生效。 
