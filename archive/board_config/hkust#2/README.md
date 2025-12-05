# HKUST NXT-Dual — Snapshot #2

- **来源目录**：`boards/hkust`
- **快照时间**：2025-12-05
- **变更简介**：USB-C 改为 NSH，uXRCE 调整到 TELEM1，TELEM2 预留给 TFmini，新增 `/fmu/out/distance_sensor`。

## 串口/接口意图

| 接口 | NuttX 设备 | 作用 | 波特率 | 关键参数 |
| --- | --- | --- | --- | --- |
| USB-C (CDCACM) | `/dev/ttyACM0` | 作为 NSH 控制台，三次回车触发 `cdcacm_autostart` | `SER_USB_BAUD=115200` | `CONFIG_NSH_USBCONSOLE=y`, `CONFIG_CDCACM_CONSOLE=y`, `MAV_0_CONFIG=0`
| TELEM1 | `/dev/ttyS1` | uXRCE-DDS 客户端物理通道，连上伴飞电脑 | `SER_TEL1_BAUD=921600` | `UXRCE_DDS_CFG=101`（选择 TELEM1）
| TELEM2 | `/dev/ttyS3` | 串行激光测距仪/TFmini | `SER_TEL2_BAUD=115200` | `SENS_TFMINI_CFG=102`
| TELEM3 | `/dev/ttyS6` | 预留（无默认服务） | `SER_TEL3_BAUD` follow board default | —
| TELEM4 / UART8 | `/dev/ttyS7` | 预留 debug/自定义（之前的 uXRCE 口） | 115200（默认） | 可在后续固件中新配置 `UXRCE_DDS_CFG=104`
| RC 串口 | `/dev/ttyS4` | 遥控输入 | — | `CONFIG_BOARD_SERIAL_RC=/dev/ttyS4`

## 主要文件差异记录

- `default.px4board`
  - `CONFIG_BOARD_SERIAL_TEL1=/dev/ttyS1`、`TEL2=/dev/ttyS3`、`TEL4=/dev/ttyS7`。
  - `CONFIG_DRIVERS_CDCACM_AUTOSTART=y` + `CONFIG_MODULES_UXRCE_DDS_CLIENT=y` 保证 USB-NSH 和 DDS 客户端均编译。
- `nuttx-config/nsh/defconfig`
  - 启用 `CONFIG_NSH_USBCONSOLE` 与 `CONFIG_CDCACM_CONSOLE`，屏蔽 `CONFIG_USART6_SERIAL_CONSOLE`，USB 直接提供 nsh。
  - UART2 (TELEM1) 设为 115200 以上缓冲，UART4 (TELEM2) 115200，UART8 已启用。
- `init/rc.board_defaults`
  - `MAV_0_CONFIG=0` 禁用 USB MAVLink，`UXRCE_DDS_CFG=101` 将 uXRCE 固定在 TELEM1。
  - `SER_USB_BAUD=115200`、`SER_TEL1_BAUD=921600`、`SER_TEL2_BAUD=115200`，并开启 `SENS_TFMINI_CFG=102`。

## 额外说明

- `src/modules/uxrce_dds_client/dds_topics.yaml` 已补充 `/fmu/out/distance_sensor`，刷机后可在 ROS 2 侧直接 `ros2 topic echo /fmu/out/distance_sensor` 查看 TFmini 数据。
- 该快照仅复制板级目录（`nxt-dual`、`nxt-v1` 等），如需恢复请整体覆盖 `boards/hkust`。
