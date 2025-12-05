# HKUST NXT-Dual — Snapshot #1

- **来源目录**：`boards/hkust`
- **快照时间**：2025-12-05
- **变更简介**：保持 USB-C 为 MAVLink/QGC，TELEM1 承担 NSH，TELEM4(UART8) 运行 uXRCE-DDS。

## 串口/接口映射

| 接口 | NuttX 设备 | 角色/用途 | 波特率 | 关键参数/宏 |
| --- | --- | --- | --- | --- |
| USB-C (CDCACM) | `/dev/ttyACM0` | QGC/MAVLink 连接与固件刷写 | `SER_USB_BAUD=115200` | `MAV_0_CONFIG=301`, `CONFIG_DRIVERS_CDCACM_AUTOSTART=y` |
| TELEM1 | `/dev/ttyS1` | NSH Console（外部 UART 适配器） | `SER_TEL1_BAUD=115200` | `CONFIG_USART2_BAUD=115200`, `CONFIG_USART2_SERIAL_CONSOLE=y` |
| TELEM2 | `/dev/ttyS3` | TFmini/串行测距仪 | `SER_TEL2_BAUD=115200` | `SENS_TFMINI_CFG=102`, `CONFIG_USART4_BAUD=921600` |
| TELEM3 | `/dev/ttyS6` | 备用 Telemetry | `SER_TEL3_BAUD=115200` | `CONFIG_BOARD_SERIAL_TEL3=/dev/ttyS6` |
| TELEM4 / UART8 | `/dev/ttyS7` | uXRCE-DDS（对接伴飞计算机） | `SER_TEL4_BAUD=115200` | `UXRCE_DDS_CFG=104` |
| RC 串口 | `/dev/ttyS4` | SBUS/DSMX 等遥控输入 | — | `CONFIG_BOARD_SERIAL_RC=/dev/ttyS4` |

## 关键文件概览

- `default.px4board`
  - 设定 `TEL1=/dev/ttyS1`、`TEL2=/dev/ttyS3`、`TEL4=/dev/ttyS7` 的别名。
  - 启用了 `CONFIG_DRIVERS_CDCACM_AUTOSTART`、`CONFIG_MODULES_UXRCE_DDS_CLIENT`，默认还包含 MAVLink 模块。
- `nuttx-config/nsh/defconfig`
  - NSH 仍在 UART2（TELEM1）上，USB 仅做 CDC 设备；`CONFIG_USART6_SERIAL_CONSOLE` 关闭。
  - UART4（TELEM2）波特率提高到 921600，为高速测距仪预留吞吐，UART8 设备已启用。
- `init/rc.board_defaults`
  - `MAV_0_CONFIG=301` 让 USB 自动起 MAVLink，`UXRCE_DDS_CFG=104` 指向 TELEM4，`SENS_TFMINI_CFG=102` 打开 TFmini。
  - `SER_*_BAUD` 统一设为 115200，符合未调优前的默认值。

## 额外说明

- 此快照代表“原厂”配置：QGroundControl 经 USB 连接，uXRCE-DDS 通过 UART8（板上标注 Debug/UART8）对接伴随计算机。
- 恢复该状态时建议整体覆盖 `boards/hkust` 目录，以避免遗漏 bootloader、extras 等配套文件。
