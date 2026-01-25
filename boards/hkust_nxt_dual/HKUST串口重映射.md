## HKUST NXT-Dual 串口重映射记录

> 目的：让 USB-C 直接输出 NSH Console、TELEM1 运行 uXRCE-DDS（115200），TELEM2 恢复串行测距仪，彻底停用 MAVLink。

### 1. 配置改动

| 文件 | 修改 | 作用 |
| --- | --- | --- |
| `boards/hkust/nxt-dual/nuttx-config/nsh/defconfig` | 打开 `CONFIG_NSH_USBCONSOLE`、`CONFIG_CDCACM_CONSOLE`，关闭 `CONFIG_USART2_SERIAL_CONSOLE` | 将 NSH 控制台转移到 USB-C (`/dev/ttyACM0`) |
| `boards/hkust/nxt-dual/init/rc.board_defaults` | `MAV_0_CONFIG=0`、`UXRCE_DDS_CFG=101`、`SER_TEL1_BAUD=115200` | 禁用 MAVLink，默认让 TELEM1 跑 uXRCE |
| 同文件 | `SENS_TFMINI_CFG=102` | 让 TELEM2 重新启用 TFmini/串行测距仪 |
| `PX4_Handbook/boards/hkust_nxt_dual/串口映射.md` | 更新 TELEM1/TELEM2/USB 行描述 | 反映最新串口用途 |
| `PX4_Handbook/boards/hkust_nxt_dual/uXRCE端口.md` | 改为 “TELEM1 → uXRCE” 流程 | 与实际固件一致 |

### 2. 各端口职责（现状）

| 物理口 | `/dev/tty*` | 角色 | 默认波特率 | 备注 |
| --- | --- | --- | --- | --- |
| UART2（TELEM1） | `/dev/ttyS1` | uXRCE-DDS Client | 921 600（默认） | `UXRCE_DDS_CFG=101`，伴飞电脑直接连接 UART2；可在参数中调整波特率 |
| UART4（TELEM2） | `/dev/ttyS3` | 串行测距仪/TFmini | 115 200 | `SENS_TFMINI_CFG=102` 自动启动 TFmini 驱动，如需改用其它外设先禁用该参数 |
| UART7（TELEM3） | `/dev/ttyS6` | 备用 | 57 600 | 可按需开启自定义串口/调试口 |
| UART8（TELEM4） | `/dev/ttyS7` | 空闲/自定义 | 57 600 | 默认不绑定功能，可用作调试或串行外设；如需 uXRCE/MAVLink，可在参数里设为 104 |
| USB-C CDC | `/dev/ttyACM0` | NSH Console / Bootloader | N/A（CDC 自适应） | `CONFIG_NSH_USBCONSOLE=y`，插线连 PC 即得 `nsh>`；刷机流程不变 |

### 3. 验证清单
1. 刷机后用 USB-C 连接电脑，`screen /dev/ttyACM0` 连续敲 3 次回车，应出现 `nsh>`。
2. 伴飞头连 TELEM1（UART2），在电脑上运行 `MicroXRCEAgent serial --dev /dev/ttyUSBx -b 115200`。
3. NSH 中 `uxrce_dds_client status` 显示 “Serial @ /dev/ttyS1” 且 `Running, connected`。
4. 若需验证激光测距仪，连接 TELEM2 并执行 `listener distance_sensor`；如要改走其它串口，先将 `SENS_TFMINI_CFG` 设为 0。

> 如需进一步调整波特率或串口角色，只需修改 `SER_*_BAUD` 与 `UXRCE_DDS_CFG` / `MAV_*_CONFIG` 等参数，无需动核心 PX4 代码。***
