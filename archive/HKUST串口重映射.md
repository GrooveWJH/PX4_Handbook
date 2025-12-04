## HKUST NXT-Dual 串口重映射记录

> 目的：保留 TELEM1 作为 NSH 调试口、TELEM2 运行 uXRCE-DDS（115200），UART8/TELEM4 恢复空闲，USB CDC 继续提供 MAVLink/QGC/刷机通道。

### 1. 配置改动

| 文件 | 修改 | 作用 |
| --- | --- | --- |
| `boards/hkust/nxt-dual/init/rc.board_defaults` | `UXRCE_DDS_CFG=102`、`SER_TEL2_BAUD=115200`、`SENS_TFMINI_CFG=0` | uXRCE-DDS 回到 TELEM2，串行测距仪默认关闭 |
| 同文件 | 删除 `SER_TEL4_BAUD` | UART8/TELEM4 恢复空闲 |
| `docs_gg_mba/hkust_nxt_dual/串口映射.md` | 更新 TELEM2= uXRCE、TELEM4=空闲 | 反映最新映射 |
| `docs_gg_mba/hkust_nxt_dual/uXRCE端口.md` | 改为 “TELEM2 → uXRCE” 流程 | 与实际固件一致 |

### 2. 各端口职责（现状）

| 物理口 | `/dev/tty*` | 角色 | 默认波特率 | 备注 |
| --- | --- | --- | --- | --- |
| UART2（TELEM1） | `/dev/ttyS1` | NSH Console | 115 200 | `CONFIG_USART2_SERIAL_CONSOLE=y`，调试线直连此口即可进 shell |
| UART4（TELEM2） | `/dev/ttyS3` | uXRCE-DDS Client | 115 200（默认） | `UXRCE_DDS_CFG=102`，接上伴飞即可输出 XRCE；可在参数中调整波特率 |
| UART7（TELEM3） | `/dev/ttyS6` | 备用 | 57 600 | 可按需开启 MAVLink 或作为额外调试口 |
| UART8（TELEM4） | `/dev/ttyS7` | 空闲/自定义 | 57 600 | 默认不绑定功能，可用作调试或串行外设；如需 uXRCE/MAVLink，可在参数里改到 104 |
| USB-C CDC | `/dev/ttyACM0` | MAVLink/QGC/刷机 | N/A（CDC 自适应） | `MAV_0_CONFIG=301`；bootloader 仍走同一口 |

### 3. 与其它串口的关系
- 若要重新启用串行测距仪，可把 `SENS_TFMINI_CFG` 设回 102（或其它端口），但需避让 uXRCE 线束。
- 想使用 UART8：可将 `UXRCE_DDS_CFG` 改成 104 或在 `MAV_*_CONFIG` 中开启 MAVLink2，记得同时更新 `SER_TEL4_BAUD`。

### 4. 验证清单
1. 刷机后用 USB-C 连接 QGC，确认仍能正常识别并上电调参。
2. 串好 UART2→USB 转串口，`screen /dev/ttyUSBx 115200` 可看到 `nsh>`。
3. ROS2 伴飞端连接 TELEM2（UART4）并运行 `MicroXRCEAgent serial --dev /dev/ttyUSBx -b 115200`，NSH `uxrce_dds_client status` 显示 “Serial @ /dev/ttyS3”。
4. 若需测试 UART8，手动在参数里切换 `UXRCE_DDS_CFG=104` 并确认 `uxrce_dds_client` 绑定 `/dev/ttyS7`。

> 如需进一步调整波特率或串口角色，只需修改 `SER_*_BAUD` 与 `UXRCE_DDS_CFG` / `MAV_*_CONFIG` 等参数，无需动核心 PX4 代码。***
