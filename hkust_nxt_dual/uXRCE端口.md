## HKUST NXT-Dual 上的 uXRCE-DDS 串口选择

本文说明当前固件默认如何在 HKUST NXT-Dual 上部署 uXRCE-DDS，并解释为什么选择 UART4（TELEM2）作为标准接口。

### 1. 板载串口概览
- `.px4board` 别名：`TELEM1=/dev/ttyS1`（USART2）、`TELEM2=/dev/ttyS3`（UART4）、`TELEM3=/dev/ttyS6`（UART7）、`TELEM4=/dev/ttyS7`（UART8）。
- `defconfig` 已启用 UART4/7/8，并允许运行期通过参数设置波特率；板级无以太网 PHY，因此 uXRCE-DDS 只能选择串口角色（`UXRCE_DDS_CFG=101~104`）。
- `rc.board_defaults`：`MAV_0_CONFIG=301`（USB CDC MAVLink）、`UXRCE_DDS_CFG=102`（TELEM2）、`SER_TEL2_BAUD=115200`；`SENS_TFMINI_CFG` 复位为 0，默认不再占用 TELEM2。

### 2. 默认：TELEM2 (`/dev/ttyS3`，UART4) 运行 uXRCE
- 参数 `UXRCE_DDS_CFG=102` → `uxrce_dds_client` 自动绑定 TELEM2。
- UART4 物理脚位 PB9（TX）/PB8（RX），在板子丝印为 UART4/TELEM2，线束常用于伴飞/高速串口。
- `SER_TEL2_BAUD=115200`。如需更高带宽，可在参数中直接修改，Agent 需保持一致。

### 3. TELEM4（UART8）恢复空闲
- `UXRCE_DDS_CFG` 不再指向 TELEM4，`SER_TEL4_BAUD` 也被移除，UART8 默认为空闲接口，可按需给调试/雷达使用。
- 若想改回 UART8，可在参数里把 `UXRCE_DDS_CFG` 改为 104，再视需要设置 `SER_TEL4_BAUD`；但需确保 UART8 线束与伴飞连接。

### 4. 用户操作步骤
1. 将 ROS2 Agent/伴飞电脑的串口接到 UART4 (PB9/PB8)。
2. QGC 参数确认/修改：
   - `UXRCE_DDS_CFG = 102`
   - `SER_TEL2_BAUD = <目标波特率>`
   - 若 Agent 需要特定命名空间/密钥，可一并设置 `UXRCE_DDS_NS_IDX`、`UXRCE_DDS_KEY` 等。
3. 伴飞端运行 `MicroXRCEAgent serial --dev /dev/ttyUSBx -b <同样的 baud>`。
4. 在 NSH 执行 `uxrce_dds_client status`，确认其绑定 `Serial type @ /dev/ttyS3`。

> 调试 tip：若 `uxrce_dds_client status` 显示 `not running`，可手动执行 `uxrce_dds_client start -t serial -d /dev/ttyS3 -b <baud>` 验证线路，再回到参数模式。

### 5. FAQ
- **还能改回 TELEM4 吗？** 可以，将 `UXRCE_DDS_CFG` 改为 104 并重启即可，但那时就需要把伴飞改接 UART8。
- **TELEM1 是干什么的？** `CONFIG_USART2_SERIAL_CONSOLE=y`，所以 `/dev/ttyS1`（TELEM1）已作为 115200 波特的 NuttX/NSH 控制台，串口调试线需接在 USART2 上。
- **USB CDC 会影响 uXRCE 吗？** 不会，USB (`/dev/ttyACM0`) 仍用于 MAVLink/QGC/刷机；uXRCE 与之独立，走 UART4。

综上，新的默认配置让 TELEM1→NSH、TELEM2→uXRCE-DDS、TELEM4 空闲待用；既能在 NSH 直接检查 uXRCE 状态，又让 USB-C 继续跑 MAVLink/QGC。***
