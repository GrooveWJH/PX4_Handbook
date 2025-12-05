## HKUST NXT-Dual 上的 uXRCE-DDS 串口选择

> 快照参考：`archive/board_config/hkust#2`（2025-12-05）。如需了解其它版本的默认口，请查看对应快照；本文重点讲“怎样选口、怎样验证”，而不是追踪每次改动。

### 1. `UXRCE_DDS_CFG` 映射规则

PX4 把常用串口封装为“接口槽位”，并用枚举的方式填写参数：

| 参数值 | 说明 | 典型 `/dev/ttyS*` | 备注 |
| --- | --- | --- | --- |
| 101 | TELEM1 | `/dev/ttyS1` | `.px4board` 决定 TELEM1 的真实 UART |
| 102 | TELEM2 | `/dev/ttyS3` | 常用作测距仪或副链路 |
| 103 | TELEM3 | `/dev/ttyS6` | 备用串口 |
| 104 | TELEM4 | `/dev/ttyS7` | 板上 Debug/UART8 |
| 0 | 关闭 | — | 不启动 uXRCE |

因此，`UXRCE_DDS_CFG` 仅仅是“选择哪一个 TELEM 槽位”，剩下的波特率、脚位都由 `.px4board` 与 `SER_TEL*_BAUD` 控制。若板厂改动了 `default.px4board`，直接查快照即可知道槽位→串口的映射。

### 2. 快照 #2 的默认安排
- `.px4board`：`TELEM1=/dev/ttyS1`（USART2）、`TELEM2=/dev/ttyS3`（UART4）…… 详见 `串口映射.md`。
- `defconfig`：USB CDC 直连 NSH（`CONFIG_NSH_USBCONSOLE=y`），USART2/4/7/8 均启用。
- `rc.board_defaults`：`UXRCE_DDS_CFG=101`（即 TELEM1），`SER_TEL1_BAUD=921600`，`SENS_TFMINI_CFG=102` 把 TELEM2 交给 TFmini。

### 3. 默认：TELEM1 (`/dev/ttyS1`) 运行 uXRCE
- `UXRCE_DDS_CFG=101` → `uxrce_dds_client` 绑定 TELEM1。
- 波特率由 `SER_TEL1_BAUD` 控制（快照 #2 设为 921600），伴飞端需一致。
- 需要换口时，更改 `UXRCE_DDS_CFG` 即可，无需改源码；但要记得释放该串口（例如 TFmini、MAVLink 等要先停掉）。

### 4. TELEM2（UART4）用于串行测距仪
- `SENS_TFMINI_CFG=102` 会在 `/dev/ttyS3` 自动起 TFmini 驱动；若要把 uXRCE 挪到 TELEM2，需要先把该参数改为 0。
- 同理，其他串口也是“谁先占用谁”，请通过参数管理好各模块的串口需求。

### 5. 用户操作步骤
1. 将 ROS2 Agent/伴飞电脑的串口接到 UART2 (PD5/PD6)。
2. QGC 参数确认/修改：
   - `UXRCE_DDS_CFG = 101`
   - `SER_TEL1_BAUD = 921600`（或你设定的速率）
   - 若 Agent 需要特定命名空间/密钥，可一并设置 `UXRCE_DDS_NS_IDX`、`UXRCE_DDS_KEY` 等。
3. 伴飞端运行 `MicroXRCEAgent serial --dev /dev/ttyUSBx -b <同样的 baud>`。
4. 在 NSH 执行 `uxrce_dds_client status`，确认其绑定 `Serial type @ /dev/ttyS1`。

> 调试 tip：若 `uxrce_dds_client status` 显示 `not running`，可手动执行 `uxrce_dds_client start -t serial -d /dev/ttyS1 -b <baud>` 验证线路，再回到参数模式。

### 6. FAQ
- **还能改用其它串口吗？** 可以，将 `UXRCE_DDS_CFG` 改为 102/103/104 并同步连线即可；注意避开 TFmini 或其它串口外设。
- **NSH 在哪里？** `CONFIG_NSH_USBCONSOLE=y`，NSH 通过 USB-C → `/dev/ttyACM0` 输出，连接后连续敲 3 次回车即可获得 `nsh>`。
- **既然 USB 是 NSH，还能刷机吗？** 可以，PX4 bootloader 仍在 USB CDC 上枚举，刷写流程不受影响。

综上，通过“参数选择槽位 + `.px4board` 提供别名 + 快照记录实际 wiring”，就能在 HKUST 板上自由切换 uXRCE 所用的串口，同时保持手册内容长期稳定。***
