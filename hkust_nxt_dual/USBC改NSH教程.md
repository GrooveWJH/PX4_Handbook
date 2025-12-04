## hkust_nxt-dual：USB-C 改 NSH 教程

说明如何把默认的 USB CDC（原本用于 MAVLink）改成直接进入 NSH Shell。

### 1. 修改 NuttX defconfig
文件：`boards/hkust/nxt-dual/nuttx-config/nsh/defconfig`

1. 打开 NuttX console 与 USB CDC 的绑定：
   ```diff
   + CONFIG_NSH_USBCONSOLE=y
   + CONFIG_CDCACM_CONSOLE=y
   ```
2. 关闭原串口控制台（USART6，`/dev/ttyS5`）：
   ```diff
   - CONFIG_USART6_SERIAL_CONSOLE=y
   + # CONFIG_USART6_SERIAL_CONSOLE is not set
   ```
效果：系统控制台 → USB CDC；NSH 也随之走 USB。

### 2. 调整 PX4 板级默认参数
文件：`boards/hkust/nxt-dual/init/rc.board_defaults`

新增（防止 PX4 在 USB 上自动启用 MAVLink）：
```sh
param set-default MAV_0_CONFIG 0
param set-default MAV_1_CONFIG 0
param set-default MAV_2_CONFIG 0
```
这样 USB CDC 就不会被 MAVLink 抢占，只留下 NSH。

### 3. 重新编译 & 刷写
```bash
make hkust_nxt-dual_default
make hkust_nxt-dual_default upload    # 或 QGC 刷机
```

### 4. 使用
1. 连接 USB-C。
2. `screen /dev/ttyACM0 57600`（波特率数值无关紧要，只是工具要求）。  
3. 按一次回车即可看到 `nsh>`（如果保留 `CONFIG_DRIVERS_CDCACM_AUTOSTART`，需要连按 3 次回车触发 `nshterm`）。

### 5. 可选调整
- 若需要继续使用 `cdcacm_autostart` 的自动检测，可以把 `CONFIG_DRIVERS_CDCACM_AUTOSTART=y`，此时三次回车触发 NSH，发送 MAVLink HEARTBEAT 则触发 MAVLink。
- 若希望 USB 口固定为 NSH，可保持 `CONFIG_DRIVERS_CDCACM_AUTOSTART=n`，并在 `rc.board_extras` 中加入 `nshterm /dev/ttyACM0 &`（或直接依赖 console 绑定）。

完成上述修改后，USB-C 将成为 NSH 控制台，可直接通过 `screen`、`picocom` 等工具调试，同时 Bootloader 刷机流程不受影响。***
