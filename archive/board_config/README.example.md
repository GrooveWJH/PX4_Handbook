# <Board Name> — Snapshot #<N>

- **来源目录**：`boards/<vendor>/<board>`
- **快照时间**：YYYY-MM-DD
- **变更简介**：一句话描述为何抓取本快照（例如“USB 切换为 NSH”）。

## 串口/接口映射

| 接口 | NuttX 设备 | 角色/用途 | 波特率 | 关键参数/宏 |
| --- | --- | --- | --- | --- |
| USB-C | `/dev/ttyACM0` | | | |
| TELEM1 | `/dev/ttyS?` | | | |
| ... | | | | |

> 备注：波特率建议写 PX4 参数名（如 `SER_TEL1_BAUD=115200`）以便回滚。

## 关键文件概览

- `default.px4board`：列出本快照与默认配置的主要差异（串口别名、启用模块等）。
- `nuttx-config/nsh/defconfig`：说明控制台、CDC、UART 相关的开关。
- `init/rc.board_defaults`：列出重要的 `param set-default` 修改，如 `MAV_*`、`SER_*`、`SENS_*`。
- 其他：若修改 `extras/`、`src/` 等，也在此简要记录。

## 额外说明（可选）

- 与上一个快照相比的目的/差异。
- 对 ROS2 / uXRCE / 传感器等额外步骤的提示。
- 恢复方法（例如“复原时覆盖整个 `boards/<vendor>` 目录”）。
