## TFmini 驱动与数据流

### 1. 文件位置
TFmini 驱动位于 `src/drivers/distance_sensor/tfmini/`，主要文件：
- `tfmini_main.cpp`：实现 shell 命令 `tfmini start/stop/status`，负责解析参数并创建驱动实例。
- `TFMINI.cpp/.hpp`：核心驱动类，负责串口打开、调度、解析数据并发布 uORB 话题。
- `tfmini_parser.cpp/.h`：实现 TFmini 协议解析状态机（识别 `0x59 0x59` 帧头、计算校验和值，输出距离）。
- `module.yaml`：定义 `SENS_TFMINI_CFG` 参数及串口自动启动命令；我们在 `rc.board_defaults` 中把该参数设为 102，使 TELEM2 开机即启动 TFmini。

### 2. 驱动流程
1. `tfmini start -d /dev/ttyS3` 或 `SENS_TFMINI_CFG=102` 触发 `tfmini_main` 创建 `TFMINI` 对象。
2. `TFMINI::init()`
   - 读取 `SENS_TFMINI_HW`，配置量程/FoV。
   - 打开串口 `/dev/ttySx`，设置 115200 8N1、非规范模式。
   - 在 WorkQueue 上调度 `Run()`，以 ~7ms 周期轮询。
3. `TFMINI::Run()` 每次尝试读取串口数据，调用 `collect()`。
   - `collect()` 读取可用字节，逐个传给 `tfmini_parse()`。
   - 解析器识别帧头 `0x59 0x59`，按协议提取距离（单位厘米）并换算为米。
   - 一旦得到合法距离，调用 `_px4_rangefinder.update(timestamp, distance_m)`，在 uORB 上发布 `distance_sensor`。
4. 其它模块通过订阅 `distance_sensor`（例如地面效应、避障、姿态估计）获取数据；你可以在 NSH 中 `listener distance_sensor` 看到实时输出。

### 3. 数据流示意
```plantuml
@startuml
title TFmini 数据流
node "TFmini 硬件" as HW
device "USART4 (TELEM2)" as UART
device "TFMINI Driver" as Driver
rectangle "tfmini_parser" as Parser
rectangle "uORB: distance_sensor" as UORB
node "消费者 (EKF/避障/日志)" as Consumers

HW --> UART : 115200/8N1 帧
UART --> Driver : read()
Driver --> Parser : 字节流
Parser --> Driver : 距离(m)
Driver --> UORB : _px4_rangefinder.update()
UORB --> Consumers : 订阅 distance_sensor
@enduml
```

### 4. 参数与自动化
- `SENS_TFMINI_CFG`：串口角色选择（101=TELEM1, 102=TELEM2 ...）。我们在 HKUST 板默认设为 102，传感器接 TELEM2 即可上电自启。
- `SENS_TFMINI_HW`：硬件型号，决定量程/最小距离。默认 1（12m 版本），可在 QGC 参数里调整。
- 波特率/引脚由 `SER_TEL2_BAUD` 和 `default.px4board` 控制，该组合在板级配置里已预设为 115200。

总结：TFmini 驱动通过串口采集激光测距数据，经过状态机解析后发布 `distance_sensor` uORB 话题。我们在板级参数中预设 `SENS_TFMINI_CFG=102`，让 TELEM2 插入雷达后自动运行，用户只需 `listener distance_sensor` 即可验证数据。EOF