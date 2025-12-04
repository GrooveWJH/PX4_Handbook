## PX4 与 NuttX：板级适配与解耦

PX4 在 NuttX 上构建，利用其 RTOS/驱动体系实现对大量芯片和开发板的兼容。本文概述 PX4 如何借助 NuttX 将核心飞控逻辑与硬件适配层解耦。

### 1. 角色划分

```plantuml
@startuml
skinparam componentStyle rectangle

rectangle "PX4 Core\n(Firmware)" as PX4Core {
  component "Modules\n(控制/算法/MAVLink)" as Modules
  component "Drivers (PX4層)\n传感器/外设" as PX4Drivers
  component "Board Scripts\n(rcS, rc.board_*)" as Scripts
}

rectangle "PX4 Board Config" as PX4Board {
  file ".px4board" as Px4board
  file "init/rc.board_*" as BoardScripts
  file "nuttx-config/**" as NuttxDefconfig
}

rectangle "NuttX RTOS" as Nuttx {
  component "Kernel/Task" as Kernel
  component "HAL Drivers\n(UART, SPI, USB...)" as NuttxDrivers
}

rectangle "Hardware" as HW {
  component "MCU\n(STM32, NXP...)" as MCU
  component "Peripherals\n(UART, SPI, ADC)" as Peripherals
}

PX4Core <--> PX4Board : 读取脚本\n& 回写串口参数
PX4Core -right-> Nuttx : 调用系统 API
PX4Drivers -right-> NuttxDrivers : 通过 NuttX HAL
Nuttx -down-> HW : 驱动芯片外设
PX4Board -right-> Nuttx : 生成 defconfig / board.h
@enduml
```

### 2. NuttX 层的板级配置
- `nuttx-config/nsh/defconfig`：指定 MCU 型号、启用哪些 UART/SPI/I2C、控制台端口、内存分区等。
- `nuttx-config/include/board.h`：定义引脚复用、GPIO、PWM 通道等。
- `nuttx-config/scripts/*.ld`：链接脚本，决定 Flash/SRAM 布局。
- 通过 `make px4_<board>_default menuconfig` 生成/调整，PX4 再用这些配置编译 NuttX 内核与基础驱动。

### 3. PX4 层的板适配
- `.px4board`：声明串口别名（`TELEM1=/dev/ttyS1`）、启用哪些 PX4 模块、MAVLink 实例、驱动等。CMake 会读取该文件为 `nuttx_px4_board.cmake` 生成编译宏。
- `init/rc.board_defaults` / `rc.board_sensors` / `rc.board_extras`：Shell 脚本，用于设置默认参数（电池、MAVLink、uXRCE 等）、启动板载传感器/服务，运行于 `rcS` 框架内。
- `src/boards/<vendor>/<board>/`：板级 C/C++ 代码（GPIO、SPI、FLASH、USB 等）的 PX4 封装，调用 NuttX API。

```plantuml
@startuml
!define RECTANGLE class
PX4Core --> "Modules (MAVLink, 控制, 任务)" : 使用参数/串口别名
PX4Core --> "Drivers (PX4层)" : 通过 PX4 HAL
"Drivers (PX4层)" --> "NuttX Drivers" : ioctl/read/write

PX4Board --> "NuttX defconfig" : 生成/调整 (menuconfig)
PX4Board --> "rc.board_defaults" : 设置参数/MAVLink

"rcS 主脚本" -> "rc.board_defaults"
"rcS 主脚本" -> "rc.board_sensors"
"rcS 主脚本" -> "rc.board_extras"

"NuttX defconfig" --> "NuttX Kernel"
"board.h" --> "PX4 Board 源码"
@enduml
```

### 4. 串口/接口映射示例
- `.px4board`：
  ```text
  CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS1"
  CONFIG_BOARD_SERIAL_TEL2="/dev/ttyS3"
  ```
- `nuttx-config/nsh/old_defconfig.txt`：
  ```text
  CONFIG_STM32H7_USART2=y  # ttyS1
  CONFIG_STM32H7_UART4=y   # ttyS3
  ```
- `rc.board_defaults`：
  ```sh
  param set-default MAV_0_CONFIG 301
  param set-default UXRCE_DDS_CFG 102
  ```
→ TELEM1 (USART2) 作为控制台或 MAVLink，TELEM2 (UART4) 作为 uXRCE，USB CDC 则对应 `MAV_0_CONFIG=301`。

### 5. 可插拔扩展
- 更换 MCU/Board：只需新增/修改 `nuttx-config`、`board.h`、`src/boards/...`；PX4 Modules 无需改动。
- 更换接口角色：通过 `.px4board` 和 `rc.board_defaults` 映射，压根不需要动核心 C++ 代码。
- 新增板级功能（如 USB 复合设备、外部传感器）：在 `src/boards/<board>` 中实现驱动或在脚本中配置，就能与 PX4 主逻辑配合。

### 6. PlantUML：配置流与启动顺序
```plantuml
@startuml
skinparam linetype ortho

rectangle PX4_Build {
  file ".px4board"
  file "nuttx-config/**"
  file "init/rc.board_*"
}

rectangle Build_Steps {
  node "CMake" as CMake
  node "NuttX Build" as NBuild
  node "PX4 Build" as PBuild
}

rectangle Runtime {
  rectangle "rcS" as rcS
  rectangle "rc.board_defaults"
  rectangle "rc.board_sensors"
  rectangle "rc.board_extras"
  rectangle "Modules"
}

".px4board" --> CMake
"nuttx-config/**" --> NBuild
CMake --> NBuild
CMake --> PBuild
NBuild --> PBuild

PBuild --> rcS
rcS --> "rc.board_defaults"
rcS --> "rc.board_sensors"
rcS --> "rc.board_extras"
"rc.board_defaults" --> Modules : 设置参数/启动服务
"rc.board_sensors" --> Modules : 启动传感器驱动
"rc.board_extras" --> Modules : 额外脚本
@enduml
```

### 总结
PX4 通过 NuttX 的板级配置和 shell 脚本，实现了“硬件适配层”与“飞控核心”解耦：NuttX 负责 MCU 外设，`.px4board`+`rc.board_*` 决定串口映射、驱动启停，核心模块只需按别名/参数运行。这种分层设计让同一套飞控算法可以运行在大量不同芯片和板卡上，只需提供相应的配置与板级驱动即可。
