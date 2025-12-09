# ROS2 External Controller 与 ROS1 MAVROS 控制路径对比

本文整理最近的讨论，说明 PX4 1.14+ 引入的 **External Controller** 与传统 **ROS1/MAVROS Offboard** 控制之间的“真正差异”与适用场景。核心结论：MAVROS 也能覆盖（override）任意层的 setpoint，但 PX4 内部控制器仍在运行；External Controller 则提供了固件级的“停用该控制器”机制，并在 uXRCE-DDS/ROS2 中给出了更加精确的接口与安全约束调用方式。

## 1. 控制链对照

| 环节 | MAVROS Offboard | ROS2 External Controller |
| --- | --- | --- |
| 数据接口 | MAVLink `SET_*`/Topic `/mavros/setpoint_*` | `/fmu/in/*` uXRCE-DDS (ROS2) |
| 典型做法 | 提供 setpoint（位置、速度、姿态、角速度、推力、执行器） | 发布控制量（trajectory、attitude、rates、thrust、actuator 等） |
| PX4 内部控制器 | **始终运行**：即便 setpoint 属于内层，PX4 的 PID/限幅仍执行 | 可通过 `EXT_CTRL_*`、`PX4_CTRL_*` 参数 **停用某级控制器** |
| 接管粒度 | 功能上覆盖所有层（最底层可用 `/mavros/motor_control/setpoint`） | 官方提供“按层关闭”机制，可只接管速度/姿态/速率等 |
| 安全与约束 | 仍依赖 MAVLink/MAVROS，直接写电机时需自担风险 | PX4 保留 failsafe、混控、输出限幅等安全逻辑 |
| ROS 版本依赖 | ROS1 + MAVROS（Nachül） | ROS2 + PX4 uXRCE-DDS（FastDDS/RMW） |

## 2. “覆盖” vs “停用” 的差别

1. **MAVROS 只能覆盖 setpoint**  
   - 你可以用自己的位置/速度控制算法，将结果转换为加速度/姿态 setpoint，通过 `/mavros/setpoint_raw/local` 发送。  
   - PX4 仍会运行自身的姿态/角速度控制器，对这些 setpoint 做 PID、限幅、混控，然后输出电机。  
   - 如果想彻底越过所有控制器，只能用 `/mavros/setpoint_actuator_control` 或 `/mavros/motor_control/setpoint`，这意味着放弃 PX4 侧的安全逻辑。

2. **External Controller 提供“停用某级控制器”的开关**  
   - PX4 1.14+ 在 `Commander`/`FlightTask` 中加入 External Controller 模式，通过参数（如 `EXT_CTRL`、`PX4_CTRL`）通知固件：某一级控制器由外部接管。  
   - 对应级别的内环 PID 不再运行，PX4 只是把 uXRCE 来的控制量送进 Mixer，并保持 failsafe/约束。  
   - 外部算法只接管需要的层次（例如速度控制器），其它层仍由 PX4 接管，大大降低开发者负担。

## 3. ROS2 External Controller 的优势

1. **固件级支持**  
   - PX4 主动停用被接管的控制器，避免“内部 PID 仍在运行”带来的耦合或双闭环冲突。  
   - 官方参数与状态机更清晰：切换模式、更名 External Control、日志/事件记录都有配套支持。

2. **uXRCE-DDS 统一桥接**  
   - `/fmu/in/out/*` 话题覆盖了 PX4 Control Pipeline 所有层级（trajectory、attitude、rates、thrust、actuator），并且 QoS 明确。  
   - ROS2 侧可直接用官方 `px4_msgs`，无需自己翻译 MAVLink。

3. **安全与约束保留**  
   - 即使停用姿态/速率控制器，PX4 仍然保留混控、failsafe、输出限幅等“最后一道防线”。  
   - 外部算法只需关注控制逻辑，而不必重写所有安全功能。

4. **对齐未来特性**  
   - External Controller 是 PX4 新增的标准扩展点，后续的控制器插件、External Mode 工具都会围绕这套接口。  
   - ROS2/uXRCE-DDS 也是 PX4 官方长期支持的通信路径，在多车、Namespace、QoS 等方面更灵活。

## 4. 何时选 MAVROS？何时选 External Controller？

| 场景 | 推荐方案 |
| --- | --- |
| 需要快速集成，外部算法只覆盖位置/速度 setpoint | ROS1 + MAVROS Offboard 仍然高效 |
| 想重写或替换 PX4 某一级控制器，并保留 PX4 安全/混控 | ROS2 + External Controller |
| 需要完全绕过 PX4，一切控制/Safety 由外部承担 | MAVROS `/motor_control` 或 PX4 自定义固件 |
| 多机/命名空间、QoS 精细管理 | ROS2 + uXRCE 更方便 |

## 5. 结论

MAVROS Offboard 与 External Controller 在“功能”上可以互相模拟——你总能通过更底层的 setpoint 去覆盖 PX4 的控制器。但 External Controller 的价值在于固件级配合：PX4 认可你接管的层级，关闭对应 PID，同时保留安全约束与混控。这样既给伴飞足够自由，也避免了“内环仍在运行”的潜在冲突，是 PX4 面向 ROS2/伴飞计算的更现代方案。
