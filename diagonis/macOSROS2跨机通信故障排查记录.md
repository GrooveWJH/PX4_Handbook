# macOS ROS 2 跨机通信故障排查记录 (2025-12-17)

## 故障概述
macOS (RoboStack) 与 Linux (Ubuntu) 在同一局域网下，macOS 能够发现 Linux 节点（`ros2 topic list` 可见），且能通过组播测试，但无法接收 Linux 发布的话题数据（`ros2 topic echo` 无响应）。

## PlantUML 排查流程图

```plantuml
@startuml
scale 1.5
skinparam backgroundColor #FFFFFF
skinparam handwriting false
skinparam shadowing false
skinparam defaultFontName "WenQuanYi Zen Hei"

skinparam activity {
  BackgroundColor #E6F7FF
  BorderColor #1890FF
  ArrowColor #1890FF
  FontSize 14
}
skinparam note {
  BackgroundColor #FFF1F0
  BorderColor #F5222D
}

title macOS ROS 2 跨机通信故障排查与修复流程

start

partition "1. 现象确认 (Symptoms)" {
    :macOS (RoboStack) 环境准备就绪;
    :局域网内 Linux 机器发布 ROS 2 话题;

    if (执行 ros2 multicast receive) then (收到 'Hello World')
        note right
            说明物理链路、路由、
            及底层组播(Multicast)正常
        end note
    else (收不到)
        :检查路由/防火墙组播设置;
        stop
    endif

    :执行 ros2 topic list;
    note right
        能看到 Linux 发布的话题名
        (发现协议工作正常)
    end note

    :执行 ros2 topic echo /topic;
    #Pink:无数据 (阻塞);
    note right
        **核心问题**:
        能发现节点，但收不到数据内容
    end note
}

partition "2. 诊断排查 (Diagnosis)" {
    :检查 ROS_DOMAIN_ID;
    note right: 双方均未设置或一致 (Pass) end note

    :检查网络代理 (Clash);
    note right
        发现开启了 TUN 模式
        怀疑虚拟网卡劫持流量
    end note
    :关闭 Clash / 设置直连规则;
    :测试结果: **问题依旧**;

    :进行**单向通信测试** (关键步骤);
    split
        :Mac 发布 -> Linux 订阅;
        #LightGreen:Linux 成功收到数据;
    split again
        :Linux 发布 -> Mac 订阅;
        #Pink:Mac 无法收到数据;
    end split
    note bottom
        结论: 单向通信故障 (Linux->Mac 入站失败)
    end note

    :检查 RMW 中间件实现;
    :echo $RMW_IMPLEMENTATION;
    :结果: 空 (默认使用 **rmw_fastrtps_cpp**);
}

partition "3. 根因锁定 (Root Cause)" {
    #Orange:锁定元凶: FastDDS (rmw_fastrtps_cpp);
    note right
        **原因分析**:
        FastDDS 在 macOS 上存在已知缺陷。
        经常错误地将数据接收端绑定到
        Loopback (127.0.0.1) 或错误的网卡，
        导致无法接收外部发回的 UDP 单播包。
    end note

    :尝试方案 B: 配置 FastDDS XML 强制指定 IP;
    :测试结果: 依然不稳定/失败 (放弃);
}

partition "4. 最终解决方案 (Fix)" {
    #LightGreen:切换至 CycloneDDS (rmw_cyclonedds_cpp);

    fork
        :macOS 端 (RoboStack);
        :mamba install ros-humble-rmw-cyclonedds-cpp;
        :echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.zshrc;
    fork again
        :Linux 端 (Ubuntu);
        :sudo apt install ros-humble-rmw-cyclonedds-cpp;
        :echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc;
        note right
            **注意**: Systemd 自启服务
            需在 .service 文件中添加
            Environment="RMW_IMPLEMENTATION=..."
        end note
    end fork

    :重启双方 ROS Daemon 及节点;
}

partition "5. 验证 (Verification)" {
    :ros2 topic echo /topic;
    #LightGreen:成功接收数据!;
    :通信完全恢复;
}

stop
@enduml
