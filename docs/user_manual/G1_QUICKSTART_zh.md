# G1 一页启动教程

> 这是一份最短路径的 G1 真机启动教程。目标只有一个：把 Watchdog 和 Agent 起起来，并在真机上执行最基本的前进、转向和停止。

## 1. 先确认你已经有这两个环境

Agent 环境：

```bash
conda activate phyagentos
```

G1 真机环境：

```bash
conda activate phyagentos-go2-py39
```

如果第二个环境名看起来像 Go2，不用管。当前它同时承载 Go2 和 G1 的 Unitree SDK 依赖。

## 2. 先改配置文件

打开这个文件：

- [g1_unitree_sdk_driver_config.json](../../examples/g1_unitree_sdk_driver_config.json)

最少确认这两个字段：

```json
{
  "unitree_sdk_path": "/home/raychen/桌面/phyAgentOS/unitree_sdk2_python",
  "network_interface": "你的真实网卡名"
}
```

如果你不知道网卡名，先执行：

```bash
ip link
```

## 3. 启动 Watchdog

终端 1：

```bash
conda activate phyagentos-go2-py39
cd ~/桌面/phyAgentOS/PhyAgentOS
python -m hal.hal_watchdog --driver g1 --driver-config examples/g1_unitree_sdk_driver_config.json
```

如果启动正常，你会看到：

- `Driver    : g1`
- `Watching ACTION.md ...`

## 4. 启动 Agent

终端 2：

```bash
conda activate phyagentos
cd ~/桌面/phyAgentOS/PhyAgentOS
paos agent
```

## 5. 先做最小测试

进入 `paos agent` 后，建议按这个顺序测：

```text
check connection
move forward 0.2 meters
turn left 45 degrees
stop
```

如果你想先做姿态测试，再试：

```text
high stand
wave hand
```

## 6. 现在能用哪些动作

简单运动：

- `move forward 0.2 meters`
- `move backward 0.1 meters`
- `turn left 45 degrees`
- `turn right 90 degrees`
- `turn around`
- `stop`

姿态与演示动作：

- `high stand`
- `low stand`
- `wave hand`
- `shake hand`

## 7. 机器人不动时先查这三件事

1. Watchdog 是否真的是这样启动的：

```bash
python -m hal.hal_watchdog --driver g1 --driver-config examples/g1_unitree_sdk_driver_config.json
```

2. 配置里的 `network_interface` 是否正确。

3. [ENVIRONMENT.md](/home/raychen/.PhyAgentOS/workspace/ENVIRONMENT.md) 里 `robots.g1_001.connection_state.status` 是否已经变成 `connected`。

## 8. 这份教程之外

如果你要看完整说明，包括动作边界、双 conda 环境原因和排查建议，继续看：

- [G1_REAL_HARDWARE_zh.md](../../docs/user_manual/G1_REAL_HARDWARE_zh.md)
