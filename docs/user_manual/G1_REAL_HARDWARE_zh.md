# G1 真机启动与操作手册

> 本文档说明如何在当前仓库中启动 Unitree G1 真机控制链，并列出当前已经接通的简单原语。

## 1. 适用范围

本文档适用于以下运行方式：

- Agent 运行在 `phyagentos` conda 环境
- G1 真机 Watchdog 运行在 `phyagentos-go2-py39` conda 环境
- G1 控制通过 `unitree_sdk2_python` 的 `LocoClient` 接入真实机器人

当前文档不包含：

- G1 视觉感知
- G1 手臂精细操作
- 低层关节控制开发

## 2. 为什么仍然建议两个 conda 环境

当前仓库仍建议保留两个环境：

- `phyagentos`
  - 用于运行 `paos agent`
  - Python 建议 `3.11`
- `phyagentos-go2-py39`
  - 用于运行 G1 真机 Watchdog
  - 当前本机验证通过的 `unitree_sdk2_python + cyclonedds` 组合在 `Python 3.9`

虽然这个环境名是为 Go2 留下来的，但里面的 Unitree SDK 依赖同样适用于 G1 的 `LocoClient`。

## 3. 依赖文件

仓库根目录已有两份 requirements：

- [requirements-phyagentos.txt](../../requirements-phyagentos.txt)
- [requirements-go2-py39.txt](../../requirements-go2-py39.txt)

安装方式：

```bash
cd ~/桌面/phyAgentOS/PhyAgentOS

conda create -n phyagentos python=3.11 -y
conda activate phyagentos
pip install -r requirements-phyagentos.txt

conda create -n phyagentos-go2-py39 python=3.9 -y
conda activate phyagentos-go2-py39
pip install -r requirements-go2-py39.txt
```

## 4. 启动前检查

启动前至少确认：

1. G1 已上电且网络可达
2. 电脑接到了正确网卡
3. [g1_unitree_sdk_driver_config.json](../../examples/g1_unitree_sdk_driver_config.json) 中的 `network_interface` 已改成真实网卡名
4. 机器人前后左右都有足够安全空间
5. 现场有人可随时接管和急停

查看本机网卡名：

```bash
ip link
```

## 5. 配置文件

当前 G1 真机配置文件是：

- [g1_unitree_sdk_driver_config.json](../../examples/g1_unitree_sdk_driver_config.json)

关键字段：

- `unitree_sdk_path`
- `network_interface`
- `stand_up_on_connect`
- `linear_speed_mps`
- `angular_speed_rps`
- `command_dt_s`

最少要先改：

```json
{
  "network_interface": "你的真实网卡名",
  "stand_up_on_connect": false
}
```

如果你希望 Watchdog 连上后自动做一次 `Damp -> Squat2StandUp`，可以把 `stand_up_on_connect` 改成 `true`。

## 6. 真机启动步骤

### 6.1 终端 1：启动 G1 Watchdog

```bash
conda activate phyagentos-go2-py39
cd ~/桌面/phyAgentOS/PhyAgentOS
python -m hal.hal_watchdog --driver g1 --driver-config examples/g1_unitree_sdk_driver_config.json
```

这一步会：

- 启动 `g1` driver
- 初始化 `unitree_sdk2_python`
- 初始化 `ChannelFactoryInitialize`
- 创建 `LocoClient`
- 持续把连接状态和运动状态写回 `~/.PhyAgentOS/workspace/ENVIRONMENT.md`

### 6.2 终端 2：启动 Agent

```bash
conda activate phyagentos
cd ~/桌面/phyAgentOS/PhyAgentOS
paos agent
```

## 7. 当前可直接使用的动作原语

### 7.1 连接管理

支持：

- `connect_robot`
- `disconnect_robot`
- `reconnect_robot`
- `check_connection`

### 7.2 简单运动

当前已经接通并适合真机小范围验证的原语：

- `forward`
- `backward`
- `turn_left`
- `turn_right`
- `turn_around`
- `stop`

可在 Agent 中这样表达：

```text
move forward 0.3 meters
move backward 0.2 meters
turn left 45 degrees
turn right 90 degrees
turn around
stop
```

这些动作会映射到 G1 的 `LocoClient.Move()` 和 `LocoClient.StopMove()`。

### 7.3 姿态与演示动作

当前 driver 还支持这些直接动作：

- `damp`
- `squat_to_stand`
- `stand_to_squat`
- `lie_to_stand`
- `low_stand`
- `high_stand`
- `zero_torque`
- `wave_hand`
- `shake_hand`

如果模型没有主动选中这些动作，你也可以让它明确执行对应动作名。

## 8. 重要边界

当前 G1 driver 已经接通的是：

- Unitree SDK 2 Python 的 `LocoClient`
- 简单运动原语
- 基础姿态切换动作

当前还没有接通的是：

- 真实视觉输入
- 真实 odometry 回传
- 手臂精细任务控制
- 自主导航栈

所以现在最合适的实验目标是：

- 站起
- 小步前进或后退
- 原地左右转
- 停止
- 演示性姿态动作

## 9. 建议的最小实验流程

建议按这个顺序验证：

1. 启动终端 1 的 Watchdog
2. 启动终端 2 的 Agent
3. 执行 `check connection`
4. 执行 `move forward 0.2 meters`
5. 执行 `turn left 45 degrees`
6. 执行 `stop`

稳定后再尝试：

7. `high stand`
8. `wave hand`

## 10. 排查建议

### 10.1 Watchdog 无法连接

重点检查：

- `network_interface` 是否正确
- `unitree_sdk2_python` 是否能导入
- `cyclonedds` 是否已正常安装
- 电脑和 G1 是否在正确网络链路上

### 10.2 Agent 能回复但机器人不动

重点检查：

- Watchdog 是否真的用的是 `--driver g1`
- `ENVIRONMENT.md` 中 `robots.g1_001.connection_state.status` 是否为 `connected`
- 机器人当前姿态是否允许执行移动命令
- 现场是否存在安全保护或人工接管状态
