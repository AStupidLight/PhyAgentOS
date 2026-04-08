# Go2 真机启动与操作手册

> 本文档针对当前仓库里的双环境方案，说明如何在真机上启动 Go2、Agent 与 Watchdog，并整理当前可直接使用的动作原语。

## 1. 适用范围

本文档适用于以下运行方式：

- Agent 运行在 `phyagentos` conda 环境
- Go2 真机 Watchdog 运行在 `phyagentos-go2-py39` conda 环境
- Go2 控制通过 `unitree_sdk2_python` 接入真实机器人

本文档不适用于：

- `unitree_mujoco` 仿真
- 机械臂抓取
- 楼梯步态控制器开发

## 2. 为什么需要两个 conda 环境

当前仓库建议保留两个环境：

- `phyagentos`
  - 用于运行 `paos agent`
  - 对应项目主包，Python 版本建议 `3.11`
- `phyagentos-go2-py39`
  - 用于运行 Go2 真机 Watchdog
  - 对应 `unitree_sdk2_python + cyclonedds`
  - 当前这台机器上验证通过的组合是 `Python 3.9`

原因：

- [pyproject.toml](../../pyproject.toml) 要求 `PhyAgentOS` 为 `Python >= 3.11`
- 当前本机验证通过的 `unitree_sdk2_python + cyclonedds` 组合在 `Python 3.9`

## 3. 依赖文件

仓库根目录已经提供两份 requirements：

- [requirements-phyagentos.txt](../../requirements-phyagentos.txt)
- [requirements-go2-py39.txt](../../requirements-go2-py39.txt)

建议安装方式：

```bash
cd ~/桌面/phyAgentOS/PhyAgentOS

conda create -n phyagentos python=3.11 -y
conda activate phyagentos
pip install -r requirements-phyagentos.txt

conda create -n phyagentos-go2-py39 python=3.9 -y
conda activate phyagentos-go2-py39
pip install -r requirements-go2-py39.txt
```

说明：

- `phyagentos` 会以可编辑模式安装当前仓库
- `phyagentos-go2-py39` 不需要安装 `PhyAgentOS` 主包，Watchdog 直接从当前源码目录运行
- `requirements-go2-py39.txt` 会以可编辑模式安装同级目录下的 `unitree_sdk2_python`

## 4. 启动前检查

在启动真机之前，请先确认：

1. Go2 已上电且网络连通
2. 电脑与 Go2 连接在正确网卡上
3. `examples/go2_unitree_sdk_bridge_config.json` 中的 `network_interface` 已改成真实网卡名
4. 当前场地足够安全，周围没有人和易碰撞物体
5. 你知道如何紧急停机，必要时随时中断 Watchdog 或直接人工接管

查看本机网卡名：

```bash
ip link
```

## 5. 真机配置文件

当前真机配置文件是：

- [go2_unitree_sdk_bridge_config.json](../../examples/go2_unitree_sdk_bridge_config.json)

关键字段：

- `target_navigation_backend: "real"`
- `unitree_sdk_path`
- `network_interface`
- `enable_live_camera`
- `enable_motion_control`
- `stand_up_on_connect`

建议你至少检查这两项：

```json
{
  "network_interface": "你的真实网卡名",
  "stand_up_on_connect": false
}
```

如果你希望 Watchdog 连上后自动站起，可以把 `stand_up_on_connect` 改成 `true`。

## 6. 真机启动步骤

### 6.1 终端 1：启动 Go2 Watchdog

```bash
conda activate phyagentos-go2-py39
cd ~/桌面/phyAgentOS/PhyAgentOS
python -m hal.hal_watchdog --driver go2_edu --driver-config examples/go2_unitree_sdk_bridge_config.json
```

这一步会：

- 启动 `go2_edu` driver
- 用 `real` backend 接入 `unitree_sdk2_python`
- 相机侧尝试初始化 `VideoClient`
- 运动侧尝试初始化 `SportClient`
- 持续把运行状态写回 `~/.PhyAgentOS/workspace/ENVIRONMENT.md`

### 6.2 终端 2：启动 Agent

```bash
conda activate phyagentos
cd ~/桌面/phyAgentOS/PhyAgentOS
paos agent
```

### 6.3 先做连接确认

进入 `paos agent` 后，建议先发：

```text
check connection
```

或者：

```text
what can you see
```

如果 Watchdog 运行正常，`ENVIRONMENT.md` 会持续更新。

## 7. 当前可直接使用的动作原语

当前对 Go2 已接通、且更适合真机验证的动作如下。

### 7.1 Primitive Motion

这是当前最直接的真机原语，Watchdog 在 `real` backend 下会调用：

- `SportClient.Move()`
- `SportClient.StopMove()`

支持的原语：

- `forward`
- `backward`
- `turn_left`
- `turn_right`
- `turn_around`

可在 Agent 中这样表达：

```text
move forward 0.5 meters
move backward 0.2 meters
turn left 90 degrees
turn right 45 degrees
turn around
stop
```

### 7.2 连接管理

支持：

- `connect_robot`
- `disconnect_robot`
- `reconnect_robot`
- `check_connection`

自然语言可这样说：

```text
connect the robot
reconnect the robot
check connection
disconnect the robot
```

### 7.3 其他已接入但应视为实验性能力

支持但目前应保守使用：

- `target_navigation`
- `semantic_navigate`
- `localize`

原因：

- 当前真机位姿推进仍然是系统内部近似状态，不是完整 odometry 闭环
- `target_navigation` 依赖实时视觉和环境快照，效果取决于相机输入质量
- `semantic_navigate` 仍依赖 `ENVIRONMENT.md` 与 `scene_graph`

## 8. 重要边界

当前真机模式已经接通的是：

- `unitree_sdk2_python`
- `VideoClient`
- `SportClient.Move()/StopMove()`

当前还没有完成的是：

- 真机楼梯步态控制器
- 低层 `LowCmd` 步态行走控制
- 真实 odometry 闭环驱动的精确位姿估计
- 机械臂操作

所以现在建议的真机实验目标是：

- 站立
- 短距离前进
- 短距离后退
- 左右转向
- 掉头
- 停止

而不是直接上楼梯。

## 9. 最小真机实验流程

建议按这个顺序：

1. 启动终端 1 的 Watchdog
2. 启动终端 2 的 Agent
3. 执行 `check connection`
4. 执行 `move forward 0.3 meters`
5. 执行 `turn left 90 degrees`
6. 执行 `stop`

如果这条链稳定，再继续扩大动作幅度。

## 10. 排查建议

### 10.1 Watchdog 启动后无法连接

重点检查：

- `network_interface` 是否正确
- Go2 是否上电
- 电脑是否在正确网段
- `unitree_sdk2_python` 是否可正常导入
- `cyclonedds` 是否已正确安装

### 10.2 Agent 能回复，但机器人不动

重点检查：

- Watchdog 是否真的运行在 `phyagentos-go2-py39`
- 配置文件中 `enable_motion_control` 是否为 `true`
- `paos agent` 写入的动作是否被 Watchdog 读取
- `SportClient` 初始化是否成功

### 10.3 有画面但不能运动

通常说明：

- `VideoClient` 通了
- `SportClient` 没通，或运动侧被禁用

### 10.4 能运动但环境感知不对

重点检查：

- `enable_live_camera`
- 当前光照、朝向与目标是否合适
- `ENVIRONMENT.md` 是否在持续刷新

## 11. 下一步建议

如果你的目标是“先把真机实验跑稳”，建议优先顺序如下：

1. 先验证 `forward / backward / turn_left / turn_right / stop`
2. 再验证相机输入是否稳定
3. 再尝试 `target_navigation`
4. 最后再讨论楼梯实验

如果你的目标是“楼梯实验”，那下一步不是继续配环境，而是进入控制器开发阶段。
