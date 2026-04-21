# nuwax_robot_bridge

`nuwax_robot_bridge` 是部署在机器人宿主机上的具身能力运行时。

它负责把机器人连接、状态采集、视觉感知、地图定位、导航、记忆、音频与制品通道整理成统一能力，并通过稳定接口提供给上层系统调用。

当前首个黄金路径是 Go2；`scripts/bootstrap.py` 已为 `go2`、`g1`、`customrobot` 预留统一部署入口。

## 快速开始

### 1. 克隆仓库并进入目录

```bash
git clone <your-repo-url>
cd nuwax_robot_bridge
```

### 2. 先做分层检查

```bash
python3 scripts/bootstrap.py check \
  --robot go2 \
  --tier full \
  --geometry \
  --data-plane \
  --memory-mode cloud \
  --perception-mode hybrid \
  --mcp-mode auto
```

`check` 会按当前机器人和能力组合检查 Python、pip、ROS2、CycloneDDS、MCP 模板等前置条件；未启用 Go2 data plane 或 geometry 时，不会强行要求 ROS2。

### 3. 选择部署方式

交互式向导：

```bash
python3 scripts/bootstrap.py wizard
```

无交互执行：

```bash
python3 scripts/bootstrap.py apply --non-interactive \
  --robot go2 \
  --tier full \
  --geometry \
  --data-plane \
  --memory-mode cloud \
  --perception-mode hybrid \
  --systemd \
  --mcp-mode auto
```

执行完成后会按当前选择生成：

- 根配置 `.env`
- Go2 专属配置 `drivers/go2/data_plane.env`
- 可选 `systemd` 示例
- 可选 MCP 配置文件

### 4. 手工启动网关

```bash
bash scripts/start_gateway.sh
```

`scripts/start_gateway.sh` 会读取根 `.env`，按 `NUWAX_ACTIVE_ROBOT` 加载机器人运行环境，并在启用 geometry 时加载平台 geometry 运行环境。

### 5. 验证接口

```bash
curl http://127.0.0.1:8766/api/health \
  -H 'Authorization: Bearer agent-robot-bridge-token'

curl http://127.0.0.1:8766/api/tools \
  -H 'Authorization: Bearer agent-robot-bridge-token'
```

## 常见部署组合

### Go2 全量能力

```bash
python3 scripts/bootstrap.py apply --non-interactive \
  --robot go2 \
  --tier full \
  --geometry \
  --data-plane \
  --memory-mode cloud \
  --perception-mode hybrid \
  --systemd \
  --mcp-mode auto
```

### Go2 轻量能力

```bash
python3 scripts/bootstrap.py apply --non-interactive \
  --robot go2 \
  --tier lite \
  --no-geometry \
  --no-data-plane \
  --memory-mode disabled \
  --perception-mode local \
  --mcp-mode config-only
```

### 自定义机器人最小入口

```bash
python3 scripts/bootstrap.py apply --non-interactive \
  --robot customrobot \
  --tier lite \
  --no-geometry \
  --memory-mode disabled \
  --perception-mode disabled \
  --mcp-mode skip
```

## 更多文档

- [部署文档.md](部署文档.md)
- [文档导航.md](docs/公开介绍文档/文档导航.md)
- [平台架构说明.md](docs/公开介绍文档/平台架构说明.md)
- [宿主机与容器通信说明.md](docs/公开介绍文档/宿主机与容器通信说明.md)
