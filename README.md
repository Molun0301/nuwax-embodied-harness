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

`check` 会按当前机器人和能力组合检查 Python、pip、MCP 模板等前置条件；默认项目 `.venv` 还会检查 `venv/ensurepip`。启用 geometry 或构建 geometry 工作区时才会额外检查 ROS2 与 CycloneDDS；启用 Go2 + geometry 时还会检查 Go2SLAM 工作区 `install/setup.bash`、源码新鲜度和关键 ROS2 包；启用 Go2 data plane 时则会检查 Go2 数据面模板。

### 3. 选择部署方式

交互式向导：

```bash
python3 scripts/bootstrap.py wizard
```

向导会按选择直接执行部署动作；默认会创建项目 `.venv` 并让 `scripts/start_gateway.sh` 使用它。启用 Go2 + geometry 时会构建平台 geometry 工作区和 Go2SLAM 嵌套工作区；如果选择安装 `NuwaxClaw` Docker 客户端，还会继续询问宿主机根目录挂载模式和个人密钥。

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

如果同时要安装 `NuwaxClaw` Docker 客户端，可追加：

```bash
  --install-nuwaclaw-docker \
  --nuwaclaw-host-mount ro \
  --nuwaclaw-saved-key <你的个人密钥>
```

脚本会自动附加 `--add-host=host.docker.internal:host-gateway`，并暴露 `6080`、`60005`、`60006` 端口。

执行完成后会按当前选择生成：

- 根配置 `.env`，包含 `NUWAX_PYTHON_BIN` 与检测到的 `NUWAX_ROS_SETUP_SCRIPT`
- Go2 专属配置 `drivers/go2/data_plane.env`
- 默认项目虚拟环境 `.venv`
- 可选 `systemd` 示例
- 可选 MCP 配置文件
- 可选 `NuwaxClaw` Docker 客户端容器

实机如果从 `main` 切换到 `go2_inner_slam`，或更新了 Go2SLAM launch/config/package 源码，必须重新构建 ROS2 工作区；不要复用旧分支留下的 `install/setup.bash`。

### 4. 手工启动网关

```bash
bash scripts/start_gateway.sh
```

`scripts/start_gateway.sh` 会读取根 `.env`，按 `NUWAX_PYTHON_BIN` 选择 Python 解释器，按 `NUWAX_ACTIVE_ROBOT` 加载机器人运行环境，并在启用 geometry 时加载平台 geometry 运行环境。

### 5. 验证接口

```bash
# 网关已完全移除鉴权，无需 Authorization header，直接 curl 即可
curl http://127.0.0.1:8766/api/health

curl http://127.0.0.1:8766/api/tools
```

### 6. 打开本机只读 Web 调试面板

启动宿主机网关后，可以在浏览器打开本机调试页：

```text
http://127.0.0.1:8766/debug
```

页面会从网关已加载的 `.env` 配置自动使用 `NUWAX_GATEWAY_AGENT_TOKENS` 中的 agent token（代理令牌）请求 `/api/debug/*` 只读接口。通过 SSH（安全外壳协议）隧道访问 Go2 时也继续打开本机地址：

```bash
ssh -L 8766:127.0.0.1:8766 unitree@<Go2_IP>
```

```text
http://127.0.0.1:8766/debug
```

如果网关运行在 Go2 真机上，并从同一局域网电脑直接访问 Go2 地址，请把 `127.0.0.1` 换成 Go2 的 IP（网络地址），并携带实际 agent token：

```text
http://<Go2_IP>:8766/debug?access_token=<实际 agent token>
```

该页面是只读调试面板，不包含运动控制、导航目标下发、探索启停、地图保存/删除等控制功能。面板会展示服务状态、运行事件/日志、2D SLAM（同步定位与建图）栅格、机器人位姿与轨迹、navigation/frontier（导航/探索前沿）摘要、点云元数据，以及数据时间戳、延迟和 stale（过期）状态。

常用只读调试 API（应用程序接口）：

```text
http://127.0.0.1:8766/api/debug/overview
http://127.0.0.1:8766/api/debug/map
http://127.0.0.1:8766/api/debug/localization
http://127.0.0.1:8766/api/debug/navigation
http://127.0.0.1:8766/api/debug/pointcloud/latest
http://127.0.0.1:8766/api/debug/logs/tail
```

兼容别名：

```text
http://127.0.0.1:8766/api/debug/snapshot
http://127.0.0.1:8766/api/debug/map/frame/latest
http://127.0.0.1:8766/api/debug/localization/pose/latest
http://127.0.0.1:8766/api/debug/localization/trajectory
http://127.0.0.1:8766/api/debug/exploration/frontier/latest
http://127.0.0.1:8766/api/debug/pointcloud/frame/latest/meta
```

真机上的地图/frontier P95（第 95 百分位延迟）、真实点云性能和 frontier 几何高亮仍需要在 Go2 环境中验收。

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

- [部署文档.md](docs/公开文档/部署文档.md)
- [文档导航.md](docs/公开介绍文档/文档导航.md)
- [平台架构说明.md](docs/公开介绍文档/平台架构说明.md)
- [宿主机与容器通信说明.md](docs/公开介绍文档/宿主机与容器通信说明.md)
