# NuwaxClaw 容器安装接入对齐记录

## 当前状态

本轮目标是把 NuwaxClaw Docker 客户端安装正式纳入 `scripts/bootstrap.py` 的一键部署主链。

## 已确认基线

1. 仓库当前公开部署入口只有 `scripts/bootstrap.py` 与 `scripts/start_gateway.sh`。
2. 既有 `bootstrap.py` 已支持：
   - 根 `.env` 生成
   - Go2 专属 `data_plane.env` 生成
   - Python 依赖安装
   - ROS2 workspace 构建
   - 资源准备
   - MCP 配置生成与 hook 安装
   - `systemd` 示例生成
3. 仓库内已经固定采用 `host.docker.internal:8766` 作为容器访问宿主机网关的正式地址。
4. `scripts/nuwaclaw_external_mcp_hook.py` 默认依赖 Docker 持久卷 `nuwax_home` 下的 `.nuwaxbot` 目录。

## 本轮实现结果

1. `scripts/bootstrap.py` 新增：
   - `--install-nuwaclaw-docker`
   - `--nuwaclaw-saved-key`
   - `--nuwaclaw-host-mount`
2. `wizard` 新增了容器安装开关、根目录挂载模式和个人密钥提问；当前 `wizard` 会直接按用户选择执行部署动作，不再只停在配置预览。
3. NuwaxClaw Docker 安装逻辑已正式落在 bootstrap 主链中，固定行为如下：
   - 容器名 `nuwax-client`
   - 持久卷 `nuwax_home`
   - 额外 host 解析 `host.docker.internal:host-gateway`
   - 端口映射 `6080:6080`、`60005:60005`、`60006:60006`
   - 宿主机根目录到 `/host` 的 `none / ro / rw` 三档挂载
4. 脚本会在打印 `docker run` 时对 `NUWAX_SAVED_KEY` 脱敏，避免把个人密钥明文打到终端日志。
5. 如果检测到已存在的 `nuwax-client`，bootstrap 会先删除旧容器再按当前参数重建；卷数据保留。
6. 当启用了 `mcp_mode=auto` 且本轮安装容器时，bootstrap 会等待一段时间让卷内 `.nuwaxbot` 初始化，再继续执行外部 MCP hook 安装，降低“容器刚起、proxy 脚本未落盘”导致的失败概率。
7. `部署文档.md` 和 `README.md` 已同步到当前行为，明确写出：
   - 新增 CLI 参数
   - 交互式向导会实际执行部署动作
   - `rw` 读写挂载具有风险
   - 等价 `docker run` 命令与端口说明

## 本轮验证

已执行：

1. `python3 -m py_compile scripts/bootstrap.py`
   - 结果：通过。
2. `PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q tests/unit/test_bootstrap.py`
   - 结果：`4 passed in 0.02s`
3. `python3 scripts/bootstrap.py check --robot customrobot --tier lite --no-geometry --memory-mode disabled --perception-mode disabled --mcp-mode skip --install-nuwaclaw-docker`
   - 结果：按预期暴露当前环境缺口：
     - `Docker CLI: 未找到 docker 可执行文件`
     - `NuwaxClaw 个人密钥: 需提供 --nuwaclaw-saved-key 或环境变量 NUWAX_SAVED_KEY`

## 当前已知约束

1. 自动 MCP hook 仍依赖宿主机可访问 `/var/lib/docker/volumes/nuwax_home/_data`；如果当前用户没有对应权限，auto 模式仍可能失败。
2. 本轮没有引入自定义容器名、自定义卷名或镜像地址覆盖；部署参数继续冻结在当前黄金路径上。
3. 当前会话环境没有可用的 `docker` 命令，因此无法在本轮里实际启动或重建 `nuwax-client` 容器；容器安装真实执行链已通过命令拼装单测覆盖。
