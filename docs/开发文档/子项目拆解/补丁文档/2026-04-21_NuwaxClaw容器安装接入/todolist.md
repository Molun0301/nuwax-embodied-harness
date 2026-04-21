# NuwaxClaw 容器安装接入

## 任务目标

把 `scripts/bootstrap.py` 从“只覆盖宿主机网关与 MCP 配置”补齐为“可直接安装或重建 NuwaxClaw Docker 客户端”的正式部署链路。

本轮目标包括：

1. 为 bootstrap 新增 NuwaxClaw Docker 客户端安装开关。
2. 接入个人密钥采集，并写入容器环境变量 `NUWAX_SAVED_KEY`。
3. 接入宿主机根目录到容器 `/host` 的挂载模式选择：`none / ro / rw`。
4. 在生成的 `docker run` 中补上平台需要的端口与 `host.docker.internal` 解析。
5. 同步更新 `部署文档.md` 与 `README.md`，避免部署入口与公开文档脱节。

## 路线冻结

1. 容器名固定为 `nuwax-client`。
2. 持久卷固定为 `nuwax_home`，不在本轮引入自定义卷名。
3. 平台需要的容器端口统一暴露为：
   - `6080:6080`
   - `60005:60005`
   - `60006:60006`
4. 宿主机根目录挂载只支持三档：
   - `none`
   - `ro`
   - `rw`
5. 如果发现已有同名容器，bootstrap 直接删除旧容器并按当前参数重建；持久化数据仍留在 `nuwax_home` 卷。
6. 自动 MCP hook 继续沿用既有 `scripts/nuwaclaw_external_mcp_hook.py`，不新增第二套容器注入机制。

## 子任务

### 1. bootstrap 参数与交互
- [x] 新增 `--install-nuwaclaw-docker`
- [x] 新增 `--nuwaclaw-saved-key`
- [x] 新增 `--nuwaclaw-host-mount`
- [x] `wizard` 新增容器安装、挂载模式和个人密钥提问

### 2. Docker 客户端安装主链
- [x] 生成正式 `docker run` 命令
- [x] 自动附加 `--add-host=host.docker.internal:host-gateway`
- [x] 自动暴露 `6080 / 60005 / 60006`
- [x] 支持 `none / ro / rw` 三档根目录挂载
- [x] 对 `NUWAX_SAVED_KEY` 做非空校验
- [x] 命令输出对个人密钥脱敏
- [x] 已存在同名容器时先删除再重建

### 3. 文档同步
- [x] 更新 `部署文档.md`
- [x] 更新 `README.md`
- [x] 新增本轮补丁文档

### 4. 验证
- [x] `python3 -m py_compile scripts/bootstrap.py`
- [x] `pytest tests/unit/test_bootstrap.py`
- [x] `python3 scripts/bootstrap.py check --robot customrobot --tier lite --no-geometry --memory-mode disabled --perception-mode disabled --mcp-mode skip --install-nuwaclaw-docker`

## 验收口径

1. bootstrap 能在交互式和无交互模式下完整表达 NuwaxClaw Docker 安装参数。
2. 容器安装命令必须同时覆盖官方镜像参数与平台所需端口、host gateway、根目录挂载模式。
3. 文档必须明确说明 `rw` 读写挂载风险。
4. 个人密钥不得以明文形式出现在脚本日志里。
