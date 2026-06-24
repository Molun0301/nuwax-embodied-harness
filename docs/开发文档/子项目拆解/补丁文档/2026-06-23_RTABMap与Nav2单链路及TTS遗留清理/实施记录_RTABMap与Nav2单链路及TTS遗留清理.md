# 实施记录：RTAB-Map + Nav2 单链路及 TTS 遗留清理

日期：2026-06-23  
执行目标：删除多后端配置/分支，仅保留 RTAB-Map 建图与 Nav2 action 导航单链路；删除 Doubao TTS、TTS player、日志转 TTS、文本播报 record-only 历史与音量初始值配置。

## 1. 本次删除范围

### 1.1 配置层

- 删除 `core/settings.py` 中的 TTS/player/Doubao/log-to-TTS 配置 dataclass 与 `load_config()` 读取。
- 删除几何栈与 Nav2 mode selector：几何栈固定为 `external_slam_nav2`，Nav2 固定为 action provider。
- RTAB-Map 配置只读取 `NUWAX_RTABMAP_*`，不再兼容 `NUWAX_BACKEND_*`。
- `core/config_registry.py` / `core/config_descriptions.json` / `.env.example` 删除：
  - Doubao：`DOUBAO_*`
  - TTS player：`TTS_PLAYER_*`
  - log-to-TTS：`TTS_LOG_BRIDGE_*`
  - 旧多后端：`NUWAX_BACKEND_*`
  - selector：`NUWAX_GEOMETRY_STACK`、`NUWAX_NAV2_MODE`
  - 音量初始值：未保留 `NUWAX_AUDIO_INITIAL_VOLUME` / `config.tts.initial_volume`

### 1.2 能力与网关层

- 删除 `AudioService.speak_text()`、record-only 文本历史、文本播报事件。
- 删除网关 `speak_text` capability descriptor、tool descriptor、handler 与动态可用性判断。
- `AudioService` 仅保留 Go2 VUI/软件音量控制。
- Debug log 面板不再读取 `config.log_tts.log_path` fallback。

### 1.3 几何/导航/定位层

- `PlatformGeometryRuntime` 导航策略固定为 `nav2_action`，删除 platform_grid/sidecar/mode 分支输出。
- `Nav2ActionNavigationProvider` 删除 `platform_grid_removed`、`grid_navigation_backend` 等旧状态字段。
- `start_rtabmap_sidecar.sh` 删除 `NUWAX_NAV2_MODE` case 分支，按 `NUWAX_NAV2_AUTOSTART` 启动 Nav2。
- 删除 localization 对 `slam_toolbox` 的状态归一、`/slam_toolbox/clear_changes` 调用与 `slam_toolbox_posegraph` 资产类型。
- 地图几何资产主字段改为 `rtabmap_database_path`；旧 `backend_state_path` 只在契约迁移入口中读取后丢弃。

## 2. GitNexus 影响分析

已按要求在编辑前对关键符号执行 impact，并保存到 `.omx/reports/impact-*.txt/json`。主要风险：

- `NuwaxRobotBridgeConfig` / TTS 相关 config：HIGH（配置扇出大）。
- `PlatformGeometryRuntime._uses_external_geometry_stack`：CRITICAL。
- `PlatformGeometryRuntime._resolve_navigation_backend`：HIGH。
- `PlatformGeometryRuntime._navigation_backend_policy`：MEDIUM。
- `MapGeometryAssetType` / `MapGeometryAssetBinding`：MEDIUM。
- `GatewayRuntime`：MEDIUM；`_refresh_dynamic_capability_matrix`：CRITICAL。

实施方式：小步删除、每步编译与静态扫描，避免扩大到未要求的机器人 VUI 音量控制和 frontier 探索规划器。

## 3. 验证结果

已通过：

```bash
python3 -m py_compile core/settings.py core/config_registry.py core/capability_registry.py \
  contracts/geometry_runtime.py contracts/map_workspace.py contracts/debug_views.py \
  services/audio/service.py services/geometry/config.py services/geometry/__init__.py \
  services/geometry/runtime.py services/geometry/nav2_action_provider.py \
  services/localization/service.py adapters/localization/geometry_state_adapter.py \
  gateways/runtime.py gateways/http/debug_panel.py skills/handlers/audio_skills.py \
  skills/map_workspace.py scripts/bootstrap.py tests/unit/test_config_io.py \
  tests/unit/test_memory_skills_audio.py tests/unit/test_bootstrap.py \
  tests/unit/test_navigation_services.py tests/unit/test_go2_ros2_workspace.py
bash -n services/geometry/start_rtabmap_sidecar.sh
git diff --check
```

配置与模板一致性检查通过：

```bash
registry/env clean 250 224
```

旧引用静态扫描无命中：

```bash
rg -n 'NUWAX_BACKEND_|BackendStateConfig|removed_platform_grid|platform_grid_removed|grid_navigation_backend|sidecar_launch_removed|_NAV2_SIDECAR_MODES|_REMOVED_PLATFORM_GRID_MODES|slam_toolbox|slam_toolbox_posegraph|TTS_LOG_BRIDGE_|DOUBAO_|TTS_PLAYER_|NUWAX_AUDIO_INITIAL_VOLUME|speak_text|record_only|TTSPlayerConfig|DoubaoConfig|TTSLogBridgeConfig|config\.log_tts|config\.tts|initial_volume|NUWAX_NAV2_MODE|NUWAX_GEOMETRY_STACK' core services adapters contracts gateways skills scripts tests .env.example -S
```

GitNexus detect-changes：

```text
Changes: 23 files, 85 symbols
Affected processes: 37
Risk level: critical
```

受环境依赖阻断：

- `python3 -m pytest -q tests/unit`：收集阶段失败，当前系统 Python 缺少 `pydantic`。
- `python3 -m pytest -q tests/integration`：收集阶段失败，当前系统 Python 缺少 `fastapi`。

## 4. 剩余风险

- 本次是跨配置、网关、几何运行时、契约和测试的大范围删除，GitNexus 判定整体 risk 为 critical；建议在安装完整依赖的项目虚拟环境中补跑 unit/integration。
- `rtabmap_db_path` 仍作为老地图版本兼容字段保留；新输出主字段为 `rtabmap_database_path`。
- Go2 VUI 音量控制仍保留；删除的是 TTS/player 初始音量配置与文本播报/历史能力。

## 5. 追加记录：Web 配置页残留清理（2026-06-23 晚间）

### 5.1 问题定位

用户截图中的「语音·豆包 TTS / 语音·播放与音量 / 语音·日志转 TTS」不应再出现在当前 schema。复核后确认：

- 当前 `core.config_registry.build_registry()` 已不再生成任何语音/TTS 分区。
- 当前 `core.config_io.build_schema()` 仅按 registry 组装字段，不会把 `.env` 中未知旧键重新变成前端字段。
- 实际残留来源是本地 `.env` 仍保留旧块，且运行中的网关/浏览器可能仍使用旧进程 schema 缓存。

### 5.2 本轮补充删除

- 清理本地 `.env`（该文件被 gitignore，不进入 git diff）：
  - 删除 `NUWAX_GEOMETRY_STACK`、`NUWAX_NAV2_MODE`。
  - 删除 `DOUBAO_*`、`TTS_PLAYER_*`、`TTS_LOG_BRIDGE_*` 及对应注释块。
  - 删除未被 settings 读取的 `GO2_ROBOT_IP` / `ROBOT_IP` 注释与键。
  - 删除启动初始音量与本地混音器死配置：`GO2_INIT_VOLUME_ON_START`、`GO2_VOLUME_PREFER_VUI`、`GO2_VOLUME_MIXER_*`、`GO2_AUDIO_COMMAND_TIMEOUT`。
  - 保留仍被代码读取的 Go2 VUI 音量控制：`GO2_VOLUME_AUTO_ENABLE_VUI_SWITCH`、`GO2_VUI_MIN_LEVEL`、`GO2_VUI_MAX_LEVEL`。
- 清理源码残留：
  - `core/settings.py` 删除 `NUWAX_LOG_LEVEL_TTS` 对应的 `tts` 子系统 logger 映射。
  - `scripts/check_layering.sh` 删除已不存在的顶层 `tts` 包治理正则。
  - `core/config_registry.py` / `core/config_descriptions.json` 删除无实际读取来源的 `ROBOT_IP` 展示/描述残留。
  - `drivers/go2/defaults.py` 删除 `tts/record_only` 注释残留。
- 同步当前用户可见文档：公开部署文档、公开架构说明、平台总体设计、Go2 SLAM 外部几何栈流程、技术手册、Web 配置面板相关当前文档，均删除旧 selector、TTS/Doubao/record-only/speak_text 说明；历史归档和本补丁审计记录保留为历史证据。

### 5.3 Web 配置页复核结论

使用包含依赖的 `.venv_model_download` 解释器复核：

```text
field_count 250
sections 17
retired_schema_hits []
```

当前配置页应只包含：机器人与运行入口、网关与服务、遥测、日志、几何话题/质量、RTAB-Map、Nav2、外部几何栈与桥接、感知、记忆与制品、Go2 数据面、Relay 等分区；不再包含任何语音/TTS/Doubao/log-to-TTS 分区。

如浏览器仍显示旧分区，需要重启宿主机网关并强制刷新前端页面；原因是旧 Python 进程仍持有修改前的 registry/schema。

### 5.4 本轮验证

通过项：

```bash
python3 -m py_compile core/settings.py core/config_registry.py drivers/go2/defaults.py tests/integration/test_config_api.py
bash -n scripts/check_layering.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv_model_download/bin/python -m pytest \
  tests/unit/test_config_io.py \
  tests/unit/test_env_example_completeness.py \
  tests/integration/test_config_api.py -q
# 结果：16 passed
```

配置模板一致性：

```text
field_count 250
core 224/224
go2 26/26
missing/extra 均为空
```

旧引用扫描（活动源码、`.env`、`.env.example`、当前用户可见文档；历史归档/补丁归档排除）无命中：

```bash
rg '语音·|豆包|Doubao|DOUBAO_|TTS_PLAYER_|TTS_LOG_BRIDGE_|\bTTS\b|tts|speak_text|record_only|record-only|initial_volume|NUWAX_AUDIO_INITIAL_VOLUME|NUWAX_NAV2_MODE|NUWAX_GEOMETRY_STACK|NUWAX_BACKEND_|GO2_INIT_VOLUME|GO2_VOLUME_PREFER_VUI|GO2_VOLUME_MIXER|GO2_AUDIO_COMMAND_TIMEOUT|GO2_ROBOT_IP|ROBOT_IP' ...
```

GitNexus 本轮增量检测：

```text
Changes: 5 files, 7 symbols
Affected processes: 0
Risk level: low
```

### 5.5 剩余风险 / 非本轮修复

- `bash scripts/check_layering.sh` 仍报告既有 R2c：`core/config_io.py` 直接导入 `drivers.go2.settings`。该违规早于本轮 TTS/selector 清理存在，本轮未通过改正架构或放宽规则来掩盖；后续可单独做配置 IO 分层整改。
- 系统 Python 仍缺少 `pydantic` / `fastapi`；本轮目标测试使用 `.venv_model_download` 成功执行。
