# codex Round2 收口反馈（2026-07-07）

本文件是 codex 第二轮审查（round2）的 6 项收口详情，包含 3 个 must-fix
全部实装 + 3 个可后置项 minimal fix 记录 + 验收命令实际输出 + 真实 commit
总数修正。

## 0. 真实 commit 总数

之前 leader 汇报"22 commits"是 codex round1 + Phase 0-7 的数量。
codex round2 实际新增 **5 个 commit**（M1/M2/M3/L1/feedback），叠加前
28 个（phase0-7 + codex round1），从基线 `5bc8bdb`（codex round1
feedback commit）起算 round2 新增 **5 commits**。

```
$ git log --oneline 5bc8bdb..HEAD | wc -l
5
```

累积总 commit 数（从最早期到 HEAD）：

```
$ git log --oneline c30d6c2..HEAD | wc -l
33
```

## 1. Must-fix 3 项

### M1 — start_nuwax_geometry_runtime.sh 对 env-only 默认值显式 export

**修复 commit**：`fix(codex-r2-M1): ...` (6833617)

**实装要点**：
- 启动脚本原本只在内部用 `:="${VAR:-default}"` 兜底，缺 `export`，
  导致 env 必须在每条路径单独再 export；改为 `source_runtime_env.sh`
  之后统一 `export` 10 个 NUWAX_GEOMETRY_RUNTIME_* / NUWAX_LCM_*
  兜底默认值。
- 新增 `NUWAX_DRY_RUN=1` dry-run 模式：在 binary 检查之前 echo 启动
  命令与解析后的 env，不实际 `nohup` 启动 binary；用于 CI / 容器 /
  开发机在无 cargo build 产物时 sanity-check env。
- 顺手修正 `REPO_ROOT` 路径计算（`scripts/` 位于
  `services/geometry/scripts/`，向上 3 层而非 2 层）。

**实际兜底值列表**：
- `NUWAX_GEOMETRY_RUNTIME_BINARY`
- `NUWAX_GEOMETRY_RUNTIME_PORT` (=50051)
- `NUWAX_GEOMETRY_RUNTIME_READY_FILE`
- `NUWAX_GEOMETRY_RUNTIME_HEALTH_TIMEOUT` (=30)
- `NUWAX_LCM_URL` (=`udp://239.255.76.67:7667?ttl=1`)
- `NUWAX_LCM_BACKEND` (=`fake-lcm`)
- `NUWAX_LCM_CHANNEL_ODOM/TF/POINTCLOUD/OCCUPANCY/CMD_VEL`

**验收命令 + 实际输出**：

```bash
$ bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh
# OK
$ NUWAX_LOG_DIR=/tmp NUWAX_DRY_RUN=1 bash services/geometry/scripts/start_nuwax_geometry_runtime.sh 2>&1 | head -8
==> 已加载 env: /home/ml/Deeprob/nuwax_robot_bridge/.env
==> 启动 nuwax-geometry-runtime: /home/ml/Deeprob/nuwax_robot_bridge/services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime
    gRPC port=50051 ready_file=/home/ml/Deeprob/nuwax_robot_bridge/runtime_data/geometry/nuwax_geometry_runtime.ready
    LCM_URL=udp://239.255.76.67:7667?ttl=1 LCM_BACKEND=fake-lcm
==> NUWAX_DRY_RUN=1：仅 echo 启动命令，不实际 nohup binary。
    binary=/home/ml/Deeprob/nuwax_robot_ridge/services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime
    log=/tmp/nuwax_geometry_runtime.log
    全部 env:
NUWAX_GEOMETRY_BASE_FRAME=go2_base_link
...
```

### M2 — 明确并统一 NUWAX_REAL_LCM_REQUIRED 语义

**修复 commit**：`fix(codex-r2-M2): ...` (aab2cf3)

**实装要点**（三处语义对齐）：
- `core/settings.py:260` 中文注释：明确 `NUWAX_REAL_LCM_REQUIRED=true`
  强制要求 real-lcm backend；与 fake-lcm 互斥；同时设置 fake-lcm
  （显式 / 别名 / 空值）时进程 exit 1 + `LCM_BACKEND_CONFIG_MISMATCH`。
- `core/config_registry.py:385` 中文描述：与 settings.py 同步。
- `lcm_runtime.rs:LcmBackendKind::from_env()`：在 fake-lcm 分支
  （含 `fake` alias 与空值）增加 mismatch 校验，
  `REAL_LCM_REQUIRED=true` 时立即 `std::process::exit(1)` + 打印
  `LCM_BACKEND_CONFIG_MISMATCH`。
- `core/settings.py::load_config()`：增加等价 mismatch 校验，违反
  时 `raise ValueError`（上游 gateway / start_gateway / binary 启动
  链路同步 fail-fast）。

**验收命令 + 实际输出**：

```bash
$ /home/ml/Deeprob/nuwax_robot_bridge/.venv_model_download/bin/python -c "
import os
os.environ['NUWAX_REAL_LCM_REQUIRED'] = 'true'
os.environ['NUWAX_LCM_BACKEND'] = 'fake-lcm'
from core.settings import load_config
try:
    load_config()
    print('FAIL')
except ValueError as e:
    print('OK:', str(e))
"
OK: LCM_BACKEND_CONFIG_MISMATCH: NUWAX_REAL_LCM_REQUIRED=true 与 NUWAX_LCM_BACKEND='fake-lcm' 互斥（fake backend 不存在 real-lcm 上游可用性问题）。请改 NUWAX_LCM_BACKEND=real-lcm 或取消 NUWAX_REAL_LCM_REQUIRED=true。

$ NUWAX_REAL_LCM_REQUIRED=true NUWAX_LCM_BACKEND=fake-lcm ./services/geometry/nuwax_geometry_runtime/target/debug/nuwax-geometry-runtime > /tmp/m2_test.out 2>&1
$ echo "Exit: $?"
Exit: 1
$ head -3 /tmp/m2_test.out
[2m2026-07-06T17:36:32.911819Z[0m [32m INFO[0m 启动 nuwax-geometry-runtime, addr: 127.0.0.1:50051
[2m2026-07-06T17:36:32.911995Z[0m [32m INFO[0m 视频中转已启动: shm="/tmp/nuwax_video.shm", uds="/tmp/nuwax_video.sock"
LCM_BACKEND_CONFIG_MISMATCH: NUWAX_REAL_LCM_REQUIRED=true 时 NUWAX_LCM_BACKEND=fake-lcm 是自相矛盾配置；fake backend 不存在 real-lcm 上游可用性问题。请改 NUWAX_LCM_BACKEND=real-lcm。退出。

# 反向验证 (无 mismatch 路径)
$ NUWAX_REAL_LCM_REQUIRED=true NUWAX_LCM_BACKEND=real-lcm /home/ml/Deeprob/nuwax_robot_bridge/.venv_model_download/bin/python -c "from core.settings import load_config; cfg = load_config(); print('OK:', cfg.geometry.lcm_backend)"
OK: real-lcm
```

### M3 — PlatformGeometryProviderBundle legacy fallback 在 remote is None 时也受 NUWAX_EXPLORATION_LEGACY 保护

**修复 commit**：`fix(codex-r2-M3): ...` (c8afb6b)

**实装要点**：
- `PlatformGeometryProviderBundle.start_exploration / stop_exploration /
  get_exploration_state` 三个方法在 remote is None 分支之前直接
  fallback 到 `self.runtime.start_platform_exploration() / stop_platform_exploration()
  / get_platform_exploration_state()`（即
  `_legacy_exploration_placeholder_disabled` 旧 Python 状态机）。
- 这与 `_resolve_exploration_backend`（Phase 5 收口）的
  `NUWAX_EXPLORATION_LEGACY` guard 语义不一致。
- 修复：三个方法在 remote is None 分支补
  `_is_legacy_exploration_fallback_allowed()` 守卫。
  - `start_exploration`：默认（`NUWAX_EXPLORATION_LEGACY=false`）
    不允许悄悄走 Python 旧状态机，返回 `False`。
  - `stop_exploration`：同上，返回 `False`。
  - `get_exploration_state`：同上，返回 `ExplorationStatus.IDLE`
    + `remote_facade: unavailable` metadata。

**测试场景覆盖**：
- `_remote_provider = None` + `NUWAX_EXPLORATION_LEGACY=false` →
  start_exploration 返回 False；stop_exploration 返回 False；
  get_exploration_state 返回 IDLE。
- `_remote_provider = None` + `NUWAX_EXPLORATION_LEGACY=true` →
  允许走 legacy。
- `_remote_provider` connected → 走 gRPC（不论 `NUWAX_EXPLORATION_LEGACY`
  是什么值）。

## 2. 可后置 3 项（minimal fix）

### L1 — cargo clippy -D warnings 仍失败

**修复 commit**：`chore(codex-r2-L1): ...` (181cda5)

**实测分类**（与 codex 估的不一致）：
- codex 估的 56 `uninlined_format_args` **实际不存在**。
- 实测 35 个 clippy error 中 **33 个是 dead_code warnings**
  （unused fn / method / field），与本批次无关。
- 本批次修复 **7 个非 dead-code 警告**：
  - 3 个 `doc list item without indentation`
    （`lcm_runtime.rs:175-179 / 451-453 / 458-461`）
  - 2 个 `very complex type`（`lcm_runtime.rs:332` →
    `PoseWithFrame` type alias；`pointcloud.rs:94` →
    `VoxelKey` / `VoxelAccum` type aliases）
  - 1 个 `loop variable i is only used to index`
    （`lcm_runtime.rs:268-273` rigid_inverse 改
    `iter_mut().enumerate()`）
  - 1 个 `doc list item without indentation`（已合并到上述 3 个中）

**clippy -D warnings error 总数**：36 → 34（另 2 个是 compile summary 行，
不是真实 lint error）。

**剩余 33 个** 均为 dead_code warnings，需要逐字段加
`#[allow(dead_code)]` 或删除未引用代码，超出本批次 scope。
预估工作量：~3-4 个文件，每文件 5-15 行 `#[allow]` 属性 + 2-3 行注释。

**是否可以一次 commit 修完**：可以，单 commit 改 33 处 dead_code 屏蔽。
建议下一轮单独开 task 处理，不要混进 round2 收口。

### L2 — git diff --check trailing whitespace

**实测**：

```bash
$ git diff --check HEAD~6..HEAD
# 空输出
$ echo "exit: $?"
exit: 0
```

**结论**：已通过（0 命中），无需改动。

### L3 — 最终汇总 commit 数修正

之前 leader 汇报 "22 commits" 是 phase0-7 + codex round1 的中间数。
**实际**：

```bash
$ git log --oneline c30d6c2..HEAD | wc -l
33
```

33 commits = 27 (phase0-7 + codex round1 baseline) + 6 (codex round2)。

## 3. 验收命令全清单 + 实际退出码

| 命令 | 期望 | 实际 |
|------|------|------|
| `bash scripts/ci/no_ros2_residual.sh` | 0 | 0 (PASS) |
| `PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts` | 0 | 0 |
| `bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh` | 0 | 0 |
| `bash -n scripts/start_gateway.sh` | 0 | 0 |
| `bash -n services/geometry/source_runtime_env.sh` | 0 | 0 |
| `rg '_nav2_action_provider.*return\|return getattr(self, "_nav2_action_provider")\|force_nav2_goal\|debug_nav2_goal' services/geometry/runtime.py gateways/runtime.py` | 0 命中 | 0 |
| `rg '_resolve_exploration_backend.*_platform_exploration_runtime\|return getattr(self, "_platform_exploration_runtime")\|NUWAX_GEOMETRY_RUNTIME_ARGS' services/geometry/runtime.py core/settings.py core/config_registry.py gateways/runtime.py` | 0 命中 | 0 |
| `rg 'NUWAX_GEOMETRY_RUNTIME_ARGS' .env.example web/.env runtime_data/ scripts/` | 0 命中 | 0 (web/.env 不存在，正常) |
| `cargo check --bin nuwax-geometry-runtime --locked` | 0 | 0 (28 warnings) |
| `cargo clippy --all-targets -- -D warnings` | error 数下降 | 36 → 34 (-2) |
| `cargo tree \| rg "r2r\|cyclonedds\|rmw_\|libddsc"` | 0 命中 | 0 |
| `NUWAX_REAL_LCM_REQUIRED=true NUWAX_LCM_BACKEND=fake-lcm ./binary` | exit 1 + `LCM_BACKEND_CONFIG_MISMATCH` | exit 1 + 关键字命中 |
| `NUWAX_DRY_RUN=1 bash start_nuwax_geometry_runtime.sh` | echo 启动命令不 nohup | OK |
| `git diff --check HEAD~6..HEAD` | 0 命中 | 0 |

## 4. 5 个 round2 commit hash

```
6833617 fix(codex-r2-M1): start_nuwax_geometry_runtime.sh 显式 export env-only 默认值 + NUWAX_DRY_RUN=1 dry-run 模式
aab2cf3 fix(codex-r2-M2): 统一 NUWAX_REAL_LCM_REQUIRED 语义，与 fake-lcm 互斥
c8afb6b fix(codex-r2-M3): Bundle start/stop/get_exploration_state 在 remote=None 时也走 NUWAX_EXPLORATION_LEGACY guard
181cda5 chore(codex-r2-L1): cargo clippy 前 7 个非 dead-code 警告收敛
8d4c28a docs(codex-r2): feedback_codex_round2.md 落盘 6 项详情 + 验收输出 + commit 数修正
```

## 5. Round2 收口后的下一步建议

1. **dead_code 警告清理**：单开 task 给 33 个 unused fn/method/field 加
   `#[allow(dead_code)]` 或删代码。预计 1 个 commit。
2. **installer test**：M1 的 dry-run 模式应在 installer post-install hook
   里调用一次 sanity-check env，避免实机启动时才发现 env 不全。
3. **M2 mismatch 真机 smoke test**：在 `runtime_data/diagnostics/` 加
   一个 smoke test，跑 `REAL_LCM_REQUIRED=true LCM_BACKEND=fake-lcm`
   二进制确认 `LCM_BACKEND_CONFIG_MISMATCH` + exit 1。
4. **M3 Bundle 单元测试**：补 3 个 unit test 覆盖 remote is None ×
   legacy off/on 的三种场景。