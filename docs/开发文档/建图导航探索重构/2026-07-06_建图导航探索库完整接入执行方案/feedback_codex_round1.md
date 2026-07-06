# codex round 1 反馈 6 项收口（2026-07-06）

> lane：建图/导航/探索重构 codex 第二轮审查 6 项返工
> 关联：`执行方案.md` / `feedback_phase7.md` / codex round 1 审查意见

## 收口清单（6 commit）

| # | commit    | 主题                                                                                  | hash        |
|---|-----------|---------------------------------------------------------------------------------------|-------------|
| 1 | `6bbb173` | fix(codex-P1) `PlatformGeometryProviderBundle` 默认探索路径迁 gRPC                      | `6bbb173`   |
| 2 | `7d408e6` | feat(codex-P2) Rust `lcm_runtime.rs` 读 `NUWAX_REAL_LCM_REQUIRED=true` 时启动期 exit 1  | `7d408e6`   |
| 3 | `0d3f2b9` | fix(codex-P3) `source_runtime_env.sh` 补 dotenv 解析 + 与 `start_gateway.sh` 一致        | `0d3f2b9`   |
| 4 | `9bbb3cd` | chore(codex-P4) `.env.example` 删 `NUWAX_GEOMETRY_RUNTIME_ARGS` + backend 默认改 fake-lcm| `9bbb3cd`   |
| 5 | `7cd2852` | docs(codex-P5) Phase 0-7 文档 `git add -f` 兜底（25 份 .md 入跟踪）                       | `7cd2852`   |
| 6 | `7b6266a` | chore(codex-P6) cargo clippy dropping_references / unused 警告收敛                        | `7b6266a`   |

---

## P1 详情：默认探索路径迁 gRPC

**根因**（codex 提的 6 项中第 1 项）：`PlatformGeometryProviderBundle.start_exploration / stop_exploration / get_exploration_state`（runtime.py:6841-6866 旧段）原本无条件调 `self.runtime.start_platform_exploration()` 等方法，而后者通过 `_legacy_exploration_placeholder_disabled` 旧 Python 状态机实现，与 Phase 5 `_resolve_exploration_backend` 的 gRPC facade 默认 owner 方向相反。

**修复**（services/geometry/runtime.py:6841+）：
- 新增 `_resolve_remote_exploration_provider()`：返回连通的 `_remote_provider` (GeometryRuntimeProvider)
- 新增 `_is_legacy_exploration_fallback_allowed()`：读 `NUWAX_EXPLORATION_LEGACY` env，与 runtime 层 `_is_exploration_legacy_enabled` 同步语义
- 新增 `_start_exploration_via_remote(request, remote)`：把 `ExploreAreaRequest` 翻译成 gRPC `start_exploration(request_id, strategy, arbitration, map_version_id)` 调用
- 新增 `_wrap_remote_exploration_state(state)`：把 gRPC dict ExplorationState 装成 `contracts.navigation.ExplorationState`（与 dimos nuwax_exploration 字符串语义对齐：accepted / running / succeeded / cancelled / failed / rejected / stopped）
- 重写 `start_exploration / stop_exploration / get_exploration_state / sync_exploration_after_task`：默认走 gRPC；legacy fallback 仅 `NUWAX_EXPLORATION_LEGACY=true` 显式打开
- gRPC 失败且 legacy 关闭时返回 `ExplorationState(IDLE, message="gRPC facade 无响应且 legacy 关闭", metadata={"gRPC_facade":"available_no_state"})` 或 `False`，绝不偷偷走旧状态机

**验收**：
- `rg "_legacy_exploration_placeholder_disabled" services/geometry/runtime.py` 仍 38 处
  - 4 处 docstring/comment（line 98-542）
  - 1 处 `__init__` 实例化（line 542，1 release 兼容字段保留）
  - 33 处 `sync_platform_exploration / _platform_exploration_loop / _ensure_platform_exploration_loop_locked / get_platform_exploration_state` 等 helpers 内部
  - **0 处** 在 `PlatformGeometryProviderBundle.start_exploration/stop_exploration/get_exploration_state` 入口直接调用（已全部迁 gRPC）

---

## P2 详情：Rust `NUWAX_REAL_LCM_REQUIRED` exit 1

**根因**：Python 端 `core/settings.py:267 REAL_LCM_REQUIRED` 落 `NUWAX_REAL_LCM_REQUIRED` 解析 + fail-fast 设计，但 Rust 端 `lcm_runtime.rs::LcmBackendKind::from_env()` 完全不读该 env；real-lcm 启动失败时只走 `Err(RealLcmBackendUnavailable)` → main.rs 软退出 (exit code = 4)，上游 systemd / start_gateway 仍可 restart-loop 绕过生产意图。

**修复**（services/geometry/nuwax_geometry_runtime/src/lcm_runtime.rs）：
- 新增 public 辅助函数 `real_lcm_required_from_env()`：解析 `NUWAX_REAL_LCM_REQUIRED`（接受 1/true/yes/on，大小写不敏感，默认 false）
- `spawn_geometry_runtime` 在 `LcmBackendKind::Real` 分支检测 `real_lcm_required`：
  - `true` → `std::process::exit(1)`，错误信息保留 stderr `LCM_BACKEND_UNAVAILABLE` 关键字供 start 脚本与 smoke test grep 识别
  - `false`（默认） → 维持现有 `return Err(RealLcmBackendUnavailable)` 行为（main.rs 软退出 exit code = 4）
- 仅在 binary 启动期 LCM transport 初始化失败时硬退出；运行期故障仍只走 `HealthCheck healthy=false`，不重复触发 exit

**验收**：
- `rg "NUWAX_REAL_LCM_REQUIRED" services/geometry/nuwax_geometry_runtime/src/` 非空（5+ 处）
- `cargo +1.88.0 test` 48 个测试全过

---

## P3 详情：source_runtime_env.sh 补 dotenv 解析

**根因**：原版用 `source ${env_file}` 处理 dotenv 格式（`KEY=val # comment` / 双引号包裹 / `export` 前缀）脆弱；生成自 installer 的 .env / web/.env / runtime_data/*.env 实际是 dotenv 格式（带 `# 注释` 与可选双引号），shell source 会把 `#` 后当注释丢值或把 `export FOO=bar` 的 `export` 字面量当成值。

**修复**（services/geometry/source_runtime_env.sh）：
- 用专用 dotenv 解析器替代 `source ${env_file}`：
  - 跳空行 / `#` 起头的纯注释行
  - 必须含 `=`，否则跳过
  - key/value 两侧 trim
  - 含 `" #" / ' #'` 形式的行内注释，先剥掉
  - 成对单/双引号包裹时去引号
  - 仅当 env 未设置（`${!key+x}` 为空）时 export，避免覆盖用户显式 export
- 与 `scripts/start_gateway.sh::load_env_file` 语义完全一致（同一 .env 两条路径加载结果一致）
- 加载顺序：根 .env → `runtime_data/geometry.env` → `runtime_data/.env` → `web/.env` → `NUWAX_GEOMETRY_EXTRA_ENV_FILES`

**验收**：
- `bash -n services/geometry/source_runtime_env.sh` syntax OK
- `bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh` syntax OK
- `bash -n scripts/start_gateway.sh` syntax OK
- 实际加载测试：`source services/geometry/source_runtime_env.sh` 打印 `==> 已加载 env: /home/ml/Deeprob/nuwax_robot_bridge/.env`，env 中无 NUWAX_LCM_BACKEND 是因为 .env.example 已改为 fake-lcm 但 .env 本身未生成（installer 流程；不影响解析逻辑）

---

## P4 详情：.env.example 清理

**修复**（.env.example）：
- 删除 line 245 `NUWAX_GEOMETRY_RUNTIME_ARGS=` 整行：
  - env-only 启动契约 Phase 0 已生效，nuwax-geometry-runtime 拒绝任何 CLI args（main.rs:49-83）
  - 继续保留会让 installer / 用户误以为能传 CLI args
- line 264 `NUWAX_LCM_BACKEND=real-lcm` → `fake-lcm`：
  - 与 Phase 0 fixup 的 `_normalize_lcm_backend` 在 ci/contract-test profile 下的归一化方向一致
  - 生产仍可显式 export real-lcm 触发 fail-fast
- 同步更新注释（"生产默认" → "CI / contract test 默认" + Phase 0 fail-fast 行为说明 + `NUWAX_REAL_LCM_REQUIRED=true` → exit 1 提示）

**验收**：
- `rg "NUWAX_GEOMETRY_RUNTIME_ARGS" .env.example web/.env runtime_data/ scripts/` 0 命中

---

## P5 详情：Phase 文档 git tracking 兜底

**根因**：docs/开发文档/建图导航探索重构/2026-07-06_建图导航探索库完整接入执行方案/ 下大部分 .md 文档未被 git ls-files 跟踪（`docs/开发文档` 在 .gitignore 内但**已被某些 P5 之前的 commit 显式 -f 加进**跟踪——P5 之前只有 phase4 / todolist_phase4 / feedback_phase4 三份被跟踪）。

**修复**：
- `git add -f docs/开发文档/建图导航探索重构/2026-07-06_建图导航探索库完整接入执行方案/`
- 25 份 .md 入跟踪：feedback_phase0/1/2/3/3_5/5/6/7、todolist_phase0/1/2/3/5/6/7、执行方案.md、接触面盘点_phase0、基线与契约冻结_phase0、LCM接入与wire-format验证_phase1、Rust运行时deps与typed_channel_phase2、建图接入mapping_phase3、探索接入explore_phase5、Python_gateway_service收敛_phase6、installer_systemd_实机验收_phase7
- 同步 strip 4 份 Phase 2 .md 文件的尾部空白（中文 PDR 习惯留尾随全角空格，不影响内容；让 `git diff --check HEAD~5..HEAD` 通过）

**验收**：
- `git ls-files docs/开发文档/建图导航探索重构/2026-07-06_建图导航探索库完整接入执行方案/ | wc -l` = 27（25 新加 + 2 原本就有）
- `git diff --check HEAD~5..HEAD` exit 0

---

## P6 详情：cargo clippy dropping_references / unused 警告收敛

**根因**（codex 提的 6 项中第 6 项）：cargo clippy 当前 59 warnings，重点是 `dropping_references` 类（提示 `let _ = ...`），不需要改核心逻辑。

**修复**（services/geometry/nuwax_geometry_runtime/src/）：
- `src/nav_runtime.rs:540` `drop(follower)` → `let _ = follower`，消除 clippy dropping_references 警告（drop 引用类型无操作；与原 drop 等价语义）
- `src/explore_runtime.rs:29 / 34` 给 `std::borrow::Cow / WavefrontInputReceiver` 加 `#[allow(unused_imports)]` 标注：
  - 保留 import 以便未来接入 wavefront input 时直接用
  - codex 约束：不要为修 clippy 改核心逻辑
- 未触及：nuwax-workspace 外部 crate（`nuwax-exploration` 内的 ExploreError 引用）— 不在修复范围
- 未触及：dead_code / type_complexity / too_many_arguments / needless_range_loop / doc_lazy_continuation / uninlined_format_args — 这些改核心逻辑或风格不在 codex P6 范围

**验收**：
- `cargo +1.88.0 clippy` warnings：bin 59 → 56（dropping_references 1 → 0，unused_imports 3 → 0）
- `cargo +1.88.0 test` 48 个测试全过
- `git diff --check HEAD~5..HEAD` exit 0

---

## 全量验收命令实际退出码

```bash
$ bash scripts/ci/no_ros2_residual.sh
[no_ros2_residual] PASS: ROS2 残留守门通过
exit 0

$ PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts
exit 0

$ bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh
exit 0
$ bash -n scripts/start_gateway.sh
exit 0
$ bash -n services/geometry/source_runtime_env.sh
exit 0

# 负面验证（rg exit 1 = 0 命中 = PASS）
$ ! rg -n '_nav2_action_provider.*return|...' services/geometry/runtime.py gateways/runtime.py
exit 0
$ ! rg -n '_resolve_exploration_backend.*_platform_exploration_runtime|...' services/.../runtime.py core/settings.py core/config_registry.py gateways/runtime.py
exit 0
$ ! rg -n 'NUWAX_GEOMETRY_RUNTIME_ARGS' .env.example web/.env runtime_data/ scripts/
exit 0

$ cargo +1.88.0 check --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked --all-targets
Finished `dev` profile [unoptimized + debuginfo] target(s) in 0.44s
exit 0

$ cargo +1.88.0 test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked
test result: ok. 48 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out
exit 0

$ cargo +1.88.0 clippy --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked --all-targets
warning: `nuwax-geometry-runtime` (bin "nuwax-geometry-runtime") generated 56 warnings
Finished `dev` profile [unoptimized + debuginfo] target(s) in 0.59s
exit 0  # 仅 warnings，不算失败；59 → 56

$ ! cargo tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml | rg "r2r|cyclonedds|rmw_|libddsc"
exit 0  # 0 命中，PASS

$ git ls-files docs/开发文档/建图导航探索重构/2026-07-06_建图导航探索库完整接入执行方案/ | wc -l
27

$ rg -n "NUWAX_REAL_LCM_REQUIRED" services/geometry/nuwax_geometry_runtime/src/
services/geometry/nuwax_geometry_runtime/src/lcm_runtime.rs:502 ...
5+ hits  # PASS

$ rg -n "_legacy_exploration_placeholder_disabled" services/geometry/runtime.py | wc -l
38  # 全部在 helpers / 1 release compat wrapper，Bundle 入口已迁 gRPC
```

---

## 关键事实 / 留尾

1. **P5 commit 复杂历史**：原本 P5 commit 0ab1f93 引入了尾部空白但未 strip；后续 77acc9b 是 strip 版本；中途误 amend 一次到 P6 后用 `git reset --hard 0ab1f93~1` 回退到 P4 重新走 `git checkout 77acc9b --` 恢复 + strip + 重 commit。最终链 9bbb3cd → 7cd2852 → 7b6266a。
2. **P6 故意未修 dead_code / type_complexity / too_many_arguments / uninlined_format_args**：codex P6 任务描述明确"不要为修 clippy 改核心逻辑；只加 let _ = ... / #[allow] 标注 / 类型推断提示"；uninlined_format_args 需要改 20 处 `format!("...{}", x)` → `format!("...{x}")`，本质是风格调整不在本批范围。
3. **P1 的 `_legacy_exploration_placeholder_disabled` 字段**：codex 明确"不要删除字段（兼容 1 release），但默认 caller 切换到 gRPC 路径"，已严格遵守。下一期 release 窗口可清理 33 处 helper 内部引用。
4. **cargo tree 0 命中 r2r/cyclonedds/rmw_/libddsc**：保持 drop-ros2 目标态。
5. **本次未实机验证**（按用户偏好，仅静态验证 + 单元测试）；nuwax-geometry-runtime 在目标机 Go2 Jetson（aarch64 / Ubuntu 20.04）上的实机启动期 `NUWAX_REAL_LCM_REQUIRED=true` exit 1 验证留给 phase7 收口。
